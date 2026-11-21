# Push the robot code to the Control Hub over its Wi-Fi, then switch back to your normal Wi-Fi.
#
# Usage (from the repo folder, in PowerShell):
#   .\push-to-robot.ps1
#   .\push-to-robot.ps1 -RobotSsid 19896-RC
# Or double-click push-to-robot.bat.
#
# The Control Hub must be powered on. If Mullvad VPN is connected, the script disconnects it
# (it blocks the robot's 192.168.43.x network) and reconnects it at the end.

param(
    [string]$RobotSsid = "19896-RC",
    [string]$RobotIp   = "192.168.43.1",
    [string]$Interface = "Wi-Fi",
    # Wi-Fi to return to if the current one can't be read (or we're already on the robot's).
    [string]$FallbackHomeSsid = "SHS_Guest",
    # Safety timer: a separate process forces the laptop back to the home Wi-Fi after this many
    # seconds, even if this script hangs or is killed.
    [int]$WatchdogSec = 240,
    # Leave Mullvad alone (use when Mullvad's "Local network sharing" is ON, so the robot is reachable with the VPN up).
    [switch]$KeepVpn
)

$ErrorActionPreference = "Continue"
Set-Location $PSScriptRoot

$adb = Join-Path $env:LOCALAPPDATA "Android\Sdk\platform-tools\adb.exe"
if (-not (Test-Path $adb)) { Write-Host "adb not found at $adb" -ForegroundColor Red; exit 1 }

# Fully stop adb on this laptop: drop Wi-Fi adb connections, stop the server, then kill every adb.exe
# that's left (from any program) and make sure none remain. Leftover adb processes/sessions are what
# break the connection to the robot.
function Reset-Adb([string]$when) {
    if (Get-Process adb -ErrorAction SilentlyContinue) {
        & $adb disconnect 2>&1 | Out-Null    # only while a server runs (with none, this would start one)
        & $adb kill-server 2>&1 | Out-Null
    }
    for ($i = 0; $i -lt 10; $i++) {
        $left = Get-Process adb -ErrorAction SilentlyContinue
        if (-not $left) { return }
        $left | Stop-Process -Force -ErrorAction SilentlyContinue
        Start-Sleep -Milliseconds 500
    }
    Write-Host "  warning: an adb process would not close ($when)" -ForegroundColor Yellow
}

# Talk to the robot's adb directly (without adb.exe): send an adb CONNECT packet and see if it answers.
# Used only after adb.exe fails, to tell whether the problem is the robot or this laptop.
function Test-RobotAdb([string]$ip, [int]$port, [int]$timeoutMs) {
    $client = New-Object System.Net.Sockets.TcpClient
    try {
        if (-not $client.ConnectAsync($ip, $port).Wait($timeoutMs)) { return "no TCP connection to port $port" }
        $stream = $client.GetStream()
        $stream.ReadTimeout = $timeoutMs
        $payload = [System.Text.Encoding]::ASCII.GetBytes("host::`0")
        $sum = 0
        foreach ($b in $payload) { $sum += $b }
        $cmd = 0x4e584e43                                              # "CNXN"
        $hdr = New-Object byte[] 24
        [BitConverter]::GetBytes([int]$cmd).CopyTo($hdr, 0)
        [BitConverter]::GetBytes([int]0x01000000).CopyTo($hdr, 4)       # protocol version
        [BitConverter]::GetBytes([int]4096).CopyTo($hdr, 8)             # max payload
        [BitConverter]::GetBytes([int]$payload.Length).CopyTo($hdr, 12)
        [BitConverter]::GetBytes([int]$sum).CopyTo($hdr, 16)            # byte-sum checksum
        [BitConverter]::GetBytes([int](-bnot $cmd)).CopyTo($hdr, 20)    # magic
        $stream.Write($hdr, 0, 24)
        $stream.Write($payload, 0, $payload.Length)
        $buf = New-Object byte[] 24
        $n = $stream.Read($buf, 0, 24)
        if ($n -lt 4) { return "robot closed the connection without answering - power-cycle the Control Hub" }
        $reply = [System.Text.Encoding]::ASCII.GetString($buf, 0, 4)
        return "robot answered '$reply', so the robot's adb works - the problem is on this laptop"
    } catch {
        $e = $_.Exception
        while ($e.InnerException) { $e = $e.InnerException }
        return "robot's adb did not answer ($($e.Message)) - power-cycle the Control Hub"
    } finally {
        $client.Close()
    }
}

# FIRST, while still on the normal Wi-Fi (VPN untouched): clear every old adb session and process.
Write-Host "== Clearing old adb sessions ==" -ForegroundColor Cyan
Reset-Adb "start"
Write-Host "adb cleared (no adb processes running)."

function Get-CurrentSsid {
    $line = netsh wlan show interfaces | Select-String '^\s*SSID\s*:' | Select-Object -First 1
    if ($line) { return ($line.ToString() -split ':', 2)[1].Trim() }
    return $null
}

function Wait-ForSsid([string]$ssid, [int]$seconds) {
    for ($i = 0; $i -lt $seconds; $i++) {
        if ((Get-CurrentSsid) -eq $ssid) { return $true }
        Start-Sleep -Seconds 1
    }
    return $false
}

# Windows only joins networks from its LAST scan, which can be stale ("The specific network is not
# available"). WlanScan asks every Wi-Fi adapter to rescan right now. It does not change any connection.
Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;
public static class WlanScanner {
    [DllImport("wlanapi.dll")] static extern int WlanOpenHandle(uint v, IntPtr r, out uint nv, out IntPtr h);
    [DllImport("wlanapi.dll")] static extern int WlanCloseHandle(IntPtr h, IntPtr r);
    [DllImport("wlanapi.dll")] static extern int WlanEnumInterfaces(IntPtr h, IntPtr r, out IntPtr list);
    [DllImport("wlanapi.dll")] static extern int WlanScan(IntPtr h, ref Guid g, IntPtr ssid, IntPtr ie, IntPtr r);
    [DllImport("wlanapi.dll")] static extern void WlanFreeMemory(IntPtr p);
    public static int ScanAll() {
        uint nv; IntPtr h; IntPtr list; int scanned = 0;
        if (WlanOpenHandle(2, IntPtr.Zero, out nv, out h) != 0) return 0;
        if (WlanEnumInterfaces(h, IntPtr.Zero, out list) == 0) {
            int n = Marshal.ReadInt32(list);
            // WLAN_INTERFACE_INFO_LIST: 2 DWORDs, then items of GUID(16) + WCHAR[256](512) + state(4) = 532 bytes
            for (int i = 0; i < n; i++) {
                Guid g = (Guid)Marshal.PtrToStructure(new IntPtr(list.ToInt64() + 8 + i * 532), typeof(Guid));
                if (WlanScan(h, ref g, IntPtr.Zero, IntPtr.Zero, IntPtr.Zero) == 0) scanned++;
            }
            WlanFreeMemory(list);
        }
        WlanCloseHandle(h, IntPtr.Zero);
        return scanned;
    }
}
"@

function Test-SsidVisible([string]$ssid) {
    $pattern = '^\s*SSID \d+ : ' + [regex]::Escape($ssid) + '\s*$'
    return [bool](netsh wlan show networks | Select-String $pattern)
}

# Rescan, wait until the robot's network shows up, then connect. Retries, because Windows sometimes
# falls straight back to the old network.
function Connect-ToSsid([string]$ssid, [string]$iface, [int]$attempts) {
    for ($a = 1; $a -le $attempts; $a++) {
        [WlanScanner]::ScanAll() | Out-Null
        $seen = $false
        for ($i = 0; $i -lt 8; $i++) {
            Start-Sleep -Seconds 1
            if (Test-SsidVisible $ssid) { $seen = $true; break }
        }
        if (-not $seen) { Write-Host "  attempt ${a}: $ssid not visible yet, rescanning..."; continue }
        netsh wlan connect name="$ssid" interface="$iface" | Out-Null
        if (Wait-ForSsid $ssid 10) { return $true }
        Write-Host "  attempt ${a}: saw $ssid but couldn't join, retrying..."
    }
    return $false
}

$homeSsid = Get-CurrentSsid
if (-not $homeSsid -or $homeSsid -eq $RobotSsid) {
    Write-Host "Couldn't read a usable current Wi-Fi ('$homeSsid'); will return to $FallbackHomeSsid." -ForegroundColor Yellow
    $homeSsid = $FallbackHomeSsid
}
Write-Host "Home Wi-Fi: $homeSsid"

# Build first, while we still have internet (Gradle may need to download things).
Write-Host "`n== Building ==" -ForegroundColor Cyan
& .\gradlew.bat :TeamCode:assembleDebug -q
if ($LASTEXITCODE -ne 0) { Write-Host "Build FAILED - not pushing." -ForegroundColor Red; exit 1 }

$mullvad = (Get-Command mullvad -ErrorAction SilentlyContinue).Source
if (-not $mullvad) {
    $p = Join-Path $env:ProgramFiles "Mullvad VPN\resources\mullvad.exe"
    if (Test-Path $p) { $mullvad = $p }
}
$vpnWasOn = $false

$ok = $false
$watchdog = $null
try {
    # Safety timer (separate hidden process): if we're still not home after $WatchdogSec seconds,
    # force the laptop back to the home Wi-Fi. Stopped in 'finally' once we're home normally.
    # It only acts if the laptop is still on the robot's Wi-Fi at that point.
    $robotRe = 'SSID\s+:\s+' + [regex]::Escape($RobotSsid) + '\s*$'
    $wdCmd = "Start-Sleep -Seconds $WatchdogSec; if ((netsh wlan show interfaces) -match '$robotRe') { netsh wlan connect name='$homeSsid' interface='$Interface' }"
    $wdEnc = [Convert]::ToBase64String([System.Text.Encoding]::Unicode.GetBytes($wdCmd))
    $watchdog = Start-Process powershell -WindowStyle Hidden -PassThru -ArgumentList "-NoProfile -EncodedCommand $wdEnc"

    if (-not $KeepVpn -and $mullvad -and ((& $mullvad status) -match '^Connected')) {
        Write-Host "`n== Disconnecting Mullvad VPN ==" -ForegroundColor Cyan
        & $mullvad disconnect | Out-Null
        $vpnWasOn = $true
        for ($i = 0; $i -lt 10; $i++) {
            if ((& $mullvad status) -match '^Disconnected') { break }
            Start-Sleep -Seconds 1
        }
    }

    Write-Host "`n== Connecting to $RobotSsid ==" -ForegroundColor Cyan
    if (-not (Connect-ToSsid $RobotSsid $Interface 6)) {
        throw "Could not join $RobotSsid after 6 tries. Is the robot on and close to the laptop?"
    }
    Write-Host "Joined $RobotSsid."

    $reached = $false
    for ($i = 0; $i -lt 15; $i++) {
        if (Test-Connection -ComputerName $RobotIp -Count 1 -Quiet) { $reached = $true; break }
        Start-Sleep -Seconds 1
    }
    if (-not $reached) { throw "Joined $RobotSsid but can't reach $RobotIp. Is the VPN off?" }
    Write-Host "Robot answers ping at $RobotIp."

    # Connect adb. Every retry starts from a completely fresh adb (all adb processes killed first).
    Write-Host "`n== Connecting adb ==" -ForegroundColor Cyan
    $adbOk = $false
    for ($a = 1; $a -le 3; $a++) {
        if ($a -gt 1) { Reset-Adb "retry"; Start-Sleep -Seconds 2 }
        $out = (& $adb connect "${RobotIp}:5555" 2>&1 | Out-String).Trim()
        $devices = (& $adb devices 2>&1 | Out-String)
        if ($devices -match ([regex]::Escape("${RobotIp}:5555") + '\s+device\b')) { $adbOk = $true; Write-Host "adb connected."; break }
        Write-Host "  adb attempt ${a}: $out"
    }
    if (-not $adbOk) {
        Reset-Adb "diagnose"
        Write-Host "  Checking the robot's adb directly..."
        throw "adb could not connect to ${RobotIp}:5555. Direct check: $(Test-RobotAdb $RobotIp 5555 4000)"
    }

    Write-Host "`n== Installing ==" -ForegroundColor Cyan
    & .\gradlew.bat --offline :TeamCode:installDebug -q
    if ($LASTEXITCODE -ne 0) { throw "Install failed." }
    $ok = $true
}
catch {
    Write-Host "`nERROR: $_" -ForegroundColor Red
}
finally {
    # Clean up adb BEFORE leaving the robot's Wi-Fi: a clean disconnect lets the robot close its side too,
    # then every adb process is stopped so nothing is left over for next time.
    Write-Host "`n== Closing adb ==" -ForegroundColor Cyan
    Reset-Adb "end"
    Write-Host "`n== Switching back to $homeSsid ==" -ForegroundColor Cyan
    $backHome = $false
    if ((Get-CurrentSsid) -eq $homeSsid) { $backHome = $true }
    else { $backHome = Connect-ToSsid $homeSsid $Interface 4 }
    if (-not $backHome) {
        # Last resort: plain connect without waiting for a scan.
        netsh wlan connect name="$homeSsid" interface="$Interface" | Out-Null
        $backHome = Wait-ForSsid $homeSsid 15
    }
    if ($backHome) {
        Write-Host "Back on $homeSsid."
        if ($watchdog -and -not $watchdog.HasExited) { Stop-Process -Id $watchdog.Id -Force -ErrorAction SilentlyContinue }
    } else {
        Write-Host "Could not reconnect to $homeSsid yet. The safety timer will try again; or reconnect manually." -ForegroundColor Yellow
    }
    if ($vpnWasOn) {
        Write-Host "== Reconnecting Mullvad VPN ==" -ForegroundColor Cyan
        & $mullvad connect | Out-Null
    }
}

if ($ok) { Write-Host "`nDONE - code is on the robot. Restart the Robot Controller app if the new OpModes don't show." -ForegroundColor Green; exit 0 }
else     { exit 1 }
