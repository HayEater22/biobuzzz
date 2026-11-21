package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/** Puts every hub in MANUAL bulk caching. Call clear() once at the top of every loop. */
public class BulkCache {

    private final List<LynxModule> hubs;

    public BulkCache(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void clear() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
