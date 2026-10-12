package org.firstinspires.ftc.teamcode.Utilities;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.LynxModule;
import com.skeletonarmy.marrow.BulkCachingSettings;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

public final class AutoBulkConfig {
    private AutoBulkConfig() {}

    @OnCreateEventLoop
    public static void configure(Context context, FtcEventLoop ftcEventLoop) {
        BulkCachingSettings.defaultMode = LynxModule.BulkCachingMode.AUTO;
    }
}
