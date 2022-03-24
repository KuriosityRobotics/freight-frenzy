package com.kuriosityrobotics.firstforward.robot.util.PID;

import android.os.SystemClock;

public class NanoClock {
    public static long now() { // nanos
        return SystemClock.elapsedRealtimeNanos();
    }
}
