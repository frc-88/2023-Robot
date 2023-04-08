package frc.robot.ros;

import org.ros.message.Time;

import edu.wpi.first.wpilibj.RobotController;

public class RosTimeManager {
    private static double offsetTime = 0.0;
    private static long offsetLocalTime = 0;
    private static final double OUT_OF_SYNC_THRESHOLD = 0.1;

    public static Time getTime() {
        double relativeLocalTime = (getLocalTime() - offsetLocalTime) * 1e-6;
        return new Time(relativeLocalTime + offsetTime);
    }
    
    public static void sync(double remoteTime) {
        if (Math.abs(remoteTime - getTime().toSeconds()) > OUT_OF_SYNC_THRESHOLD) {
            offsetTime = remoteTime;
            offsetLocalTime = getLocalTime();
        }
    }
    public static void sync(Time remoteTime) {
        sync(remoteTime.toSeconds());
    }
    
    public static long getLocalTime() {
        return RobotController.getFPGATime();
    }
    
    public static long asLocalTime(Time remoteTime) {
        return (long)((remoteTime.toSeconds() - offsetTime) * 1e6) + offsetLocalTime;
    }
}
