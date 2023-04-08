package frc.robot.ros.messages;

import edu.wpi.first.math.geometry.Pose3d;

public class Pose3dStamped {
    public long stamp = 0;
    public Pose3d pose = new Pose3d();

    public Pose3dStamped(long stamp, Pose3d pose) {
        this.stamp = stamp;
        this.pose = pose;
    }
}
