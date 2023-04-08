package frc.robot.ros.messages;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Odometry2d {
    public Pose2d pose = new Pose2d();
    public ChassisSpeeds twist = new ChassisSpeeds();

    public Odometry2d(Pose2d pose, ChassisSpeeds twist) {
        this.pose = pose;
        this.twist = twist;
    }
}
