package frc.robot.ros.messages;

import edu.wpi.first.math.geometry.Pose2d;

public class Pose2dWithCovariance {
    public Pose2d pose = new Pose2d();
    public double covariances[] = new double[3];

    public Pose2dWithCovariance(Pose2d pose, double[] covariances) {
        this.pose = pose;
        this.covariances = covariances;
    }

    public Pose2dWithCovariance(Pose2d pose, double covariance_x, double covariance_y, double covariance_angle) {
        this.pose = pose;
        this.covariances = new double[] {covariance_x, covariance_y, covariance_angle};
    }
}
