package frc.robot.util.coprocessor.detections;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;

public class Detection {
    // All poses are assumed to be in the base_link frame
    // (center of the robot projected onto the ground)

    private String name;
    private int index;
    private Pose3d pose = new Pose3d();

    public Detection(String name, int index, Pose3d pose) {
        this.name = name;
        this.index = index;
        this.pose = pose;
    }
    
    public Detection(
        String name,
        int index,
        double position_x,
        double position_y,
        double position_z,
        double orientation_w,
        double orientation_x,
        double orientation_y,
        double orientation_z
    ) {
        this.name = name;
        this.index = index;
        pose = new Pose3d(
            position_x,
            position_y,
            position_z,
            new Rotation3d(new Quaternion(
                orientation_w,
                orientation_x,
                orientation_y,
                orientation_z)
            )
        );
    }

    public String getName() {
        return name;
    }

    public int getIndex() {
        return index;
    }

    public Pose3d getPose() {
        return pose;
    }

    public double getX() {
        return pose.getX();
    }

    public double getY() {
        return pose.getY();
    }

    public double getZ() {
        return pose.getZ();
    }

    public void setX(double x) {
        pose = new Pose3d(x, pose.getY(), pose.getZ(), pose.getRotation());
    }

    public void setY(double y) {
        pose = new Pose3d(pose.getX(), y, pose.getZ(), pose.getRotation());
    }

    public void setZ(double z) {
        pose = new Pose3d(pose.getX(), pose.getY(), z, pose.getRotation());
    }
}
