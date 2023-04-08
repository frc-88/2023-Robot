package frc.robot.ros.messages;

public class FrcImu {
    public double roll = 0.0, pitch = 0.0, yaw = 0.0;
    public double yaw_rate = 0.0;
    public double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;

    public FrcImu(double roll, double pitch, double yaw, double yaw_rate, double accel_x, double accel_y, double accel_z) {
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
        this.yaw_rate = yaw_rate;
        this.accel_x = accel_x;
        this.accel_y = accel_y;
        this.accel_z = accel_z;
    }
}
