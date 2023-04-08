package frc.robot.ros;

public class Constants {
    public static enum Joints {
        MODULE_0("base_link_to_wheel_0_joint"),
        MODULE_1("base_link_to_wheel_1_joint"),
        MODULE_2("base_link_to_wheel_2_joint"),
        MODULE_3("base_link_to_wheel_3_joint");
        public final String value;

        private Joints(final String frame_id) {
            this.value = frame_id;
        }
    }

    public static enum Frames {
        MAP("map"),
        ODOM("odom"),
        BASE_LINK("base_link"),
        IMU("imu");
        public final String value;

        private Frames(final String frame_id) {
            this.value = frame_id;
        }
    }
}
