package frc.robot.ros.messages;

public enum FrcMatchPeriod {
    DISABLED(0),
    AUTONOMOUS(1),
    TELEOP(2);
    public final int value;

    private FrcMatchPeriod(final int value) {
        this.value = value;
    }
}