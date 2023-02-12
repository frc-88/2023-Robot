package frc.robot.util.arm;

import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class ArmState {
    private final String name;
    private final DoublePreferenceConstant shoulderAngle;
    private final DoublePreferenceConstant elbowAngle;
    private final DoublePreferenceConstant wristAngle;

    ArmState(String name) {
        this.name = name;

        shoulderAngle = new DoublePreferenceConstant("Arm/States/" + name + "/Shoulder Angle", 0);
        elbowAngle = new DoublePreferenceConstant("Arm/States/" + name + "/Elbow Angle", 0);
        wristAngle = new DoublePreferenceConstant("Arm/States/" + name + "/Wrist Angle", 0);
    }

    public String getName() {
        return name;
    }

    public double getShoulderAngle() {
        return shoulderAngle.getValue();
    }

    public double getElbowAngle() {
        return elbowAngle.getValue();
    }

    public double getWristAngle() {
        return wristAngle.getValue();
    }
}
