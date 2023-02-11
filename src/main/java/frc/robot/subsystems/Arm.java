package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.arm.ArmJoint;

public class Arm extends SubsystemBase {
    
    private final DigitalInput coastButton;

    public final ArmJoint shoulder;
    public final ArmJoint elbow;
    public final ArmJoint wrist;

    private List<ArmJoint> allJoints;

    public Arm() {
        this.coastButton = new DigitalInput(0);
        SmartDashboard.putBoolean("Coast Arm", false);

        shoulder = new ArmJoint("Shoulder", Constants.SHOULDER_ID, Constants.SHOULDER_ENCODER_ID, false, false, (16./42.) * (1./49.), 19.5);
        elbow = new ArmJoint("Elbow", Constants.ELBOW_ID, Constants.ELBOW_ENCODER_ID, false, false, (16./42.) * (1./49.), 15.5);
        wrist = new ArmJoint("Wrist", Constants.WRIST_ID, Constants.WRIST_ENCODER_ID, false, false, (16./42.) * (1./49.), 25.5);

        allJoints = Arrays.asList(new ArmJoint[]{shoulder, elbow, wrist});
    }

    public void calibrate() {
        allJoints.forEach(ArmJoint::calibrateAbsolute);
    }

    // COMMAND FACTORIES

    public CommandBase calibrateFactory() {
        return new InstantCommand(this::calibrate).ignoringDisable(true);
    }

    @Override
    public void periodic() {
        allJoints.forEach(ArmJoint::zeroRelative);
        
        if (DriverStation.isDisabled() && (coastButton.get() || SmartDashboard.getBoolean("Coast Arm", false))) {
            allJoints.forEach(ArmJoint::coast);
        }
        else {
            allJoints.forEach(ArmJoint::brake);
        }

        allJoints.forEach((ArmJoint j) -> SmartDashboard.putNumber(j.getName() + " Angle", j.getAngle()));
        allJoints.forEach((ArmJoint j) -> SmartDashboard.putNumber(j.getName() + " Absolute Angle", j.getAbsoluteAngle()));
        allJoints.forEach((ArmJoint j) -> SmartDashboard.putBoolean(j.getName() + " Is Zeroed", j.isZeroed()));
    }
}