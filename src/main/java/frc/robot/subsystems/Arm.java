package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import com.ctre.phoenix.music.Orchestra;
import com.fasterxml.jackson.databind.jsontype.impl.AsDeductionTypeDeserializer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.arm.ArmJoint;
import frc.robot.util.arm.ArmState;
import frc.robot.util.arm.ArmStates;

public class Arm extends SubsystemBase {
    
    private final DigitalInput coastButton;

    public final ArmJoint m_shoulder;
    public final ArmJoint m_elbow;
    public final ArmJoint m_wrist;
    public final Translation2d shoulderPosition;

    private ArmState targetArmState;
    private List<ArmJoint> allJoints;

    public Arm() {
        this.coastButton = new DigitalInput(0);
        SmartDashboard.putBoolean("Coast Arm", false);

        m_shoulder = new ArmJoint("Shoulder", Constants.SHOULDER_ID, Constants.SHOULDER_ENCODER_ID, true, true, (16./42.) * (1./49.), 19.5);
        m_elbow = new ArmJoint("Elbow", Constants.ELBOW_ID, Constants.ELBOW_ENCODER_ID, true, true, (16./42.) * (1./49.), 15.5);
        m_wrist = new ArmJoint("Wrist", Constants.WRIST_ID, Constants.WRIST_ENCODER_ID, true, true, (16./42.) * (1./49.), 25.5);
        shoulderPosition = new Translation2d(18.5, 3);

        targetArmState = ArmStates.stow;
        allJoints = Arrays.asList(new ArmJoint[]{m_shoulder, m_elbow, m_wrist});
    }

    public Translation2d getGrabberPosition(Translation2d shoulder, Translation2d elbow, Translation2d wrist) {
        return shoulderPosition.plus(shoulder).plus(elbow).plus(wrist);
    }

    public Translation2d getGrabberPosition() {
        return getGrabberPosition(m_shoulder.getPositionVector(), m_elbow.getPositionVector(), m_wrist.getPositionVector());
    }

    public boolean isValidState(ArmState armState) {    
        Translation2d grabberPosition = getGrabberPosition(
            new Translation2d(m_shoulder.getLength(), new Rotation2d(armState.getShoulderAngle())),
            new Translation2d(m_elbow.getLength(), new Rotation2d(armState.getElbowAngle())),
            new Translation2d(m_wrist.getLength(), new Rotation2d(armState.getWristAngle())));
        if (grabberPosition.getX() > 48 || grabberPosition.getY() > 78) return false;
        if (grabberPosition.getX() < -48 || grabberPosition.getY() < 0) return false;
        if (armState.getShoulderAngle() < 0 || armState.getShoulderAngle() > 180) return false;
        if (armState.getElbowAngle() < -360 || armState.getElbowAngle() > 360) return false;
        if (armState.getWristAngle() < -360 || armState.getWristAngle() > 360) return false;
        if ((grabberPosition.getX() > -2.5 && grabberPosition.getX() < 29.5) && grabberPosition.getY() < 20) return false;
        return true;
    }

    public void goToArmState(ArmState armState) {
        if (!isValidState(armState)) {
            System.out.println("Arm state invalid");
            return;
        }
        targetArmState = armState;
        double greatestAngle = Collections.max(Arrays.asList(new Double[]{
            Math.abs(armState.getShoulderAngle()-m_shoulder.getAngle()), 
            Math.abs(armState.getElbowAngle()-m_elbow.getAngle()), 
            Math.abs(armState.getWristAngle()-m_wrist.getAngle())}));
        // Keep max velocities the same or change this
        m_shoulder.setMotionMagic(armState.getShoulderAngle(), m_shoulder.getMaxVelocity() * Math.abs(m_shoulder.getAngle() - armState.getShoulderAngle()) / greatestAngle);
        m_elbow.setMotionMagic(armState.getElbowAngle(), m_elbow.getMaxVelocity() * Math.abs(m_elbow.getAngle() - armState.getElbowAngle()) / greatestAngle);
        m_wrist.setMotionMagic(armState.getWristAngle(), m_wrist.getMaxVelocity() * Math.abs(m_wrist.getAngle() - armState.getWristAngle()) / greatestAngle);
    }

    public boolean isAtTarget() {
        if (!(m_shoulder.isOnTarget(targetArmState.getShoulderAngle()))) return false;
        if (!(m_elbow.isOnTarget(targetArmState.getElbowAngle()))) return false;
        if (!(m_wrist.isOnTarget(targetArmState.getWristAngle()))) return false;
        return true;
    }

    public boolean coastModeEnabled() {
        return DriverStation.isDisabled() && (!coastButton.get() || SmartDashboard.getBoolean("Coast Arm", false));
    }

    // COMMAND FACTORIES

    public CommandBase calibrateShoulderFactory() {
        return new InstantCommand(() -> m_shoulder.calibrateAbsolute(90)).ignoringDisable(true);
    }

    public CommandBase calibrateElbowFactory() {
        return new InstantCommand(() -> m_elbow.calibrateAbsolute(90)).ignoringDisable(true);
    }

    public CommandBase calibrateWristFactory() {
        return new InstantCommand(() -> m_wrist.calibrateAbsolute(90)).ignoringDisable(true);
    }

    public CommandBase sendArmToState(ArmState armState) {
        return new RunCommand(() -> goToArmState(armState), this);
    }

    public CommandBase sendArmToStateAndEnd(ArmState armState) {
        return sendArmToState(armState).until(this::isAtTarget);
    }

    public void addToOrchestra(Orchestra m_orchestra) {
        m_orchestra.addInstrument(m_elbow.getMotor());
        m_orchestra.addInstrument(m_shoulder.getMotor());
        m_orchestra.addInstrument(m_wrist.getMotor());
    }

    @Override
    public void periodic() {
        allJoints.forEach(ArmJoint::zeroRelative);
        
        if (coastModeEnabled()) {
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