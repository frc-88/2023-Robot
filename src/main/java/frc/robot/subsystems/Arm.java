package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.arm.ArmJoint;
import frc.robot.util.arm.ArmState;
import frc.robot.util.arm.ArmStates;

public class Arm extends SubsystemBase {
    
    private final DigitalInput m_coastButton;

    public final ArmJoint m_shoulder;
    public final ArmJoint m_elbow;
    public final ArmJoint m_wrist;
    public final Translation2d m_shoulderPosition;

    private ArmState m_targetArmState = ArmStates.stow;
    private List<ArmJoint> m_allJoints;
    private CommandBase m_stowCommand = new RunCommand(() -> goToArmState(ArmStates.stow));

    private double m_aimX = 0;

    public Arm() {
        this.m_coastButton = new DigitalInput(0);
        SmartDashboard.putBoolean("Coast Arm", false);

        m_shoulder = new ArmJoint("Shoulder", Constants.SHOULDER_ID, Constants.SHOULDER_ENCODER_ID, true, true, (16./42.) * (1./49.), 19.5, 270);
        m_elbow = new ArmJoint("Elbow", Constants.ELBOW_ID, Constants.ELBOW_ENCODER_ID, true, true, (16./42.) * (1./49.), 15.5, 90);
        m_wrist = new ArmJoint("Wrist", Constants.WRIST_ID, Constants.WRIST_ENCODER_ID, true, true, (16./42.) * (1./49.), 25.5, 270);
        m_shoulderPosition = new Translation2d(3, 18.5);

        m_targetArmState = ArmStates.stow;
        m_allJoints = Arrays.asList(new ArmJoint[]{m_shoulder, m_elbow, m_wrist});
    }

    public Translation2d getGrabberPosition(Translation2d shoulder, Translation2d elbow, Translation2d wrist) {
        return m_shoulderPosition.plus(shoulder).plus(elbow).plus(wrist);
    }

    public Translation2d getGrabberPosition() {
        return getGrabberPosition(m_shoulder.getPositionVector(), m_elbow.getPositionVector(), m_wrist.getPositionVector());
    }

    private Translation2d getWristPosition(double shoulderAngle, double elbowAngle) {
        return new Translation2d(shoulderAngle, new Rotation2d(Math.toRadians(m_shoulder.getLength())))
                .plus(new Translation2d(m_elbow.getLength(), new Rotation2d(Math.toRadians(elbowAngle))));
    }

    public double getMaxAim(double shoulderAngle, double elbowAngle) {
        Translation2d position = getWristPosition(shoulderAngle, elbowAngle);
        return Math.sqrt(Math.pow(m_shoulder.getLength() + m_elbow.getLength(), 2) - Math.pow(position.getY(), 2)) - position.getX();
    }

    private double[] getAimedAngles(double shoulderAngle, double elbowAngle) {
        Translation2d originalPosition = getWristPosition(shoulderAngle, elbowAngle);

        double l1 = m_shoulder.getLength();
        double l2 = m_elbow.getLength();
        double x = originalPosition.getX() + m_aimX;
        double y = originalPosition.getY();
        double q1;
        double q2;

        double q2_interim = (x*x + y*y - l1*l1 - l2*l2) / (2.*l1*l2);
        if (q2_interim > 1 || q2_interim < 0) {
            return new double[]{shoulderAngle, elbowAngle};
        }

        if (elbowAngle > shoulderAngle) {
            q2 = Math.acos(x);
            q1 = Math.atan(y / x) - Math.atan((l2*Math.sin(q2)) / (l1 + l2*Math.cos(q2)));
        } else {
            q2 = -Math.acos(x);
            q1 = Math.atan(y / x) + Math.atan((l2*Math.sin(q2)) / (l1 + l2*Math.cos(q2)));
        }

        return new double[]{Math.toDegrees(q1), Math.toDegrees(q1 + q2)};
    }

    public void setAim(double x) {
        m_aimX = x;
    }

    public boolean isValidState(ArmState armState) {    
        Translation2d grabberPosition = getGrabberPosition(
            new Translation2d(m_shoulder.getLength(), new Rotation2d(Math.toRadians(armState.getShoulderAngle()))),
            new Translation2d(m_elbow.getLength(), new Rotation2d(Math.toRadians(armState.getElbowAngle()))),
            new Translation2d(m_wrist.getLength(), new Rotation2d(Math.toRadians(armState.getWristAngle()))));
        
        if (grabberPosition.getY() > 78) {
            System.out.println("Exceeded height limit with position: " + grabberPosition);
            return false;
        }
        if (grabberPosition.getY() < 0) {
            System.out.println("Exceeded floor limit with position: " + grabberPosition);
            return false;
        }
        if (grabberPosition.getX() > 48 + 27 || grabberPosition.getX() < -48) {
            System.out.println("Exceeded extension limit with position: " + grabberPosition);
            return false;
        }
        if (armState.getShoulderAngle() < 0 || armState.getShoulderAngle() > 180) {
            System.out.println("Exceeded shoulder limit with angle: " + armState.getShoulderAngle());
            return false;
        }
        if (armState.getElbowAngle() < -360 || armState.getElbowAngle() > 360) {
            System.out.println("Exceeded elbow limit with angle: " + armState.getElbowAngle());
            return false;
        }
        if (armState.getWristAngle() < -360 || armState.getWristAngle() > 360) {
            System.out.println("Exceeded wrist limit with angle: " + armState.getWristAngle());
            return false;
        }
        if ((grabberPosition.getX() > -2.5 && grabberPosition.getX() < 29.5) && grabberPosition.getY() < 20) {
            System.out.println("Exceeded chassis hit limit with position: " + grabberPosition);
            return false;
        }
        return true;
    }

    public void goToArmState(ArmState armState) {
        if (!isValidState(armState)) return;

        double shoulderAngle = armState.getShoulderAngle();
        double elbowAngle = armState.getElbowAngle();
        double wristAngle = armState.getWristAngle();

        // double[] aimedAngles = getAimedAngles(shoulderAngle, elbowAngle);
        // shoulderAngle = aimedAngles[0];
        // elbowAngle = aimedAngles[1];

        double greatestAngle = Collections.max(Arrays.asList(new Double[]{
            Math.abs(shoulderAngle-m_shoulder.getAngle()), 
            Math.abs(elbowAngle-m_elbow.getAngle()), 
            Math.abs(wristAngle-m_wrist.getAngle())}));

        if (greatestAngle > 2) {
            // Keep max velocities the same or change this
            m_shoulder.setMotionMagic(shoulderAngle, m_shoulder.getMaxVelocity() * Math.abs(m_shoulder.getAngle() - shoulderAngle) / greatestAngle);
            m_elbow.setMotionMagic(elbowAngle, m_elbow.getMaxVelocity() * Math.abs(m_elbow.getAngle() - elbowAngle) / greatestAngle);
            m_wrist.setMotionMagic(wristAngle, m_wrist.getMaxVelocity() * Math.abs(m_wrist.getAngle() - wristAngle) / greatestAngle);
        } else {
            m_shoulder.setMotionMagic(shoulderAngle);
            m_elbow.setMotionMagic(elbowAngle);
            m_wrist.setMotionMagic(wristAngle);
        }
    }

    public boolean isAtTarget(ArmState state) {
        if (!(m_shoulder.isOnTarget(state.getShoulderAngle()))) return false;
        if (!(m_elbow.isOnTarget(state.getElbowAngle()))) return false;
        if (!(m_wrist.isOnTarget(state.getWristAngle()))) return false;
        return true;
    }

    public boolean isAtTarget(ArmState state, double tolerance) {
        if (!(m_shoulder.isOnTarget(state.getShoulderAngle(), tolerance))) return false;
        if (!(m_elbow.isOnTarget(state.getElbowAngle(), tolerance))) return false;
        if (!(m_wrist.isOnTarget(state.getWristAngle(), tolerance))) return false;
        return true;
    }

    public boolean isStowed() {
        return m_targetArmState.getName().equals("Stow") && isAtTarget(m_targetArmState);
    }

    public boolean coastModeEnabled() {
        return DriverStation.isDisabled() && (!m_coastButton.get() || SmartDashboard.getBoolean("Coast Arm", false));
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

    private CommandBase chainIntermediaries(CommandBase initialCommand, List<ArmState> intermediaries, double tolerance) {
        CommandBase command = initialCommand;
        for (ArmState intermediary : intermediaries) {
            command = new RunCommand(() -> goToArmState(intermediary), this)
                .until(() -> isAtTarget(intermediary, tolerance))
                .andThen(command);
        }
        return command;
    }

    private CommandBase sendArmToState(Supplier<ArmState> armState, BooleanSupplier until) {
        if (armState.get().isStow()) {
            CommandBase command = new ProxyCommand(() -> m_stowCommand).until(until);
            command.addRequirements(this);
            return command;
        } else {

            ArmState state = armState.get();
            CommandBase command = chainIntermediaries(
                new RunCommand(() -> goToArmState(state), this).until(until),
                state.getDeployIntermediaries(),
                state.getDeployIntermediaryTolerance()
            );
            command = new InstantCommand(() -> {m_targetArmState = state;}).alongWith(command);

            m_stowCommand = chainIntermediaries(
                new RunCommand(() -> goToArmState(ArmStates.stow)),
                state.getRetractIntermediaries(),
                state.getRetractIntermediaryTolerance()
            );
            m_stowCommand = new InstantCommand(() -> {m_targetArmState = ArmStates.stow;}).alongWith(m_stowCommand);

            return command;
        }
    }

    public CommandBase holdTargetState() {
        return new RunCommand(() -> goToArmState(m_targetArmState), this);
    }

    public CommandBase sendArmToState(Supplier<ArmState> armState) {
        return sendArmToState(armState, () -> false);
    }

    public CommandBase sendArmToStateAndEnd(Supplier<ArmState> armState) {
        return sendArmToState(armState, () -> isAtTarget(armState.get()));
    }

    public CommandBase sendArmToState(ArmState armState) {
        return sendArmToState(() -> armState);
    }

    public CommandBase sendArmToStateAndEnd(ArmState armState) {
        return sendArmToStateAndEnd(() -> armState);
    }

    public void addToOrchestra(Orchestra m_orchestra) {
        m_orchestra.addInstrument(m_elbow.getMotor());
        m_orchestra.addInstrument(m_shoulder.getMotor());
        m_orchestra.addInstrument(m_wrist.getMotor());
    }

    @Override
    public void periodic() {
        m_allJoints.forEach(ArmJoint::zeroRelative);
        
        if (coastModeEnabled()) {
            m_allJoints.forEach(ArmJoint::coast);
        }
        else {
            m_allJoints.forEach(ArmJoint::brake);
        }

        m_allJoints.forEach((ArmJoint j) -> SmartDashboard.putNumber(j.getName() + " Angle", j.getAngle()));
        m_allJoints.forEach((ArmJoint j) -> SmartDashboard.putNumber(j.getName() + " Absolute Angle", j.getAbsoluteAngle()));
        m_allJoints.forEach((ArmJoint j) -> SmartDashboard.putBoolean(j.getName() + " Is Zeroed", j.isZeroed()));
    }
}