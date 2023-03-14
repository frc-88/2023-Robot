package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
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

/*
 * a triple jointed
 *               arm of amazing beuaty
 *                                   what else can it do?
 */

public class Arm extends SubsystemBase {
    
    private BooleanSupplier m_isIntaking = () -> false;
    private BooleanSupplier m_hasGamePiece = () -> false;
    private BooleanSupplier m_hpMode = () -> false;

    private final DigitalInput m_coastButton;

    public final ArmJoint m_shoulder;
    public final ArmJoint m_elbow;
    public final ArmJoint m_wrist;
    public final Translation2d m_shoulderPosition;

    private ArmState m_targetArmState;
    private List<ArmJoint> m_allJoints;
    private CommandBase m_stowCommand;

    private int loopCount = 0;

    private double m_aimX = 0;

    public Arm() {
        this.m_coastButton = new DigitalInput(0);
        SmartDashboard.putBoolean("Coast Arm", false);

        m_shoulder = new ArmJoint("Shoulder", Constants.SHOULDER_ID, Constants.SHOULDER_ENCODER_ID, true, true, (16./42.) * (1./49.), 19.5, 270);
        m_elbow = new ArmJoint("Elbow", Constants.ELBOW_ID, Constants.ELBOW_ENCODER_ID, true, true, (16./42.) * (1./49.), 15.5, 90);
        m_wrist = new ArmJoint("Wrist", Constants.WRIST_ID, Constants.WRIST_ENCODER_ID, true, true, (16./42.) * (1./49.), 25.5, 270);
        m_shoulderPosition = new Translation2d(3, 18.5);

        m_targetArmState = getCurrentStow();
        m_allJoints = Arrays.asList(new ArmJoint[]{m_shoulder, m_elbow, m_wrist});

        SmartDashboard.putBoolean("Coast Arm", false);

        resetStow();
    }

    public void setStowSuppliers(BooleanSupplier isIntaking, BooleanSupplier hasGamePiece, BooleanSupplier hpMode) {
        m_isIntaking = isIntaking;
        m_hasGamePiece = hasGamePiece;
        m_hpMode = hpMode;
    }

    public void resetStow() {
        m_stowCommand = stowSimple();
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

    public void goToArmState(ArmState armState, double acceleration) {
        if (!isValidState(armState)) return;
        if (!m_shoulder.isZeroed() || !m_elbow.isZeroed() || !m_wrist.isZeroed()) return;

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
        double shoulderFactor = Math.abs(m_shoulder.getAngle() - shoulderAngle) / greatestAngle;
        double elbowFactor = Math.abs(m_elbow.getAngle() - elbowAngle) / greatestAngle;
        double wristFactor = Math.abs(m_wrist.getAngle() - wristAngle) / greatestAngle;


        if (greatestAngle > 2) {
            // Keep max velocities the same or change this
            m_shoulder.setMotionMagic(shoulderAngle, m_shoulder.getMaxVelocity() * shoulderFactor, acceleration * shoulderFactor);
            m_elbow.setMotionMagic(elbowAngle, m_elbow.getMaxVelocity() * elbowFactor, acceleration * elbowFactor);
            m_wrist.setMotionMagic(wristAngle, m_wrist.getMaxVelocity() * wristFactor, acceleration * wristFactor);
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

    public boolean isAtTarget(ArmState state, DoubleSupplier tolerance) {
        if (!(m_shoulder.isOnTarget(state.getShoulderAngle(), tolerance.getAsDouble()))) return false;
        if (!(m_elbow.isOnTarget(state.getElbowAngle(), tolerance.getAsDouble()))) return false;
        if (!(m_wrist.isOnTarget(state.getWristAngle(), tolerance.getAsDouble()))) return false;
        return true;
    }

    public boolean isAtTarget(ArmState state, double tolerance) {
        return isAtTarget(state, () -> tolerance);
    }

    public boolean isAtTarget(double tolerance) {
        return isAtTarget(m_targetArmState, tolerance);
    }

    public boolean isStowed() {
        return m_targetArmState.isStow() && isAtTarget(m_targetArmState, 30);
    }

    public boolean coastModeEnabled() {
        return DriverStation.isDisabled() && (!m_coastButton.get() || SmartDashboard.getBoolean("Coast Arm", false));
    }

    public void resetMotionMagic() {
        m_allJoints.forEach((ArmJoint::stop));
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

    public ArmState getCurrentStow() {
        if (m_hasGamePiece.getAsBoolean()) {
            return ArmStates.stowWithPiece;
        } else if (m_isIntaking.getAsBoolean()) {
            return ArmStates.stowForHandoff;
        } else if (m_hpMode.getAsBoolean()) {
            return ArmStates.stowForHP;
        } else {
            return ArmStates.stowFlat;
        }
    }

    public CommandBase stowSimple() {
        return new CommandBase() {
            double m_acceleration;
            @Override
            public void initialize() {
                m_acceleration = m_targetArmState.getAcceleration();
                m_targetArmState = getCurrentStow();
            }

            public void execute() {
                goToArmState(getCurrentStow(), m_acceleration);
            }
        };
    }

    private CommandBase chainIntermediaries(CommandBase initialCommand, List<ArmState> intermediaries, DoubleSupplier tolerance, double acceleration, BooleanSupplier unless) {
        CommandBase command = initialCommand;
        for (ArmState intermediary : intermediaries) {
            command = new RunCommand(() -> goToArmState(intermediary, acceleration), this)
                .until(() -> isAtTarget(intermediary, tolerance))
                .unless(unless)
                .andThen(command);
        }
        return command;
    }

    public CommandBase stowFrom(ArmState from) {
        return chainIntermediaries(
            stowSimple(),
            from.getRetractIntermediaries(),
            from.getRetractIntermediaryTolerance(),
            from.getAcceleration(),
            () -> false
        ).andThen(new InstantCommand(this::resetStow));
    }

    private CommandBase sendArmToState(ArmState armState, BooleanSupplier until) {
        if (armState.isStow()) {
            CommandBase command = new ProxyCommand(() -> isAtTarget(m_targetArmState, 15) ? m_stowCommand : stowSimple()).until(until);
            command.addRequirements(this);
            return command;
        } else {
            CommandBase command = chainIntermediaries(
                new InstantCommand(() -> m_stowCommand = stowFrom(armState)).alongWith(new RunCommand(() -> goToArmState(armState, armState.getAcceleration()), this).until(until)),
                armState.getDeployIntermediaries(),
                armState.getDeployIntermediaryTolerance(),
                armState.getAcceleration(),
                () -> !isAtTarget(ArmStates.stowFlat, 15) && !isAtTarget(ArmStates.stowForHP, 15) && !isAtTarget(ArmStates.stowWithPiece, 15) && !isAtTarget(ArmStates.stowForHandoff, 15)
            );
            command = new InstantCommand(() -> {m_targetArmState = armState;}).alongWith(command);
            return command;
        }
    }

    public CommandBase holdTargetState() {
        return new RunCommand(() -> goToArmState(m_targetArmState, m_targetArmState.getAcceleration()), this);
    }

    public CommandBase sendArmToState(ArmState armState) {
        return sendArmToState(armState, () -> false);
    }

    public CommandBase sendArmToStateAndEnd(ArmState armState) {
        return sendArmToState(armState, () -> isAtTarget(armState));
    }

    public void addToOrchestra(Orchestra m_orchestra) {
        m_orchestra.addInstrument(m_elbow.getMotor());
        m_orchestra.addInstrument(m_shoulder.getMotor());
        m_orchestra.addInstrument(m_wrist.getMotor());
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            m_allJoints.forEach(ArmJoint::checkZero);

            if (!coastModeEnabled() && loopCount < 2500) {
                switch (loopCount % 60) {
                    case 0:
                        m_shoulder.configureMotor();
                        break;
                    case 20:
                        m_elbow.configureMotor();
                        break;
                    case 40:
                        m_wrist.configureMotor();
                        break;
                }
                loopCount++;
            }

        }
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
        SmartDashboard.putBoolean("Coast Mode Enabled", coastModeEnabled());
        SmartDashboard.putString("Arm Target State", m_targetArmState.toString());
        SmartDashboard.putString("Arm Stow", getCurrentStow().toString());
    }
}