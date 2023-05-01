package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Aiming extends SubsystemBase {

    private final SwerveDrive m_drive;
    private final Arm m_arm;
    private final Grabber m_grabber;
    private final ScorpionCoprocessorBridge m_ros;
    private final Limelight m_limelight;
    private final BooleanSupplier m_coneMode;
    private final BooleanSupplier m_enabled;

    private final DoublePreferenceConstant p_aimHeightMid
        = new DoublePreferenceConstant("Aiming/HeightMid", 7.);
    private final DoublePreferenceConstant p_aimHeightHigh
        = new DoublePreferenceConstant("Aiming/HeightHigh", 4.);
    private final DoublePreferenceConstant p_aimAdjustY
        = new DoublePreferenceConstant("Aiming/AdjustY", 0);
    private final DoublePreferenceConstant p_aimAdjustYLimelight
        = new DoublePreferenceConstant("Aiming/AdjustYLimelight", 4);

    private final Timer m_pipelineTimer = new Timer();
    private final Debouncer m_readyDebounce = new Debouncer(0.4, DebounceType.kRising);

    private static final double MAX_DISTANCE_ERROR_MID = 7;
    private static final double MAX_DISTANCE_ERROR_HIGH = 7;
    private static final double READY_TO_AIM_SEC = 1.5;
    private static final double READY_TO_SCORE_SEC = 2;
    
    public Aiming(SwerveDrive drive, Arm arm, Grabber grabber, ScorpionCoprocessorBridge ros, Limelight limelight, BooleanSupplier coneMode, BooleanSupplier enabled) {
        m_drive = drive;
        m_arm = arm;
        m_grabber = grabber;
        m_limelight = limelight;
        m_ros = ros;
        m_coneMode = coneMode;
        m_enabled = enabled;

        m_pipelineTimer.stop();
        m_pipelineTimer.reset();
    }

    public void giveAim(double outreach, boolean mid) {
        double aimAngle;
        double armAdjust;
        if (m_coneMode.getAsBoolean()) {
            setRetroPipeline(mid);
            aimAngle = Math.toDegrees(Math.atan((getTargetOffset(mid) / (mid ? p_aimHeightMid : p_aimHeightHigh).getValue())));
            armAdjust = Math.min(Math.max(getTargetDistance(mid) - getCalibratedDistance(mid), -getMaxDistanceError(mid)), getMaxDistanceError(mid));
        } else {
            m_limelight.setAprilTagPipeline();
            aimAngle = 0;
            armAdjust = 0;
        }

        if ((!m_enabled.getAsBoolean() && DriverStation.isTeleop()) || !m_pipelineTimer.hasElapsed(READY_TO_AIM_SEC)) {
            noAim();
            return;
        }

        m_grabber.aim(aimAngle);
        m_arm.setAim(armAdjust);
    }

    private double getCalibratedDistance(boolean mid) {
        return mid ? 39.4 : 53;
    }

    private double getMaxDistanceError(boolean mid) {
        return mid ? MAX_DISTANCE_ERROR_MID : MAX_DISTANCE_ERROR_HIGH;
    }

    private double getMaxOffsetError(boolean mid) {
        return mid ? 8 : 6;
    }

    public CommandBase aimFactory(double outreach, boolean mid) {
        return new RunCommand(() -> giveAim(outreach, mid), this);
    }

    public void noAim() {
        m_grabber.aim(0);
        m_arm.resetAim();
    }

    public CommandBase noAimFactory() {
        return new RunCommand(this::noAim, this);
    }

    private double getTargetDistance(boolean mid) {
        double tapeHeight;
        if (mid) {
            tapeHeight = 10.65;
        } else {
            tapeHeight = 29.65;
        }
        double ret = tapeHeight / (Math.tan(m_limelight.getTY()));
        SmartDashboard.putNumber("Aiming: Target Distance", ret);
        return ret;
    }

    private double getTargetOffset(boolean mid) {
        double ret = getTargetDistance(mid)*Math.tan(m_limelight.getTX()) + p_aimAdjustYLimelight.getValue();
        SmartDashboard.putNumber("Aiming: Target Offset", ret);
        return ret;
    }

    public boolean readyToScore(boolean mid) {
        boolean ret =  m_arm.isAtTarget(8) 
            && m_grabber.isAtTarget()
            && m_drive.notMoving()
            && m_enabled.getAsBoolean()
            && m_pipelineTimer.hasElapsed(READY_TO_SCORE_SEC)
            && (m_coneMode.getAsBoolean() && Math.abs(getTargetDistance(mid) - getCalibratedDistance(mid)) < getMaxDistanceError(mid) && Math.abs(getTargetOffset(mid)) < getMaxOffsetError(mid));
        return m_readyDebounce.calculate(ret);
    }

    public void setRetroPipeline(boolean mid) {
        if (mid) {
            if (!m_limelight.isRetroMidPipelineActive()) {
                m_pipelineTimer.reset();
                m_pipelineTimer.start();
            }
            m_limelight.setRetroMidPipeline();
        } else {
            if (!m_limelight.isRetroHighPipelineActive()) {
                m_pipelineTimer.reset();
                m_pipelineTimer.start();
            }
            m_limelight.setRetroHighPipeline();
        }
    }

    public void setAprilTagPipeline() {
        if (!m_limelight.isAprilTagPipelineActive()) {
            m_pipelineTimer.restart();
        }
        m_limelight.setAprilTagPipeline();
    }

    public CommandBase setRetroPipelineFactory(boolean mid) {
        return new InstantCommand(() -> setRetroPipeline(mid));
    }

    public CommandBase setAprilTagPipelineFactory() {
        return new InstantCommand(this::setAprilTagPipeline);
    }

    @Override
    public void periodic() {
    }
}
