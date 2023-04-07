package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.coprocessor.GridZone;
import frc.robot.util.coprocessor.networktables.ScorpionTable;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Aiming extends SubsystemBase {

    private final SwerveDrive m_drive;
    private final Arm m_arm;
    private final Grabber m_grabber;
    private final ScorpionTable m_ros;
    private final Limelight m_limelight;
    private final GameObjectManager m_objects;
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
    
    public Aiming(SwerveDrive drive, Arm arm, Grabber grabber, ScorpionTable ros, Limelight limelight, GameObjectManager objectManager, BooleanSupplier coneMode, BooleanSupplier enabled) {
        m_drive = drive;
        m_arm = arm;
        m_grabber = grabber;
        m_limelight = limelight;
        m_ros = ros;
        m_objects = objectManager;
        m_coneMode = coneMode;
        m_enabled = enabled;

        m_pipelineTimer.stop();
        m_pipelineTimer.reset();
    }

    public Pose2d getNearestScorePoint(Pose2d botPose) {
        Pose2d closestZone = new Pose2d();
        double closestZoneDistance = 1000; //unreasonably high number to have something to compare to
        for(GridZone gridZone : m_objects.gridZones) {
            gridZone.getY();
            double distance = Math.sqrt(Math.pow(botPose.getX() - gridZone.getX(), 2) + Math.pow(botPose.getY() - gridZone.getY(), 2));
            if(distance < closestZoneDistance) {
                closestZone = new Pose2d(new Translation2d(gridZone.getX(), gridZone.getY()), new Rotation2d(0.));
                closestZoneDistance = distance;
            }
        }
        return closestZone;
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
            // Pose2d botPose = m_ros.getBotPoseInches().plus(new Transform2d(new Translation2d(outreach, p_aimAdjustY.getValue()), new Rotation2d(0)));
            // aimAngle = Math.toDegrees(Math.atan((getNearestScorePoint(botPose).getY()-botPose.getY()) / p_aimHeight.getValue()));
            aimAngle = 0;
            armAdjust = 0;
            System.out.println("Cube mode");
        }

        if ((!m_enabled.getAsBoolean() && DriverStation.isTeleop()) || !m_pipelineTimer.hasElapsed(READY_TO_AIM_SEC)) {
            noAim();
            return;
        }

        m_grabber.aim(aimAngle);
        m_arm.setAim(armAdjust);

        System.out.println("Aiming: " + aimAngle);
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
        System.out.println("Offset: " + ret);
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
