package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

    private final DoublePreferenceConstant p_aimHeight
        = new DoublePreferenceConstant("Aiming/Height", 8.);
    private final DoublePreferenceConstant p_aimAdjustY
        = new DoublePreferenceConstant("Aiming/AdjustY", 0);
    private final DoublePreferenceConstant p_aimAdjustYLimelight
        = new DoublePreferenceConstant("Aiming/AdjustYLimelight", 4);
    
    public Aiming(SwerveDrive drive, Arm arm, Grabber grabber, ScorpionTable ros, Limelight limelight, GameObjectManager objectManager, BooleanSupplier coneMode, BooleanSupplier enabled) {
        m_drive = drive;
        m_arm = arm;
        m_grabber = grabber;
        m_limelight = limelight;
        m_ros = ros;
        m_objects = objectManager;
        m_coneMode = coneMode;
        m_enabled = enabled;
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
        if (!m_enabled.getAsBoolean() && DriverStation.isTeleop()) {
            noAim();
            return;
        }

        double aimAngle;
        double armAdjust;
        if (m_coneMode.getAsBoolean()) {
            if (mid) {
                m_limelight.setRetroMidPipeline();
            } else {
                m_limelight.setRetroHighPipeline();
            }
            aimAngle = Math.toDegrees(Math.atan((getTargetOffset(mid) / p_aimHeight.getValue())));
            armAdjust = getTargetDistance(mid) - getCalibratedDistance(mid);
        } else {
            m_limelight.setAprilTagPipeline();
            Pose2d botPose = m_ros.getBotPoseInches().plus(new Transform2d(new Translation2d(outreach, p_aimAdjustY.getValue()), new Rotation2d(0)));
            aimAngle = Math.toDegrees(Math.atan((getNearestScorePoint(botPose).getY()-botPose.getY()) / p_aimHeight.getValue()));
            armAdjust = 0;
        }
        m_grabber.aim(aimAngle); 
    }

    private double getCalibratedDistance(boolean mid) {
        return mid ? 36 : 62;
    }

    public CommandBase aimFactory(double outreach, boolean mid) {
        return new RunCommand(() -> giveAim(outreach, mid));
    }

    public void noAim() {
        m_limelight.setAprilTagPipeline();
        double aimAngle = 0.;
        m_grabber.aim(aimAngle);
    }

    public CommandBase noAimFactory() {
        return new RunCommand(this::noAim);
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
        return m_arm.isAtTarget(10) 
            && m_grabber.isAtTarget()
            && m_enabled.getAsBoolean()
            && ((m_coneMode.getAsBoolean() && Math.abs(getTargetDistance(mid) - getCalibratedDistance(mid)) < 2 && getTargetOffset(mid) < 5)
                || !m_coneMode.getAsBoolean());
    }

    @Override
    public void periodic() {
        if (m_limelight.isRetroMidPipelineActive()) {
            SmartDashboard.putNumber("Aiming: Target Distance", getTargetDistance(true));
            SmartDashboard.putNumber("Aiming: Target Offset", getTargetOffset(true));
        } else if (m_limelight.isRetroHighPipelineActive()) {
            SmartDashboard.putNumber("Aiming: Target Distance", getTargetDistance(false));
            SmartDashboard.putNumber("Aiming: Target Offset", getTargetOffset(false));
        }
    }
}
