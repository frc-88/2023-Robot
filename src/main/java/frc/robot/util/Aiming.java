package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GameObjectManager;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.coprocessor.GridZone;
import frc.robot.util.coprocessor.networktables.ScorpionTable;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Aiming {

    private final SwerveDrive m_drive;
    private final Arm m_arm;
    private final Grabber m_grabber;
    private final ScorpionTable m_ros;
    private final GameObjectManager m_objects;
    private final BooleanSupplier m_enabled;

    private final DoublePreferenceConstant p_aimHeight
        = new DoublePreferenceConstant("Aiming/Height", 8.);
    private final DoublePreferenceConstant p_aimAdjustY
        = new DoublePreferenceConstant("Aiming/AdjustY", 0);
    
    public Aiming(SwerveDrive drive, Arm arm, Grabber grabber, ScorpionTable ros, GameObjectManager objectManager, BooleanSupplier enabled) {
        m_drive = drive;
        m_arm = arm;
        m_grabber = grabber;
        m_ros = ros;
        m_objects = objectManager;
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
    public void giveWristAim(double outreach) {
        if (!m_enabled.getAsBoolean()) {
            noWristAim();
            return;
        }
        Pose2d botPose = m_drive.getPoseEstimate().times(39.3701).plus(new Transform2d(new Translation2d(outreach, p_aimAdjustY.getValue()), new Rotation2d(0)));
        double aimAngle = Math.toDegrees(Math.atan((getNearestScorePoint(botPose).getY()-botPose.getY()) / p_aimHeight.getValue()));
        m_grabber.aim(aimAngle); 
    }
    public CommandBase aimGrabberFactory(double outreach) {
        return new RunCommand(() -> giveWristAim(outreach));
    }
    public void noWristAim() {
        double aimAngle = 0.;
        m_grabber.aim(aimAngle);
    }
    public CommandBase noGrabberAimFactory() {
        return new RunCommand(this::noWristAim);
    }
}
