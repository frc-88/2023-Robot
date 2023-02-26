package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GameObjectManager;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.coprocessor.GridZone;
import frc.robot.util.coprocessor.networktables.ScorpionTable;

public class Aiming {

    public Arm m_arm;
    public Grabber m_grabber;
    public ScorpionTable m_ros;
    public GameObjectManager m_objects;
    
    public Aiming(Arm arm, Grabber grabber, ScorpionTable ros, GameObjectManager objectManager) {
        m_arm = arm;
        m_grabber = grabber;
        m_ros = ros;
        m_objects = objectManager;
    }

    public Pose2d getNearestScorePoint() {
        Pose2d closestZone = new Pose2d();
        double closestZoneDistance = 1000; //unreasonably high number to have something to compare to
        Pose2d botPose = m_ros.getBotPoseInches();
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
    public void giveWristAim() {
        double aimAngle = Math.atan2(1., getNearestScorePoint().getY()-m_ros.getBotPoseInches().getY());
        m_grabber.aim(aimAngle); 
    }
    public CommandBase aimGrabberFactory() {
        return new RunCommand(this::giveWristAim);
    }
    public void noWristAim() {
        double aimAngle = 0.;
        m_grabber.aim(aimAngle);
    }
    public CommandBase noGrabberAimFactory() {
        return new RunCommand(this::noWristAim);
    }
}
