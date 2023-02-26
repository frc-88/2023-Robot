package frc.robot.util;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.coprocessor.networktables.ScorpionTable;

public class Aiming {

    public Arm m_arm;
    public Grabber m_grabber;
    public ScorpionTable m_ros;
    
    public Aiming(Arm arm, Grabber grabber, ScorpionTable ros) {
        m_arm = arm;
        m_grabber = grabber;
        m_ros = ros;
    }
}
