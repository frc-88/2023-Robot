package frc.robot.commands;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

public class PlaySong extends CommandBase {
    String filename;
    Orchestra m_orchestra = new Orchestra();
    Intake m_intake;
    SwerveDrive m_swerveDrive;
    Arm m_arm;

    public PlaySong(String filename, Intake intake, SwerveDrive swerveDrive, Arm arm) {
        this.filename = filename;
        addRequirements(intake, swerveDrive, arm);
        m_intake = intake;
        m_swerveDrive = swerveDrive;
        m_arm = arm;
    }

    public void initialize() {
        m_intake.addToOrchestra(m_orchestra);
        m_swerveDrive.addToOrchestra(m_orchestra);
        m_arm.addToOrchestra(m_orchestra);
        m_orchestra.loadMusic(filename);
        m_orchestra.play();
    }

    public void end() {
        m_orchestra.stop();
        m_orchestra.clearInstruments();
    }

    public boolean isFinished() {
        return !m_orchestra.isPlaying();
    }
}
