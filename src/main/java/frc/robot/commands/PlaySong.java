package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

public class PlaySong extends CommandBase {
    String filename;
    Orchestra m_orchestra = new Orchestra();
    Intake m_intake;
    SwerveDrive m_swerveDrive;

    public PlaySong(String filename, Intake intake, SwerveDrive swerveDrive) {
        this.filename = filename;
        addRequirements(intake);
        m_intake = intake;
        m_swerveDrive = swerveDrive;
    }

    public void initialize() {
        m_orchestra.addInstrument(m_intake.getMotors()[0]);
        m_orchestra.addInstrument(m_intake.getMotors()[1]);
        m_orchestra.addInstrument(m_intake.getMotors()[2]);
        m_orchestra.addInstrument(new WPI_TalonFX(6, Constants.INTAKE_CANBUS));
        m_orchestra.addInstrument(new WPI_TalonFX(12, Constants.INTAKE_CANBUS));
        m_orchestra.addInstrument(new WPI_TalonFX(2, Constants.INTAKE_CANBUS));
        m_orchestra.addInstrument(m_swerveDrive.getMotors()[0]);
        m_orchestra.addInstrument(m_swerveDrive.getMotors()[1]);
        m_orchestra.addInstrument(m_swerveDrive.getMotors()[2]);
        m_orchestra.addInstrument(m_swerveDrive.getMotors()[3]);
        m_orchestra.addInstrument(m_swerveDrive.getMotors()[4]);
        m_orchestra.addInstrument(m_swerveDrive.getMotors()[5]);
        m_orchestra.addInstrument(m_swerveDrive.getMotors()[6]);
        m_orchestra.addInstrument(m_swerveDrive.getMotors()[7]);
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
