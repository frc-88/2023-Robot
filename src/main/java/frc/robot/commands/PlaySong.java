package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class PlaySong extends CommandBase {
    String filename;
    Orchestra m_orchestra = new Orchestra();   

    public PlaySong(String filename) {
        this.filename = filename;
    }

    public void initialize() {
        m_orchestra.addInstrument(new WPI_TalonFX(Constants.INTAKE_INNER_ROLLER_ID, Constants.INTAKE_CANBUS));
        m_orchestra.addInstrument(new WPI_TalonFX(Constants.INTAKE_OUTER_ROLLER_ID, Constants.INTAKE_CANBUS));
        m_orchestra.addInstrument(new WPI_TalonFX(Constants.INTAKE_ARM_ID, Constants.INTAKE_CANBUS));
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
