// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class AutoBalancePID extends CommandBase {
  /** Creates a new AutoBalancePID. */
  private SwerveDrive m_drive;
  private PIDController m_pid;

  private final PIDPreferenceConstants p_pid = new PIDPreferenceConstants("Auto/Balance/PID/");

  public AutoBalancePID(SwerveDrive drive) {
    m_drive = drive;
    m_pid = new PIDController(p_pid.getKP().getValue(), p_pid.getKI().getValue(), p_pid.getKD().getValue());
    p_pid.addChangeHandler((Double unused) -> m_pid.setPID(p_pid.getKP().getValue(), p_pid.getKI().getValue(), p_pid.getKD().getValue()));
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(-m_pid.calculate(m_drive.getNavX().getRoll(), 0), 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
