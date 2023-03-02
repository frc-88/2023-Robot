// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class AutoBalanceSimple extends CommandBase {
  /** Creates a new AutoBalanceSimple. */
  private SwerveDrive m_drive;
  private double m_lastAngle;
  private final SwerveModuleState [] lockStates = { new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(90))
};

  public AutoBalanceSimple(SwerveDrive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lastAngle = getTrueAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double true_angle = getTrueAngle();
    
    SmartDashboard.putNumber("Auto/ChargeAngle", true_angle);

    if (Math.abs(true_angle) < 1.0 || Math.abs(true_angle - m_lastAngle) > 0.1) {
      m_drive.setModuleStates(lockStates);
    } else {
      if (true_angle > 0) {
        m_drive.drive(-0.1, 0, 0);
      } else if (true_angle < 0){
        m_drive.drive(0.1, 0, 0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getTrueAngle(){
    double yaw = m_drive.getGyroscopeRotation().getDegrees();
    double pitch = m_drive.getNavX().getPitch();
    double roll = m_drive.getNavX().getRoll();
    return (pitch*-Math.cos(Math.toRadians(yaw)))+(roll*Math.sin(Math.toRadians(yaw)));
  }
}
