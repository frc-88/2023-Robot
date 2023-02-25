// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class AutoBalance extends CommandBase {
  private SwerveDrive m_drive;
  private static double m_pitch;
  private static Rotation2d m_heading;
  private static double m_degrees;
  private static double m_position_x;
  private static double m_position_y;
  private static double maxDistance = .5;

  private static int m_state;
  private static boolean m_reverse = false;
  /** Creates a new AutoBalance. */
  public AutoBalance(SwerveDrive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_heading = m_drive.getGyroscopeRotation();
    m_degrees = m_heading.getDegrees();
    m_drive.resetOdometry(new Pose2d(0,0,m_heading), m_heading);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_degrees = m_drive.getGyroscopeRotation().getDegrees();
    m_pitch = m_drive.getNavX().getPitch();

    m_position_y = m_drive.getOdometryPose().getY();
    m_position_x = m_drive.getOdometryPose().getX();

    //if the robot hasn't moved more than a maxmimum allotted distance, 
    //the robot can move only on y-axis
    if ((m_position_y < maxDistance) || (m_position_x < maxDistance)) {
      if (m_pitch > Constants.CHARGE_STATION_LEVEL) {
        m_drive.drive(-Math.cos(Math.toRadians(m_degrees))*Constants.MAX_TRAJ_VELOCITY, Math.sin(Math.toRadians(m_degrees))*Constants.MAX_TRAJ_VELOCITY, 0);
      } else {
        m_state = 1;
      }
    }
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (m_state) {
      case 1:
        return true;
      case 2:
        return true;
      default:
        return false;
    }
    
  }
}