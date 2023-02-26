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
  private boolean onChargeStation;
  private static double m_pitch;
  private static double m_roll;
  private static double true_angle;
  private static Rotation2d m_heading;
  private static double m_degrees;
  private static double m_position_x;
  private static double m_position_y;
  private static double maxDistance = .5;
  private static int robotOrientation = 1;

  private static int m_state;
  /** Creates a new AutoBalance. */
  public AutoBalance(SwerveDrive drive, Boolean startChargeStation) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    onChargeStation = startChargeStation;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_heading = m_drive.getGyroscopeRotation();
    m_degrees = m_heading.getDegrees();
    m_drive.resetOdometry(new Pose2d(0,0,m_heading), m_heading);
    m_pitch = m_drive.getNavX().getPitch();
    m_roll = m_drive.getNavX().getRoll();
    if ((m_degrees > 90) || (m_degrees < -90)) {
      robotOrientation = -1;
    }
    if (onChargeStation) {
      m_state = 2;
    } else {
      m_state = 0;
    }
    
  }

  /* Tomorrow when the farm boys find this freak of nature,
   * they will wrap his body in newspaper and carry him to the museum.
   * But tonight he is alive and in the north field with his mother.
   * It is a perfect summer evening:
   * the moon rising over the orchard, the wind in the grass.
   * And he stares into the sky,
   * there are twice as many stars as usual.
   * - Two-Headed Calf, by Laura Gilpin
   */

  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_state) {
      case 0:
        m_drive.drive(Constants.MAX_TRAJ_VELOCITY, 0, 0);
          if (Math.abs(m_drive.getNavX().getPitch()) > (Math.abs(m_pitch)+.01)) {
            m_state = m_state + 1;
          }
          
      case 1:
        m_drive.drive(Constants.MAX_TRAJ_VELOCITY/2, 0, 0);
          if ((Math.abs(m_drive.getNavX().getPitch()) + Math.abs(m_drive.getNavX().getRoll())) < ((Math.abs(m_pitch) + Math.abs(m_roll))+.005)) {
            m_state = m_state + 1;
          }
        m_pitch = m_drive.getNavX().getPitch();
        m_roll = m_drive.getNavX().getRoll();
    
      //robot begins to automatically adjust its angle to 
      case 2:
        m_pitch = m_drive.getNavX().getPitch();
        m_roll = m_drive.getNavX().getRoll();
        true_angle = (Math.abs(m_pitch*(Math.pow(Math.sin(Math.toRadians(m_degrees)), 2)))) + (Math.abs(m_roll* Math.pow(Math.cos(Math.toRadians(m_degrees)), 2)));
        m_position_y = m_drive.getOdometryPose().getY();
        m_position_x = m_drive.getOdometryPose().getX();
    
        //if the robot hasn't moved more than a maxmimum allotted distance, 
        //the robot can move only on y-axis
        if ((m_position_y < maxDistance) || (m_position_x < maxDistance)) {
          System.out.println("the robot is under its max distance");
          System.out.println(true_angle);
          System.out.println(m_degrees);
          if (true_angle > Constants.CHARGE_STATION_LEVEL) {
            System.out.println("true angle is greater than level");
            if (m_pitch > Constants.CHARGE_STATION_LEVEL+.2) {
              System.out.println("robot has its pitch larger than the charge station level, it should drive");
              m_drive.drive((robotOrientation * Constants.MAX_TRAJ_VELOCITY/4), 0, 0);
            } 
            else if (m_pitch < -Constants.CHARGE_STATION_LEVEL-.2) {
              System.out.println("System has a pitch greater than negative charge station");
              m_drive.drive((-robotOrientation * Constants.MAX_TRAJ_VELOCITY/4), 0, 0);
            } else if ((m_pitch > -Constants.CHARGE_STATION_LEVEL-.2) && (m_pitch < Constants.CHARGE_STATION_LEVEL)) {
              System.out.println("high roll");
              m_drive.drive((m_degrees/Math.abs(m_degrees)) * (m_roll/Math.abs(m_roll)) * Constants.MAX_TRAJ_VELOCITY, 0, 0);
            }
          } 
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
    return false;
  }
}