// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class AutoBalanceSimple extends CommandBase {
  /** Creates a new AutoBalanceSimple. */
  private SwerveDrive m_drive;
  private Pose2d m_startPose;
  private double m_lastAngle;
  private final SwerveModuleState [] LOCK_STATES = { 
    new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(90))
  };

  private final DoublePreferenceConstant m_levelThreshold = new DoublePreferenceConstant("Auto/Balance/Level Theshold", 2.0);
  private final DoublePreferenceConstant m_movingThreshold = new DoublePreferenceConstant("Auto/Balance/Moving Theshold", 0.5);
  private final DoublePreferenceConstant m_climbSpeed = new DoublePreferenceConstant("Auto/Balance/Climb Speed", 0.1);
  private final DoublePreferenceConstant m_climbMaxDistance = new DoublePreferenceConstant("Auto/Balance/Climb Max", 1.0);

  public AutoBalanceSimple(SwerveDrive drive) {
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startPose = m_drive.getOdometryPose();
    m_lastAngle = getChargeStationAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = getChargeStationAngle();
    double deltaAngle = currentAngle - m_lastAngle;
    m_lastAngle = currentAngle;
    
    SmartDashboard.putNumber("Auto/currentAngle", currentAngle);
    SmartDashboard.putNumber("Auto/deltaAngle", deltaAngle);

    if (Math.abs(currentAngle) < m_levelThreshold.getValue()) {
      // if the angle is level, lock in place
      m_drive.setModuleStates(LOCK_STATES);
    } else if (Math.abs(deltaAngle) > m_movingThreshold.getValue()) {
      // if the angle is moving, lock in place
      m_drive.setModuleStates(LOCK_STATES);
    } else if (m_drive.getOdometryPose().getTranslation().getDistance(m_startPose.getTranslation()) > m_climbMaxDistance.getValue() ) {
      // if we have driven too far from where we began, lock in place
      m_drive.setModuleStates(LOCK_STATES);
    } else {
      // drive up
      m_drive.drive(m_climbSpeed.getValue() * Math.signum(-currentAngle), 0, 0);
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

  private double getChargeStationAngle(){
    // double yaw = m_drive.getGyroscopeRotation().getDegrees();
    // double pitch = m_drive.getNavX().getPitch();
    // double roll = m_drive.getNavX().getRoll();
    // return (pitch*-Math.cos(Math.toRadians(yaw)))+(roll*Math.sin(Math.toRadians(yaw)));
    // trying something simple first:
    return m_drive.getNavX().getRoll();
  }
}
