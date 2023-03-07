// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class FollowHolonomicTrajectory extends CommandBase {
  private final SwerveDrive m_drive;
  private final HolonomicDriveController m_controller;
  private final Trajectory m_trajectory;
  private final Timer m_timer = new Timer();

  private final PIDPreferenceConstants p_vxPID;
  private final PIDPreferenceConstants p_vyPID;
  private final PIDPreferenceConstants p_thetaPID;
  private final DoublePreferenceConstant p_thetaMaxVelocity;
  private final DoublePreferenceConstant p_thetaMaxAcceleration;
  private final DoublePreferenceConstant p_xTolerance;
  private final DoublePreferenceConstant p_yTolerance;
  private final DoublePreferenceConstant p_thetaTolerance;

  /** Creates a new FollowHolonomicTrajectory. */
  public FollowHolonomicTrajectory(SwerveDrive drive, Trajectory trajectory) {
    m_drive = drive;
    m_trajectory = trajectory;

    p_vxPID = new PIDPreferenceConstants("Auto/vxPID/");
    p_vyPID = new PIDPreferenceConstants("Auto/vyPID/");
    p_thetaPID = new PIDPreferenceConstants("Auto/theta/PID/");
    p_thetaMaxVelocity = new DoublePreferenceConstant("Auto/theta/Max Velocity", 0);
    p_thetaMaxAcceleration = new DoublePreferenceConstant("Auto/theta/Max Acceleration", 0);
    p_xTolerance = new DoublePreferenceConstant("Auto/Tolerance/x", 0);
    p_yTolerance = new DoublePreferenceConstant("Auto/Tolerance/y", 0);
    p_thetaTolerance = new DoublePreferenceConstant("Auto/Tolerance/theta", 0);

    m_controller = new HolonomicDriveController(new PIDController(p_vxPID.getKP().getValue(), p_vxPID.getKI().getValue(), p_vxPID.getKD().getValue()),
      new PIDController(p_vyPID.getKP().getValue(), p_vyPID.getKI().getValue(), p_vyPID.getKD().getValue()),
      new ProfiledPIDController(p_thetaPID.getKP().getValue(), p_thetaPID.getKI().getValue(), p_thetaPID.getKD().getValue(), 
      new TrapezoidProfile.Constraints(p_thetaMaxVelocity.getValue(), p_thetaMaxAcceleration.getValue())));

    m_controller.setTolerance(new Pose2d(p_xTolerance.getValue(), p_yTolerance.getValue(), Rotation2d.fromDegrees(p_thetaTolerance.getValue())));

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.setEnabled(true);
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(m_controller.calculate(m_drive.getOdometryPose(), m_trajectory.sample(m_timer.get()), new Rotation2d()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controller.atReference();
  }
}
