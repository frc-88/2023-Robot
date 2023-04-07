// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class FollowHolonomicTrajectory extends CommandBase {
  private final SwerveDrive m_drive;
  private final Trajectory m_trajectory;
  private final Rotation2d m_rotation;
  private final boolean m_resetOdometry;
  private final boolean m_stopOnTip;
  private final Timer m_timer = new Timer();

  private final static PIDPreferenceConstants p_vxPID = new PIDPreferenceConstants("Auto/vxPID/");
  private final static PIDPreferenceConstants p_vyPID = new PIDPreferenceConstants("Auto/vyPID/");
  private final static PIDPreferenceConstants p_thetaPID = new PIDPreferenceConstants("Auto/theta/PID/");
  private final static DoublePreferenceConstant p_thetaMaxVelocity = new DoublePreferenceConstant("Auto/theta/Max Velocity", Math.PI);;
  private final static DoublePreferenceConstant p_thetaMaxAcceleration = new DoublePreferenceConstant("Auto/theta/Max Acceleration", Math.PI);
  private final static DoublePreferenceConstant p_xTolerance = new DoublePreferenceConstant("Auto/Tolerance/x", 0.1);
  private final static DoublePreferenceConstant p_yTolerance = new DoublePreferenceConstant("Auto/Tolerance/y", 0.1);
  private final static DoublePreferenceConstant p_thetaTolerance = new DoublePreferenceConstant("Auto/Tolerance/theta", 0.05);
  private final static DoublePreferenceConstant p_rollRateTolerance = new DoublePreferenceConstant("Auto/Tolerance/roll rate", 0.25);


  private HolonomicDriveController m_controller;
  private boolean m_cancel = false;
  private double m_lastRoll;

  /** Creates a new FollowHolonomicTrajectory. */
  public FollowHolonomicTrajectory(SwerveDrive drive, Trajectory trajectory, Rotation2d startRotation,  Rotation2d endRotation, boolean resetOdometry, boolean stopOnTip) {
    m_drive = drive;
    m_trajectory = trajectory;
    m_rotation = endRotation.minus(startRotation);
    m_resetOdometry = resetOdometry;
    m_stopOnTip = stopOnTip;

    addRequirements(m_drive);
  }

  public FollowHolonomicTrajectory(SwerveDrive drive, Trajectory trajectory, Rotation2d startRotation,  Rotation2d endRotation, boolean resetOdometry) {
    this(drive, trajectory, startRotation, endRotation, resetOdometry, false);
  }

  public FollowHolonomicTrajectory(SwerveDrive drive, Trajectory trajectory, boolean resetOdometry, boolean stopOnTip) {
    this(drive, trajectory, new Rotation2d(), new Rotation2d(), resetOdometry, stopOnTip);
  }

  public FollowHolonomicTrajectory(SwerveDrive drive, Trajectory trajectory, boolean resetOdometry) {
    this(drive, trajectory, new Rotation2d(), new Rotation2d(), resetOdometry, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_cancel = false;

    m_controller = new HolonomicDriveController(
      new PIDController(p_vxPID.getKP().getValue(), p_vxPID.getKI().getValue(), p_vxPID.getKD().getValue()),
      new PIDController(p_vyPID.getKP().getValue(), p_vyPID.getKI().getValue(), p_vyPID.getKD().getValue()),
      new ProfiledPIDController(p_thetaPID.getKP().getValue(), p_thetaPID.getKI().getValue(), p_thetaPID.getKD().getValue(), 
        new TrapezoidProfile.Constraints(p_thetaMaxVelocity.getValue(), p_thetaMaxAcceleration.getValue())));

    m_controller.setTolerance(new Pose2d(p_xTolerance.getValue(), p_yTolerance.getValue(), Rotation2d.fromDegrees(p_thetaTolerance.getValue())));

    if (m_resetOdometry) {
      m_drive.resetTrajectoryPose(m_trajectory.getInitialPose());
    } else {
      Transform2d offset = m_drive.getOdometryPose().minus(m_trajectory.getInitialPose());
      if (offset.getTranslation().getDistance(new Translation2d()) > 6.0) {
        System.out.println("!!!canceling holomic trajectory!!!");
        m_cancel = true;
      }
    }

    m_lastRoll = m_drive.getNavX().getRoll();

    m_controller.setEnabled(true);
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double roll = m_drive.getNavX().getRoll();
    double rollRate = roll - m_lastRoll;
    m_lastRoll = roll;

    if (m_stopOnTip && Math.abs(roll) < 10 && (Math.abs(rollRate) > Math.abs(p_rollRateTolerance.getValue())) &&
        (Math.signum(rollRate) != Math.signum(roll)) ) {
      m_cancel = true;
      System.out.println("!!Auto Follow Cancel: Tipping");
    }

    Pose2d currentPose = m_drive.getOdometryPose();
    Trajectory.State desiredState = m_trajectory.sample(m_timer.get());
    Rotation2d targetRotation = new Rotation2d(m_timer.get() * m_rotation.getRadians() / m_trajectory.getTotalTimeSeconds());

    ChassisSpeeds targetSpeeds = m_controller.calculate(currentPose, desiredState, targetRotation);

    SmartDashboard.putNumber("Auto:Measured vX", m_drive.getChassisSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("Auto:Measured vY", m_drive.getChassisSpeeds().vyMetersPerSecond);
    SmartDashboard.putNumber("Auto:Measured omega", m_drive.getChassisSpeeds().omegaRadiansPerSecond);

    SmartDashboard.putNumber("Auto:Desired vX", desiredState.velocityMetersPerSecond * Math.cos(desiredState.poseMeters.getRotation().getRadians()));
    SmartDashboard.putNumber("Auto:Desired vY", desiredState.velocityMetersPerSecond * Math.sin(desiredState.poseMeters.getRotation().getRadians()));
    SmartDashboard.putNumber("Auto:Desired omega", desiredState.curvatureRadPerMeter);

    SmartDashboard.putNumber("Auto:Commanded vX", targetSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Auto:Commanded vY", targetSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Auto:Commanded omega", targetSpeeds.omegaRadiansPerSecond);

    SmartDashboard.putNumber("Auto:Rollm rate", rollRate);

    if (!m_cancel) m_drive.drive(targetSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  m_timer.get() > m_trajectory.getTotalTimeSeconds() || m_cancel;
  }
}
