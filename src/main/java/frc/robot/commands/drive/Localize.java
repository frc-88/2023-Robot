// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.BotPoseProvider;

public class Localize extends CommandBase {
  private SwerveDrive m_drive;
  private BotPoseProvider m_source;

  /** Creates a new Localize. */
  public Localize(SwerveDrive drive, BotPoseProvider source) {
    m_drive = drive;
    m_source = source;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d sourcePose = m_source.getBotPose();

    if (m_source.isConnected() && sourcePose.getX()!=0 && sourcePose.getY() !=0) {
      m_drive.resetPosition(new Pose2d(m_source.getBotPose().getTranslation(), new Rotation2d(0)));
    } else {
      System.err.println("Did not localize");
      FollowHolonomicTrajectory.forceResetOdometry();
    }

    m_drive.updateOdometry();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.getOdometryPose().getTranslation().getDistance(m_source.getBotPose().getTranslation()) < 0.1;
  }
}
