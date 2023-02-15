// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.Results;

public class Limelight extends SubsystemBase {

  public Limelight() {

  }

  public InstantCommand llLocalize(SwerveDrive drive) {
    return new InstantCommand (
      () -> {drive.resetPosition(LimelightHelpers.getBotPose2d(Constants.LIMELIGHT_NAME));},
      drive);
  }


  @Override
  public void periodic() {
    Results aprilTagResults = LimelightHelpers.getLatestResults(Constants.LIMELIGHT_NAME).targetingResults;
    Pose3d botPose3d = aprilTagResults.getBotPose3d();
    SmartDashboard.putNumber("LL:AprilTag", aprilTagResults.targets_Fiducials[0].fiducialID);
    SmartDashboard.putNumber("LL:BotX", botPose3d.getX());
    SmartDashboard.putNumber("LL:BotY", botPose3d.getY());
    SmartDashboard.putNumber("LL:BotZ", botPose3d.getZ());
    SmartDashboard.putNumber("LL:BotRoll", Math.toDegrees(botPose3d.getRotation().getX()));
    SmartDashboard.putNumber("LL:BotPitch", Math.toDegrees(botPose3d.getRotation().getY()));
    SmartDashboard.putNumber("LL:BotYaw", Math.toDegrees(botPose3d.getRotation().getZ()));

  }
}
