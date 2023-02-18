// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;

public class Limelight extends SubsystemBase {

  public Limelight() {

  }

  public Pose2d getBotPose() {
    if (DriverStation.getAlliance() == Alliance.Red) {
      return LimelightHelpers.getBotPose2d_wpiRed(Constants.LIMELIGHT_NAME);
    } else {
      return LimelightHelpers.getBotPose2d_wpiBlue(Constants.LIMELIGHT_NAME);
    }
    
  }

  public InstantCommand llLocalize(SwerveDrive drive) {
    return new InstantCommand (
      () -> {drive.resetPosition(getBotPose());},
      drive);
  }


  @Override
  public void periodic() {
    Pose2d botPose = getBotPose();
    SmartDashboard.putNumber("LL:BotX", Units.metersToFeet(botPose.getX()));
    SmartDashboard.putNumber("LL:BotY", Units.metersToFeet(botPose.getY()));
    SmartDashboard.putNumber("LL:BotYaw", botPose.getRotation().getDegrees());
  }
}
