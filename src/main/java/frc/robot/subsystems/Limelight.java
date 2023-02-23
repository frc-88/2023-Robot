// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;

public class Limelight extends SubsystemBase {

  private String m_name;

  public Limelight(String name) {
    m_name = name;
  }

  public Pose2d getBotPose() {
    // always use the blue coordinate system
    //      CHARGED UP 2023 FIELD
    //    BLUE                   RED
    //  |----------------------------|
    //  |- 5                      4 -|
    //  |              +Y            |
    //  |-- 6          |        3 -- |
    //  |              o--+X         |
    //  |-- 7                   2 -- |
    //  |                            |
    //  |-- 8                   1 -- |
    //  |----------------------------|
    //          SCORING TABLE
    return LimelightHelpers.getBotPose2d_wpiBlue(m_name);
  }

  public InstantCommand llLocalize(SwerveDrive drive) {
    return new InstantCommand (
      () -> {drive.resetPosition(getBotPose());},
      drive);
  }


  @Override
  public void periodic() {
    Pose2d botPose;
    
    if(LimelightHelpers.getFiducialID(m_name) > 0.0) {
      botPose = getBotPose();
    } else {
      botPose = new Pose2d();
    }

    SmartDashboard.putNumber("LL:"+ m_name +":BotX", Units.metersToFeet(botPose.getX()));
    SmartDashboard.putNumber("LL:"+ m_name +":BotY", Units.metersToFeet(botPose.getY()));
    SmartDashboard.putNumber("LL:"+ m_name +":BotYaw", botPose.getRotation().getDegrees());
  }
}
