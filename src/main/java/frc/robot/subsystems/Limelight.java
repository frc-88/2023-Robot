// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.BotPoseProvider;
import frc.robot.util.LimelightHelpers;

/*
 *     some think that I need
 * bright green glowing eyes to see
 *      but April guides me
 */

public class Limelight extends SubsystemBase implements BotPoseProvider {

  private String m_name;
  public double m_distance;
  private NetworkTable limelightTable;

  public Limelight(String name) {
    m_name = name;
    limelightTable = NetworkTableInstance.getDefault().getTable(m_name);
  }

  public double getTX() {
    return Math.toRadians(LimelightHelpers.getTX(m_name));
  }

  public double getTY() {
    return Math.toRadians(LimelightHelpers.getTY(m_name) + 25.5);
  }


  public void limelightSwitch(int pipe) {
    NetworkTableEntry limeLightPipe = limelightTable.getEntry("pipeline");
    limeLightPipe.setNumber(pipe);
  }

  public void setRetroMidPipeline() {
    limelightSwitch(1);
  }

  public void setRetroHighPipeline() {
    limelightSwitch(2);
  }

  public void setAprilTagPipeline() {
    limelightSwitch(0);
  }

  public boolean isAprilTagPipelineActive() {
    NetworkTableEntry limeLightPipe = limelightTable.getEntry("pipeline");
    return limeLightPipe.getInteger(0) == 0;
  }

  public boolean isRetroMidPipelineActive() {
    NetworkTableEntry limeLightPipe = limelightTable.getEntry("pipeline");
    return limeLightPipe.getInteger(0) == 1;
  }

  public boolean isRetroHighPipelineActive() {
    NetworkTableEntry limeLightPipe = limelightTable.getEntry("pipeline");
    return limeLightPipe.getInteger(0) == 2;
  }

  public CommandBase setRetroMidPipelineFactory() {
    return new InstantCommand(this::setRetroMidPipeline).ignoringDisable(true);
  }

  public CommandBase setRetroHighPipelineFactory() {
    return new InstantCommand(this::setRetroHighPipeline).ignoringDisable(true);
  }

  public CommandBase setAprilTagPipelineFactory() {
    return new InstantCommand(this::setAprilTagPipeline).ignoringDisable(true);
  }

  public Pose2d getBotPose() {
    if (LimelightHelpers.getFiducialID(m_name) > 0.0) {
      if (DriverStation.getAlliance() == Alliance.Red) {
        return LimelightHelpers.getBotPose2d_wpiRed(m_name);
      } else {
        return LimelightHelpers.getBotPose2d_wpiBlue(m_name);
      }
    } else {
      return new Pose2d();
    }
  }

  public boolean isConnected() {
    return LimelightHelpers.getFiducialID(m_name) > 0.0;
  }

  public InstantCommand llLocalize(SwerveDrive drive) {
    return new InstantCommand(() -> {drive.resetPosition(getBotPose());}, drive);
  }

  @Override
  public void periodic() {
    Pose2d botPose = getBotPose();

    SmartDashboard.putNumber("LL:" + m_name + ":BotX", botPose.getX());
    SmartDashboard.putNumber("LL:" + m_name + ":BotY", botPose.getY());
    SmartDashboard.putNumber("LL:" + m_name + ":BotYaw", botPose.getRotation().getDegrees());
    SmartDashboard.putNumber("LL:" + m_name + ":M_Distance", m_distance);
  }
}
