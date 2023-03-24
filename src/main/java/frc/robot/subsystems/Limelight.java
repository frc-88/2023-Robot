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
  private double m_length; 
  private double tx;
  private double ty;
  private double pipeHeight = .6223;
  private double m_angle;
  private double heightOverPipe = .3;
  private NetworkTable limelightTable;

  public Limelight(String name) {
    m_name = name;
    limelightTable = NetworkTableInstance.getDefault().getTable(m_name);
  }

  public double limelightAngleCalculator() { 
    
    //Calculates distance from limelight horizontally
    tx = Math.toRadians(LimelightHelpers.getTX(m_name));
    ty = Math.toRadians(LimelightHelpers.getTY(m_name));
    m_length = (pipeHeight/(Math.tan(ty)));
    m_distance = m_length*Math.tan(tx);
    return Math.toDegrees(Math.atan2(heightOverPipe, m_distance))-Constants.ANGLE_CONSTANT;
  }

  public void limelightSwitch(int pipe) {
    NetworkTableEntry limeLightPipe = limelightTable.getEntry("pipeline");
    limeLightPipe.setNumber(pipe);
  }

  public void SetRetroPipeline() {
    limelightSwitch(1);
  }

  public void SetAprilTagPipeline() {
    limelightSwitch(0);
  }

  public CommandBase SetRetroPipelineFactory() {
    return new InstantCommand(this::SetRetroPipeline);
  }

  public CommandBase SetAprilTagPipelineFactory() {
    return new InstantCommand(this::SetAprilTagPipeline);
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

    SmartDashboard.putNumber("LL:" + m_name + "Wrist Aiming", limelightAngleCalculator());
    SmartDashboard.putNumber("LL:" + m_name + ":BotX", botPose.getX());
    SmartDashboard.putNumber("LL:" + m_name + ":BotY", botPose.getY());
    SmartDashboard.putNumber("LL:" + m_name + ":BotYaw", botPose.getRotation().getDegrees());
    SmartDashboard.putNumber("LL:" + m_name + ":M_Distance", m_distance);
  }
}
