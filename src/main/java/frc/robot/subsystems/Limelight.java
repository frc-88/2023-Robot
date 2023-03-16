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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BotPoseProvider;
import frc.robot.util.LimelightHelpers;

/*
 *     some think that I need
 * bright green glowing eyes to see
 *      but April guides me
 */

public class Limelight extends SubsystemBase implements BotPoseProvider {

  private String m_name;
  private double m_distance; 
  private double m_length; 
  private double tx;
  private double ty;
  private double pipeHeight = .6223;
  public NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  public Limelight(String name) {
    m_name = name;
  }

  public double limelightOffset() { 
    NetworkTableEntry limeLightPipe = limelightTable.getEntry("pipeline");
    limeLightPipe.setNumber(1);
    tx = Math.toRadians(LimelightHelpers.getTX("limelight"));
    ty = Math.toRadians(LimelightHelpers.getTY("limelight"));
    m_length = (pipeHeight/(Math.tan(ty)));
    m_distance = m_length*Math.tan(tx);
    return m_distance = Math.tan(tx);
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
  }
}
