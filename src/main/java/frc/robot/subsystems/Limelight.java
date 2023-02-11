// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public Limelight() {}

  public InstantCommand llLocalize(SwerveDrive drive, NetworkTable networkTable) {
    return new InstantCommand (
      () -> {drive.resetPosition(LimelightHelpers.getBotPose2d(getName()));},
      drive);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
