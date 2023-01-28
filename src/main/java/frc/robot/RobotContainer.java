// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.controllers.DriverController;
import frc.robot.util.controllers.FrskyDriverController;

public class RobotContainer {
  private final SwerveDrive m_drive = new SwerveDrive();
  private final DriverController m_driverController = new FrskyDriverController(Constants.DRIVER_CONTROLLER_ID);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // m_drive.setDefaultCommand(m_drive.fieldOrientedDriveCommandFactory(m_drive, m_driverController));
    m_drive.setDefaultCommand(m_drive.grantDriveCommandFactory(m_drive, m_driverController));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  
}
