// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.controllers.DriverController;
import frc.robot.util.controllers.FrskyDriverController;

public class RobotContainer {
  private final SwerveDrive m_drive = new SwerveDrive();
  private final DriverController m_driverController = new FrskyDriverController(Constants.DRIVER_CONTROLLER_ID);
  private final SlewRateLimiter filterX = new SlewRateLimiter(3.0);
  private final SlewRateLimiter filterY = new SlewRateLimiter(3.0);
  private CommandBase m_swerveDrive = new SwerveDriveCommand(
    m_drive,
    () -> modifyAxis(filterY.calculate(m_driverController.getTranslationY())) * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND,
    () -> modifyAxis(filterX.calculate(m_driverController.getTranslationX())) * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND,
    () -> modifyAxis(m_driverController.getRotation()) * SwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
  );
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  
}
