// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANdleSystem;
//import frc.robot.commands.CANdleConfigCommands;
//import frc.robot.commands.CANdlePrintCommands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.controllers.DriverController;
import frc.robot.util.controllers.FrskyDriverController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive m_drive = new SwerveDrive();
  private final DriverController m_driverController = new FrskyDriverController(Constants.DRIVER_CONTROLLER_ID);
  private final XboxController joy = new XboxController(Constants.JoystickId);
  private final CANdleSystem m_candleSubsystem = new CANdleSystem(joy);

  public RobotContainer() {
    configureControllers();
    configureDefaultCommands();
    configureSmartDashboardButtons();
  }

  private void configureControllers() {
    //new JoystickButton(joy, Constants.BlockButton).whenPressed(m_candleSubsystem::setColors, m_candleSubsystem);
    new JoystickButton(joy, Constants.ConeButton).whenPressed(m_candleSubsystem::setCone, m_candleSubsystem);
    new JoystickButton(joy, Constants.CubeButton).whenPressed(m_candleSubsystem::setCube, m_candleSubsystem);
    new JoystickButton(joy, 9).whenPressed(()->m_candleSubsystem.clearAllAnims(), m_candleSubsystem);
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(m_drive.grantDriveCommandFactory(m_drive, m_driverController));
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData("Reset Yaw", m_drive.resetYawCommandFactory());
    SmartDashboard.putData("Field Drive", m_drive.fieldOrientedDriveCommandFactory(m_drive, m_driverController));
    SmartDashboard.putData("Grant Drive", m_drive.grantDriveCommandFactory(m_drive, m_driverController));
    
    SmartDashboard.putData(m_drive);
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
