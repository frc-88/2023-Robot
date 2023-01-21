// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.commands.CANdleConfigCommands;
import frc.robot.commands.CANdlePrintCommands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController joy = new XboxController(Constants.JoystickId);
  
  private final CANdleSystem m_candleSubsystem = new CANdleSystem(joy);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joy, Constants.BlockButton).whenPressed(m_candleSubsystem::setColors, m_candleSubsystem);
    new JoystickButton(joy, Constants.IncrementAnimButton).whenPressed(m_candleSubsystem::incrementAnimation, m_candleSubsystem);
    new JoystickButton(joy, Constants.DecrementAnimButton).whenPressed(m_candleSubsystem::decrementAnimation, m_candleSubsystem);

    new POVButton(joy, Constants.MaxBrightnessAngle).whenPressed(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 1.0));
    new POVButton(joy, Constants.MidBrightnessAngle).whenPressed(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 0.3));
    new POVButton(joy, Constants.ZeroBrightnessAngle).whenPressed(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 0));
    new POVButton(joy, Constants.ChangeDirectionAngle).whenPressed(()->m_candleSubsystem.toggleAnimDirection(), m_candleSubsystem);

    new JoystickButton(joy, 9).whenPressed(()->m_candleSubsystem.clearAllAnims(), m_candleSubsystem);
    new JoystickButton(joy, 10).whenPressed(()->m_candleSubsystem.toggle5VOverride(), m_candleSubsystem);

    new JoystickButton(joy, Constants.VbatButton).whenPressed(new CANdlePrintCommands.PrintVBat(m_candleSubsystem));
    new JoystickButton(joy, Constants.V5Button).whenPressed(new CANdlePrintCommands.Print5V(m_candleSubsystem));
    new JoystickButton(joy, Constants.CurrentButton).whenPressed(new CANdlePrintCommands.PrintCurrent(m_candleSubsystem));
    new JoystickButton(joy, Constants.TemperatureButton).whenPressed(new CANdlePrintCommands.PrintTemperature(m_candleSubsystem));
  }
  
  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}