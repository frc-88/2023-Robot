// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  private final XboxController joy = new XboxController(Constants.JoystickId);
  
  private final CANdleSystem m_candleSubsystem = new CANdleSystem(joy);

  public RobotContainer() {
    configureBindings();
  }

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
