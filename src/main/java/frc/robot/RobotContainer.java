// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.controllers.DriverController;
import frc.robot.util.controllers.FrskyDriverController;
import frc.robot.subsystems.Intake;
import frc.robot.util.controllers.ButtonBox;

public class RobotContainer {

  /////////////////////////////////////////////////////////////////////////////
  //                              SUBSYSTEMS                                 //
  /////////////////////////////////////////////////////////////////////////////

  private final SwerveDrive m_drive = new SwerveDrive();
  private final Intake m_intake = new Intake();

  /////////////////////////////////////////////////////////////////////////////
  //                              CONTROLLERS                                //
  /////////////////////////////////////////////////////////////////////////////

  private final DriverController m_driverController = new FrskyDriverController(Constants.DRIVER_CONTROLLER_ID);
  private final CommandXboxController m_testController = new CommandXboxController(Constants.TEST_CONTROLLER_ID);
  private final ButtonBox m_buttonBox = new ButtonBox(Constants.BUTTON_BOX_ID);


  public RobotContainer() {
    configureControllers();
    configureDefaultCommands();
    configureSmartDashboardButtons();
  }

  private void configureControllers() {
    m_buttonBox.outgestButton.whileTrue(m_intake.outgestFactory());
    m_buttonBox.intakeButton.and(m_buttonBox.gamepieceSwitch).whileTrue(m_intake.intakeConeFactory());
    m_buttonBox.intakeButton.and(m_buttonBox.gamepieceSwitch.negate()).whileTrue(m_intake.intakeCubeFactory());

    // // Test controller
    m_testController.a().onTrue(m_intake.intakeConeFactory());
    m_testController.b().onTrue(m_intake.intakeCubeFactory());
    m_testController.x().onTrue(m_intake.holdConeFactory());
    m_testController.y().onTrue(m_intake.holdCubeFactory());
    m_testController.rightBumper().onTrue(m_intake.outgestFactory());
    m_testController.leftBumper().onTrue(m_intake.stowFactory());
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(m_drive.grantDriveCommandFactory(m_drive, m_driverController));
    m_intake.setDefaultCommand(m_intake.stowFactory());
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData("Reset Yaw", m_drive.resetYawCommandFactory());
    SmartDashboard.putData("Field Drive", m_drive.fieldOrientedDriveCommandFactory(m_drive, m_driverController));
    SmartDashboard.putData("Grant Drive", m_drive.grantDriveCommandFactory(m_drive, m_driverController));
   // Intake
    SmartDashboard.putData("Intake Cube", m_intake.intakeCubeFactory());
    SmartDashboard.putData("Intake Cone", m_intake.intakeConeFactory());
    SmartDashboard.putData("Hold Cube", m_intake.holdCubeFactory());
    SmartDashboard.putData("Hold Cone", m_intake.holdConeFactory());
    SmartDashboard.putData("Outgest", m_intake.outgestFactory());
    SmartDashboard.putData("Stow Intake", m_intake.stowFactory());
    SmartDashboard.putData("Handoff Intake", m_intake.handoffFactory());

    SmartDashboard.putData(m_drive);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
