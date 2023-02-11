// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANdleSystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.TrajectoryHelper;
import frc.robot.util.controllers.DriverController;
import frc.robot.util.controllers.FrskyDriverController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.util.controllers.ButtonBox;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /////////////////////////////////////////////////////////////////////////////
  //                              SUBSYSTEMS                                 //
  /////////////////////////////////////////////////////////////////////////////

  private final SwerveDrive m_drive = new SwerveDrive();
  private final Intake m_intake = new Intake();
  private final Limelight m_limelight = new Limelight();

  /////////////////////////////////////////////////////////////////////////////
  //                              CONTROLLERS                                //
  /////////////////////////////////////////////////////////////////////////////

  private final DriverController m_driverController = new FrskyDriverController(Constants.DRIVER_CONTROLLER_ID);
  private final CANdleSystem m_candleSubsystem = new CANdleSystem();
  private final CommandXboxController m_testController = new CommandXboxController(Constants.TEST_CONTROLLER_ID);
  private final ButtonBox m_buttonBox = new ButtonBox(Constants.BUTTON_BOX_ID);



  public RobotContainer() {
    configureControllers();
    configureDefaultCommands();
    configureSmartDashboardButtons();
  }

  private void configureControllers() {
    m_buttonBox.outgestButton.whileTrue(m_intake.outgestFactory());
    m_buttonBox.intakeButton.whileTrue(m_intake.intakeFactory());
    m_buttonBox.gamepieceSwitch.onTrue(m_intake.setConeFactory()).onFalse(m_intake.setCubeFactory());

    // // Test controller
    m_testController.a().onTrue(m_intake.intakeFactory());
    m_testController.x().onTrue(m_intake.holdFactory());
    m_testController.rightBumper().onTrue(m_intake.outgestFactory());
    m_testController.leftBumper().onTrue(m_intake.stowFactory());
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(m_drive.grantDriveCommandFactory(m_drive, m_driverController));
    m_intake.setDefaultCommand(m_intake.holdFactory());
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData("Reset Yaw", m_drive.resetYawCommandFactory());
    SmartDashboard.putData("Field Drive", m_drive.fieldOrientedDriveCommandFactory(m_drive, m_driverController));
    SmartDashboard.putData("Grant Drive", m_drive.grantDriveCommandFactory(m_drive, m_driverController));
    
    SmartDashboard.putData("Want Cone", m_candleSubsystem.wantConeFactory());
    SmartDashboard.putData("Holding Cone", m_candleSubsystem.holdingConeFactory());
    SmartDashboard.putData("Want Cube", m_candleSubsystem.wantCubeFactory());
    SmartDashboard.putData("Holding Cube", m_candleSubsystem.holdingCubeFactory());
   // Intake
    SmartDashboard.putData("Set Mode Cube", m_intake.setCubeFactory());
    SmartDashboard.putData("Set Mode Cone", m_intake.setConeFactory());
    SmartDashboard.putData("Intake Game Piece", m_intake.intakeFactory());
    SmartDashboard.putData("Hold Game Piece", m_intake.holdFactory());
    SmartDashboard.putData("Outgest", m_intake.outgestFactory());
    SmartDashboard.putData("Stow Intake", m_intake.stowFactory());
    SmartDashboard.putData("Handoff Intake", m_intake.handoffFactory());

    SmartDashboard.putData(m_drive);
    SmartDashboard.putData(m_intake);

    SmartDashboard.putData("Auto Simple1", new FollowTrajectory(m_drive, TrajectoryHelper.generatePathWeaverTrajectory("Simple1.wpilib.json"), true));
    SmartDashboard.putData("Auto Drive 5 Feet", new FollowTrajectory(m_drive, TrajectoryHelper.generateStraightTrajectory(5), true));
    SmartDashboard.putData("Auto Drive 10 Feet", new FollowTrajectory(m_drive, TrajectoryHelper.generateStraightTrajectory(10), true));

    SmartDashboard.putData("Reset Robot Pose", m_limelight.llLocalize(m_drive));
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
