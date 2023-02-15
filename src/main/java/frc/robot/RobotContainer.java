// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.PlaySong;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.TrajectoryHelper;
import frc.robot.util.controllers.DriverController;
import frc.robot.util.controllers.FrskyDriverController;
import frc.robot.commands.Handoff;
import frc.robot.commands.PlaySong;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.util.arm.ArmStates;
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
  private final Arm m_arm = new Arm();
  private final Grabber m_grabber = new Grabber(m_arm::coastModeEnabled);

  /////////////////////////////////////////////////////////////////////////////
  //                              CONTROLLERS                                //
  /////////////////////////////////////////////////////////////////////////////

  private final DriverController m_driverController = new FrskyDriverController(Constants.DRIVER_CONTROLLER_ID);
  private final Lights m_candleSubsystem = new Lights();
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

    m_buttonBox.getFromShelfButton.and(m_buttonBox.gamepieceSwitch)
      .onTrue(m_arm.sendArmToState(ArmStates.getConeFromShelf)).onTrue(m_grabber.grabConeFactory());
    m_buttonBox.getFromShelfButton.and(m_buttonBox.gamepieceSwitch.negate())
      .onTrue(m_arm.sendArmToState(ArmStates.getCubeFromShelf)).onTrue(m_grabber.grabCubeFactory());

    m_buttonBox.setLow.and(m_buttonBox.gamepieceSwitch).and(m_drive.isFacingForwards())
      .onTrue(m_arm.sendArmToState(ArmStates.scoreConeLow));
    m_buttonBox.setMiddle.and(m_buttonBox.gamepieceSwitch).and(m_drive.isFacingForwards())
      .onTrue(m_arm.sendArmToState(ArmStates.scoreConeMiddle));
    m_buttonBox.setHigh.and(m_buttonBox.gamepieceSwitch).and(m_drive.isFacingForwards())
      .onTrue(m_arm.sendArmToState(ArmStates.scoreConeHigh));

    m_buttonBox.setLow.and(m_buttonBox.gamepieceSwitch).and(m_drive.isFacingBackwards())
      .onTrue(m_arm.sendArmToState(ArmStates.scoreConeLowFront));
    m_buttonBox.setMiddle.and(m_buttonBox.gamepieceSwitch).and(m_drive.isFacingBackwards())
      .onTrue(m_arm.sendArmToState(ArmStates.scoreConeMiddleFront));

    m_buttonBox.setLow.and(m_buttonBox.gamepieceSwitch.negate()).and(m_drive.isFacingForwards())
      .onTrue(m_arm.sendArmToState(ArmStates.scoreCubeLow));
    m_buttonBox.setMiddle.and(m_buttonBox.gamepieceSwitch.negate()).and(m_drive.isFacingForwards())
      .onTrue(m_arm.sendArmToState(ArmStates.scoreCubeMiddle));
    m_buttonBox.setHigh.and(m_buttonBox.gamepieceSwitch.negate()).and(m_drive.isFacingForwards())
      .onTrue(m_arm.sendArmToState(ArmStates.scoreCubeHigh));

    m_buttonBox.setLow.and(m_buttonBox.gamepieceSwitch.negate()).and(m_drive.isFacingBackwards())
      .onTrue(m_arm.sendArmToState(ArmStates.scoreCubeLowFront));
    m_buttonBox.setMiddle.and(m_buttonBox.gamepieceSwitch.negate()).and(m_drive.isFacingBackwards())
      .onTrue(m_arm.sendArmToState(ArmStates.scoreCubeMiddleFront));

    m_buttonBox.scoreButton.and(m_buttonBox.gamepieceSwitch)
      .onTrue(m_grabber.grabConeFactory());
    m_buttonBox.scoreButton.and(m_buttonBox.gamepieceSwitch.negate())
      .onTrue(m_grabber.grabCubeFactory());

    // m_drive.isFacingForwards().onTrue(m_grabber.setPivotForwardsFactory());
    // m_drive.isFacingBackwards().onTrue(m_grabber.setPivotBackwardsFactory());

    // m_intake.holdAndHasPiece().and(m_grabber.hasGamePieceTrigger().negate())
    //   .onTrue(new Handoff(m_intake, m_arm, m_grabber, m_buttonBox.gamepieceSwitch.getAsBoolean()));

    // // Test controller
    m_testController.a().onTrue(m_intake.intakeFactory());
    m_testController.x().onTrue(m_intake.holdFactory());
    m_testController.rightBumper().onTrue(m_intake.outgestFactory());
    m_testController.leftBumper().onTrue(m_intake.stowFactory());
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(m_drive.grantDriveCommandFactory(m_drive, m_driverController));
    m_intake.setDefaultCommand(m_intake.holdFactory());
    m_arm.setDefaultCommand(m_arm.sendArmToState(ArmStates.stow));
    m_grabber.setDefaultCommand(m_grabber.holdFactory(m_buttonBox::isConeSelected));
  }

  private void configureSmartDashboardButtons() {

    // Drive
    SmartDashboard.putData("Reset Yaw", m_drive.resetYawCommandFactory());
    SmartDashboard.putData("Field Drive", m_drive.fieldOrientedDriveCommandFactory(m_drive, m_driverController));
    SmartDashboard.putData("Grant Drive", m_drive.grantDriveCommandFactory(m_drive, m_driverController));

    // CANdle

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

    // Arm
    SmartDashboard.putData("!!Calibrate Shoulder Absolute!!", m_arm.calibrateShoulderFactory());
    SmartDashboard.putData("!!Calibrate Elbow Absolute!!", m_arm.calibrateElbowFactory());
    SmartDashboard.putData("!!Calibrate Wrist Absolute!!", m_arm.calibrateWristFactory());

    SmartDashboard.putData("Arm Stow", m_arm.sendArmToState(ArmStates.stow));

    // Grabber
    SmartDashboard.putData("!!Calibrate Grabber Pivot Absolute!!", m_grabber.calibrateAbsolutePivotFactory());
    SmartDashboard.putData("Set Grabber Forwards", m_grabber.setPivotForwardsFactory());
    SmartDashboard.putData("Set Grabber Backwards", m_grabber.setPivotBackwardsFactory());
    SmartDashboard.putData("Force Grabber Backwards", m_grabber.forcePivotBackwardsFactory());
    SmartDashboard.putData("Grab Cone", m_grabber.grabConeFactory());
    SmartDashboard.putData("Grab Cube", m_grabber.grabCubeFactory());
    SmartDashboard.putData("Drop Cone", m_grabber.dropConeFactory());
    SmartDashboard.putData("Drop Cube", m_grabber.dropCubeFactory());

    // Subsystems
    SmartDashboard.putData(m_drive);
    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_arm);

    SmartDashboard.putData("Auto Simple1", new FollowTrajectory(m_drive, TrajectoryHelper.generatePathWeaverTrajectory("Simple1.wpilib.json"), true));
    SmartDashboard.putData("Auto Drive 5 Feet", new FollowTrajectory(m_drive, TrajectoryHelper.generateStraightTrajectory(5), true));
    SmartDashboard.putData("Auto Drive 10 Feet", new FollowTrajectory(m_drive, TrajectoryHelper.generateStraightTrajectory(10), true));

    SmartDashboard.putData("Reset Robot Pose", m_limelight.llLocalize(m_drive));
    SmartDashboard.putData("Play Song", new PlaySong("somethingcomfortingrobot.chrp", m_intake, m_drive, m_arm));
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
