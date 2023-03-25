// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Lights;
import frc.robot.commands.PlaySong;
import frc.robot.commands.drive.AutoBalanceSimple;
import frc.robot.commands.drive.Localize;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.Aiming;
import frc.robot.util.controllers.DriverController;
import frc.robot.util.controllers.FrskyDriverController;
import frc.robot.commands.Autonomous;
import frc.robot.util.coprocessor.networktables.ScorpionTable;
import frc.robot.commands.Handoff;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GameObjectManager;
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
  //                              CONTROLLERS                                //
  /////////////////////////////////////////////////////////////////////////////

  private final DriverController m_driverController = new FrskyDriverController(Constants.DRIVER_CONTROLLER_ID);
  private final ButtonBox m_buttonBox = new ButtonBox(Constants.BUTTON_BOX_ID);

  /////////////////////////////////////////////////////////////////////////////
  //                              AUTONOMOUS                                 //
  /////////////////////////////////////////////////////////////////////////////

  private CommandBase m_autoCommand = new WaitCommand(15);
  private String m_autoCommandName = "Wait";

  /////////////////////////////////////////////////////////////////////////////
  //                              SUBSYSTEMS                                 //
  /////////////////////////////////////////////////////////////////////////////

  private final SwerveDrive m_drive = new SwerveDrive();
  private final Intake m_intake = new Intake();
  private final Arm m_arm = new Arm();
  private final Grabber m_grabber = new Grabber(m_arm::coastModeEnabled, m_arm::isStowed);
  private final Limelight m_limelight_front = new Limelight(Constants.LIMELIGHT_FRONT_NAME);
  private final Limelight m_limelight_back = new Limelight(Constants.LIMELIGHT_BACK_NAME);
  private final ScorpionTable m_coprocessor = new ScorpionTable(m_drive, m_drive.getNavX(), Constants.COPROCESSOR_ADDRESS, Constants.COPROCESSOR_PORT, Constants.COPROCESSOR_UPDATE_DELAY);
  private final GameObjectManager m_manager = new GameObjectManager(m_coprocessor);
  private final Lights m_candleSubsystem = new Lights(m_drive, m_intake, m_arm, m_grabber, m_coprocessor, m_limelight_back, () -> m_autoCommandName);
  private final Aiming m_aiming = new Aiming(m_drive, m_arm, m_grabber, m_coprocessor, m_limelight_back, m_manager, m_buttonBox.gamepieceSwitch, m_buttonBox.enableAimingSwitch);


  public RobotContainer(Robot robot) {
    m_arm.setStowSuppliers(
      () -> m_intake.isIntaking() || m_intake.hasGamePiece(), 
      () -> m_grabber.hasGamePiece() || !m_grabber.isAtZero(),
      m_buttonBox.hpModeSwitch
      );
  
    configureControllers();
    configureDefaultCommands();
    configureTriggers();
    configureSmartDashboardButtons();
    configurePeriodics(robot);
  }

  public void enableInit() {
    if (m_drive.isFacingBackwards().getAsBoolean() && DriverStation.isTeleop()) {
      new RepeatCommand(m_grabber.setPivotForwardsFactory()).schedule();
    } else if (DriverStation.isTeleop()) {
      new RepeatCommand(m_grabber.setPivotBackwardsFactory()).schedule();
    }
    if (m_buttonBox.gamepieceSwitch.getAsBoolean()) {
      m_candleSubsystem.wantConeFactory().schedule();
      m_intake.setCone();
    } else {
      m_candleSubsystem.wantCubeFactory().schedule();
      m_intake.setCube();
    }
    m_arm.resetMotionMagic();
    m_arm.resetStow();
    m_grabber.aim(0);
  }

  public void disableInit() {
    m_candleSubsystem.rainbow();
    m_arm.resetStow();
    m_limelight_back.setAprilTagPipeline();
  }

  public void disabledPeriodic() {
    if (m_buttonBox.intakeButton.getAsBoolean() && !m_autoCommandName.equals("Wait")) {
      m_autoCommand = new WaitCommand(15);
      m_autoCommandName = "Wait";
    }

    if (m_buttonBox.setHigh.getAsBoolean() && !m_autoCommandName.equals("Engage")) {
      m_autoCommand = Autonomous.engage(m_drive, m_intake, m_arm, m_grabber, m_coprocessor);
      m_autoCommandName = "Engage";
    }

    // if (m_buttonBox.outgestButton.getAsBoolean() && !m_autoCommandName.equals("Center2Link")) {
    //   m_autoCommand = Autonomous.center2Link(m_drive, m_intake, m_arm, m_grabber, m_candleSubsystem, m_coprocessor);
    //   m_autoCommandName = "Center2Link";
    // }

    if (m_buttonBox.handoffButton.getAsBoolean() && !m_autoCommandName.equals("Center2Balance")) {
      m_autoCommand = Autonomous.center2Balance(m_drive, m_intake, m_arm, m_grabber, m_candleSubsystem, m_coprocessor);
      m_autoCommandName = "Center2Balance";
    }

    if (m_buttonBox.setLow.getAsBoolean() && !m_autoCommandName.equals("Center3")) {
      m_autoCommand = Autonomous.center3(m_drive, m_intake, m_arm, m_grabber, m_candleSubsystem, m_aiming, m_coprocessor);
      m_autoCommandName = "Center3";
    }

    if (m_buttonBox.scoreButton.getAsBoolean() && !m_autoCommandName.equals("Center3Balance")) {
      m_autoCommand = Autonomous.center3Balance(m_drive, m_intake, m_arm, m_grabber, m_candleSubsystem, m_aiming, m_coprocessor);
      m_autoCommandName = "Center3Balance";
    }

    if (m_buttonBox.setMiddle.getAsBoolean() && !m_autoCommandName.equals("Charge1.5")) {
      m_autoCommand = Autonomous.charge1balance(m_drive, m_intake, m_arm, m_grabber, m_candleSubsystem, m_coprocessor);
      m_autoCommandName = "Charge1.5";
    }

    if (m_buttonBox.setFlat.getAsBoolean() && !m_autoCommandName.equals("WallSide")) {
      m_autoCommand = Autonomous.wall2(m_drive, m_intake, m_arm, m_grabber, m_candleSubsystem, m_coprocessor);
      m_autoCommandName = "WallSide";
    }

    SmartDashboard.putString("Auto", m_autoCommandName);
  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

  /////////////////////////////////////////////////////////////////////////////
  //                              BUTTON BOX                                 //
  /////////////////////////////////////////////////////////////////////////////

  private void configureControllers() {
    m_driverController.getPivotButton().whileTrue(m_drive.pivotOnCommandFactory()).whileFalse(m_drive.pivotOffCommandFactory());

    m_buttonBox.outgestButton.whileTrue(m_intake.outgestFactory());
    m_buttonBox.intakeButton.whileTrue(m_intake.intakeFactory())
        .and(m_buttonBox.gamepieceSwitch.negate()).onFalse(m_intake.downFactory().withTimeout(0.4));
    m_buttonBox.gamepieceSwitch.onTrue(m_intake.setConeFactory()).onFalse(m_intake.setCubeFactory());

    m_buttonBox.getFromShelfButton.and(m_buttonBox.gamepieceSwitch)
        .whileTrue(m_arm.sendArmToState(ArmStates.getConeFromShelf)).whileTrue(m_grabber.grabConeFactory())
        .onFalse(m_grabber.grabConeFactory().withTimeout(1))
        .whileTrue(m_aiming.noGrabberAimFactory());
    m_buttonBox.getFromShelfButton.and(m_buttonBox.gamepieceSwitch.negate())
        .whileTrue(m_arm.sendArmToState(ArmStates.getCubeFromShelf)).whileTrue(m_grabber.grabCubeFactory())
        .onFalse(m_grabber.grabCubeFactory().withTimeout(1))
        .whileTrue(m_aiming.noGrabberAimFactory());

    m_buttonBox.getFromChuteButton.and(m_buttonBox.gamepieceSwitch)
        .whileTrue(m_arm.sendArmToState(ArmStates.getConeFromChute))
        .whileTrue(m_grabber.grabConeFactory())
        .onFalse(m_grabber.grabConeFactory().withTimeout(1))
        .whileTrue(m_aiming.noGrabberAimFactory());
    m_buttonBox.getFromChuteButton.and(m_buttonBox.gamepieceSwitch.negate())
        .whileTrue(m_arm.sendArmToState(ArmStates.getCubeFromChute))
        .whileTrue(m_grabber.grabCubeFactory())
        .onFalse(m_grabber.grabCubeFactory().withTimeout(1))
        .whileTrue(m_aiming.noGrabberAimFactory());

    m_buttonBox.getFromShelfButton.and(m_buttonBox.gamepieceSwitch)
        .whileTrue(m_arm.sendArmToState(ArmStates.getConeFromShelf))
        .whileTrue(m_grabber.grabConeFactory())
        .onFalse(m_grabber.grabConeFactory().withTimeout(1))
        .whileTrue(m_aiming.noGrabberAimFactory());
    m_buttonBox.getFromShelfButton.and(m_buttonBox.gamepieceSwitch.negate())
        .whileTrue(m_arm.sendArmToState(ArmStates.getCubeFromShelf))
        .whileTrue(m_grabber.grabCubeFactory())
        .onFalse(m_grabber.grabCubeFactory().withTimeout(1))
        .whileTrue(m_aiming.noGrabberAimFactory());

    m_buttonBox.setLow.and(m_buttonBox.gamepieceSwitch).and(m_drive.isFacingForwards())
        .whileTrue(m_arm.sendArmToState(ArmStates.scoreConeLow))
        .whileTrue(m_aiming.noGrabberAimFactory())
        .whileTrue(m_grabber.centerConeFactory().andThen(m_grabber.holdConeFactory()));
    m_buttonBox.setMiddle.and(m_buttonBox.gamepieceSwitch).and(m_drive.isFacingForwards())
        .whileTrue(
          m_arm.sendArmToState(ArmStates.scoreConeMiddle)
            .alongWith(m_grabber.centerConeFactory().andThen(m_grabber.holdConeFactory()))
            .until(() -> m_aiming.readyToScore(true) && m_drive.notMoving())
            .andThen(
              m_arm.holdTargetState()
                .alongWith(m_grabber.dropConeFactory())
                .withTimeout(0.1),
              m_arm.stowFrom(ArmStates.scoreConeMiddle)
                .deadlineWith(m_grabber.dropConeFactory())
            )
        )
        .whileTrue(m_aiming.aimGrabberFactory(Constants.AIM_MIDDLE_OUTREACH, true));
    m_buttonBox.setHigh.and(m_buttonBox.gamepieceSwitch).and(m_drive.isFacingForwards())
        .whileTrue(
          m_arm.sendArmToState(ArmStates.scoreConeHigh)
            .alongWith(m_grabber.centerConeFactory().andThen(m_grabber.holdConeFactory()))
            .until(() -> m_aiming.readyToScore(false) && m_drive.notMoving())
            .andThen(
              m_arm.holdTargetState()
                .alongWith(m_grabber.dropConeFactory())
                .withTimeout(0.1),
              m_arm.stowFrom(ArmStates.scoreConeHigh)
                .deadlineWith(m_grabber.dropConeFactory())
            )
        )
        .whileTrue(m_aiming.aimGrabberFactory(Constants.AIM_HIGH_OUTREACH, false))
        .onTrue(m_intake.downFactory())
        .onFalse(m_intake.downFactory().withTimeout(0.25));

    m_buttonBox.setLow.and(m_buttonBox.gamepieceSwitch).and(m_drive.isFacingBackwards())
        .whileTrue(m_arm.sendArmToState(ArmStates.scoreConeLowFront))
        .whileTrue(m_aiming.noGrabberAimFactory());
    m_buttonBox.setMiddle.and(m_buttonBox.gamepieceSwitch).and(m_drive.isFacingBackwards())
        .whileTrue(m_arm.sendArmToState(ArmStates.scoreConeMiddleFront))
        .whileTrue(m_aiming.aimGrabberFactory(0, true));

    m_buttonBox.setLow.and(m_buttonBox.gamepieceSwitch.negate()).and(m_drive.isFacingForwards())
        .whileTrue(m_arm.sendArmToState(ArmStates.scoreCubeLow))
        .whileTrue(m_aiming.noGrabberAimFactory());
    m_buttonBox.setMiddle.and(m_buttonBox.gamepieceSwitch.negate()).and(m_drive.isFacingForwards())
        .whileTrue(m_arm.sendArmToState(ArmStates.scoreCubeMiddle))
        .whileTrue(m_aiming.aimGrabberFactory(Constants.AIM_MIDDLE_OUTREACH, true));
    m_buttonBox.setHigh.and(m_buttonBox.gamepieceSwitch.negate()).and(m_drive.isFacingForwards())
        .whileTrue(m_arm.sendArmToState(ArmStates.scoreCubeHigh))
        .whileTrue(m_aiming.aimGrabberFactory(Constants.AIM_HIGH_OUTREACH, false))
        .and(() -> m_coprocessor.isInCommunity(m_drive.getPoseEstimate()))
        .onTrue(m_intake.downFactory())
        .onFalse(m_intake.downFactory()
          .until(() -> !m_coprocessor.isInCommunity(m_drive.getPoseEstimate()))
          .withTimeout(0.5));

    m_buttonBox.setLow.and(m_buttonBox.gamepieceSwitch.negate()).and(m_drive.isFacingBackwards())
        .whileTrue(m_arm.sendArmToState(ArmStates.scoreCubeLowFront))
        .whileTrue(m_aiming.noGrabberAimFactory());
    m_buttonBox.setMiddle.and(m_buttonBox.gamepieceSwitch.negate()).and(m_drive.isFacingBackwards())
        .whileTrue(m_arm.sendArmToState(ArmStates.scoreCubeMiddleFront))
        .whileTrue(m_aiming.aimGrabberFactory(0, true));

    m_buttonBox.setFlat.whileTrue(m_arm.sendArmToState(ArmStates.flat));

    m_buttonBox.handoffButton.and(m_buttonBox.gamepieceSwitch)
        .onTrue(new Handoff(m_intake, m_arm, m_grabber, true, false));
        m_buttonBox.handoffButton.and(m_buttonBox.gamepieceSwitch.negate())
        .onTrue(new Handoff(m_intake, m_arm, m_grabber, false, false));

    m_buttonBox.scoreButton.or(m_driverController.getScoreButton()).and(m_buttonBox.gamepieceSwitch)
        .whileTrue(m_grabber.dropConeFactory());
    m_buttonBox.scoreButton.or(m_driverController.getScoreButton()).and(m_buttonBox.gamepieceSwitch.negate())
        .whileTrue(m_grabber.dropCubeFactory());

    m_buttonBox.gamepieceSwitch.and(m_grabber.hasGamePieceTrigger().negate())
        .whileTrue(m_candleSubsystem.wantConeFactory());
    m_buttonBox.gamepieceSwitch.and(m_grabber.hasGamePieceTrigger()).whileTrue(m_candleSubsystem.holdingConeFactory());
    m_buttonBox.gamepieceSwitch.negate().and(m_grabber.hasGamePieceTrigger().negate())
        .whileTrue(m_candleSubsystem.wantCubeFactory());
    m_buttonBox.gamepieceSwitch.negate().and(m_grabber.hasGamePieceTrigger())
        .whileTrue(m_candleSubsystem.holdingCubeFactory());
  }

  /////////////////////////////////////////////////////////////////////////////
  //                               TRIGGERS                                  //
  /////////////////////////////////////////////////////////////////////////////

  private void configureTriggers() {
    m_drive.isFacingForwards().and(m_buttonBox.forcePivotForwardsSwitch.negate()).and(m_buttonBox.forcePivotBackwardsSwitch.negate()).and(DriverStation::isTeleopEnabled).whileTrue(new RepeatCommand(m_grabber.setPivotBackwardsFactory()));
    m_buttonBox.forcePivotBackwardsSwitch.and(DriverStation::isTeleopEnabled).whileTrue(new RepeatCommand(m_grabber.forcePivotBackwardsFactory()));
    m_drive.isFacingBackwards().and(m_buttonBox.forcePivotBackwardsSwitch.negate()).or(m_buttonBox.forcePivotForwardsSwitch).and(DriverStation::isTeleopEnabled).whileTrue(new RepeatCommand(m_grabber.setPivotForwardsFactory()));

    m_intake.holdAndHasPiece().and(m_grabber.hasGamePieceTrigger().negate()).and(m_buttonBox.gamepieceSwitch).and(DriverStation::isTeleopEnabled)
        .onTrue(new Handoff(m_intake, m_arm, m_grabber, true, false));
    m_intake.holdAndHasPiece().and(m_grabber.hasGamePieceTrigger().negate()).and(m_buttonBox.gamepieceSwitch.negate()).and(DriverStation::isTeleopEnabled)
        .onTrue(new Handoff(m_intake, m_arm, m_grabber, false, false));

    m_grabber.hasGamePieceTrigger().and(m_arm::isStowed).and(m_buttonBox.gamepieceSwitch).and(DriverStation::isTeleopEnabled)
        .whileTrue(m_grabber.centerConeFactory());
  }

  /////////////////////////////////////////////////////////////////////////////
  //                          DEFAULT COMMANDS                               //
  /////////////////////////////////////////////////////////////////////////////

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(m_drive.grantDriveCommandFactory(m_drive, m_driverController));
    m_intake.setDefaultCommand(m_intake.stowFactory());
    m_arm.setDefaultCommand(m_arm.sendArmToState(ArmStates.stowGeneric).alongWith(m_aiming.noGrabberAimFactory()));
    m_grabber.setDefaultCommand(m_grabber.holdFactory(m_buttonBox::isConeSelected));
  }

  /////////////////////////////////////////////////////////////////////////////
  //                         SMARTDASHBOARD BUTTONS                          //
  /////////////////////////////////////////////////////////////////////////////

  private void configureSmartDashboardButtons() {
    // Subsystems
    SmartDashboard.putData(m_drive);
    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_arm);
    SmartDashboard.putData(m_grabber);

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
    SmartDashboard.putData("Outgest", m_intake.outgestFactory());
    SmartDashboard.putData("Stow Intake", m_intake.stowFactory());
    SmartDashboard.putData("Handoff Intake", m_intake.handoffFactory());

    // Arm
    SmartDashboard.putData("!!Calibrate Shoulder Absolute!!", m_arm.calibrateShoulderFactory());
    SmartDashboard.putData("!!Calibrate Elbow Absolute!!", m_arm.calibrateElbowFactory());
    SmartDashboard.putData("!!Calibrate Wrist Absolute!!", m_arm.calibrateWristFactory());

    // Grabber
    SmartDashboard.putData("!!Calibrate Grabber Pivot Absolute!!", m_grabber.calibrateAbsolutePivotFactory());
    SmartDashboard.putData("Set Grabber Forwards", m_grabber.setPivotForwardsFactory());
    SmartDashboard.putData("Set Grabber Backwards", m_grabber.setPivotBackwardsFactory());
    SmartDashboard.putData("Force Grabber Backwards", m_grabber.forcePivotBackwardsFactory());
    SmartDashboard.putData("Grab Cone", m_grabber.grabConeFactory());
    SmartDashboard.putData("Grab Cube", m_grabber.grabCubeFactory());
    SmartDashboard.putData("Drop Cone", m_grabber.dropConeFactory());
    SmartDashboard.putData("Drop Cube", m_grabber.dropCubeFactory());

    // Localization
    SmartDashboard.putData("LL Front Localize", new Localize(m_drive, m_limelight_front).ignoringDisable(true));
    SmartDashboard.putData("LL Back Localize", new Localize(m_drive, m_limelight_back).ignoringDisable(true));
    SmartDashboard.putData("ROS Localize", new Localize(m_drive, m_coprocessor).ignoringDisable(true));

    // Combined
    SmartDashboard.putData("Handoff Cone", new Handoff(m_intake, m_arm, m_grabber, true, false));
    SmartDashboard.putData("Handoff Cube", new Handoff(m_intake, m_arm, m_grabber, false, false));

    // Autonomous
    SmartDashboard.putData("Auto Balance Simple", new AutoBalanceSimple(m_drive));

    // Misc
    SmartDashboard.putData("Play Song", new PlaySong("somethingcomfortingrobot.chrp", m_intake, m_drive, m_arm));
    SmartDashboard.putData("Retro Mid Pipeline", m_limelight_back.setRetroMidPipelineFactory());
    SmartDashboard.putData("Retro High Pipeline", m_limelight_back.setRetroHighPipelineFactory());
    SmartDashboard.putData("April Tag Pipeline", m_limelight_back.setAprilTagPipelineFactory());
  }
  
  private void configurePeriodics(Robot robot) {
    robot.addPeriodic(m_coprocessor::update, Constants.COPROCESSOR_UPDATE_DELAY, Constants.COPROCESSOR_UPDATE_DELAY_OFFSET);
    robot.addPeriodic(m_coprocessor::updateSlow, Constants.COPROCESSOR_SLOW_UPDATE_DELAY, Constants.COPROCESSOR_SLOW_UPDATE_DELAY_OFFSET);
  }

} 