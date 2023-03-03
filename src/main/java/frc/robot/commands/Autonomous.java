// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AutoBalanceSimple;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.drive.Localize;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.BotPoseProvider;
import frc.robot.util.TrajectoryHelper;
import frc.robot.util.arm.ArmStates;

/** Add your docs here. */
public class Autonomous {

    public static ConditionalCommand engage(SwerveDrive drive, Arm arm, Grabber grabber, BotPoseProvider source) {
        return new ConditionalCommand(redEngage(drive,arm,grabber,source), 
            blueEngage(drive,grabber,source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    public static ConditionalCommand center2(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new ConditionalCommand(center2("Red", drive, intake, arm, grabber, candle, source), 
            center2("Red", drive, intake, arm, grabber, candle, source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    public static ConditionalCommand center2Balance(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new ConditionalCommand(center2Balance("Red", drive, intake, arm, grabber, candle, source), 
            center2Balance("Red", drive, intake, arm, grabber, candle, source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    public static ConditionalCommand center3(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new ConditionalCommand(center3("Red", drive, intake, arm, grabber, candle, source), 
            center3("Red", drive, intake, arm, grabber, candle, source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    public static ConditionalCommand center3Balance(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new ConditionalCommand(center3Balance("Red", drive, intake, arm, grabber, candle, source), 
            center3Balance("Red", drive, intake, arm, grabber, candle, source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    public static SequentialCommandGroup upAndOver(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            initialScoreCubeHigh(drive, arm, grabber, source)
                .deadlineWith(intake.setConeFactory(), candle.wantConeFactory()),
            new ConditionalCommand(
                new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("RedEngageOver.wpilib.json"), false),
                new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("BlueEngageOver.wpilib.json"), false),
                () -> {return DriverStation.getAlliance() == Alliance.Red;})
               .deadlineWith(new WaitCommand(3.5).andThen(intake.intakeFactory()), arm.holdTargetState(), grabber.holdConeFactory(), grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new WaitCommand(0.5),
            new SequentialCommandGroup(
                intake.stowFactory().alongWith(arm.holdTargetState(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                new Handoff(intake, arm, grabber, true, false),
                new ConditionalCommand(
                            new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("RedEngageBack.wpilib.json"), false),
                            new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("BlueEngageBack.wpilib.json"), false),
                            () -> {return DriverStation.getAlliance() == Alliance.Red;})
                // arm.sendArmToStateAndEnd(ArmStates.scoreConeMiddle).deadlineWith(intake.downFactory(), grabber.centerConeFactory().andThen(grabber.holdConeFactory()), grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot()))
            ),
            new AutoBalanceSimple(drive)
            //drive.lockCommandFactory().alongWith(arm.holdTargetState(), grabber.holdConeFactory())
            // arm.stowFrom(ArmStates.scoreConeMiddle).alongWith(grabber.dropConeFactory()).withTimeout(0.25)
        );
    }

    public static SequentialCommandGroup redEngage(SwerveDrive drive, Arm arm, Grabber grabber, BotPoseProvider source) {
        return new SequentialCommandGroup(
            initialScoreCubeHigh(drive, arm, grabber, source),
            new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("RedEngage.wpilib.json"), false).deadlineWith(arm.holdTargetState(), grabber.holdCubeFactory()),
            drive.lockCommandFactory().alongWith(arm.holdTargetState(), grabber.holdCubeFactory())
        );
    }

    public static SequentialCommandGroup blueEngage(SwerveDrive drive, Grabber grabber, BotPoseProvider source) {
        return new SequentialCommandGroup(
            new WaitCommand(1)
        );
    }

    public static SequentialCommandGroup center3Balance(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            center3(alliance, drive, intake, arm, grabber, candle, source),
            centerBalance(alliance, drive, intake, arm, grabber, candle, source)
        );
    }

    public static SequentialCommandGroup center3(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            centerBaseTo1(alliance, drive, intake, arm, grabber, candle, source),
            new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid1ToPiece2.wpilib.json"), false)
                .deadlineWith(intake.intakeFactory(), arm.holdTargetState(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelCommandGroup(
                new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece2ToGrid1.wpilib.json"), false),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.holdTargetState(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    new Handoff(intake, arm, grabber, true, false),
                    arm.sendArmToStateAndEnd(ArmStates.scoreConeMiddle)
                        .deadlineWith(intake.downFactory(), grabber.centerConeFactory().andThen(grabber.holdConeFactory()), 
                            grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot()))
                )
            ),
            arm.stowFrom(ArmStates.scoreConeMiddle).alongWith(grabber.dropConeFactory(), new Localize(drive, source)).withTimeout(0.75)
        );
    }

    public static SequentialCommandGroup center2Balance(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            centerBaseTo1(alliance, drive, intake, arm, grabber, candle, source),
            new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid1ToPiece2.wpilib.json"), false)
                .deadlineWith(intake.intakeFactory(), arm.holdTargetState(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelCommandGroup(
                new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece2ToEngage.wpilib.json"), false),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.holdTargetState(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    arm.sendArmToStateAndEnd(ArmStates.getConeFromIntake1).deadlineWith(grabber.grabConeFactory(), intake.stowFactory()),
                    arm.sendArmToState(ArmStates.flat).alongWith(grabber.grabConeFactory(), intake.handoffFactory())
                )
            )
        );
    }

    public static SequentialCommandGroup center2(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return centerBaseTo1(alliance, drive, intake, arm, grabber, candle, source);
    }

    private static SequentialCommandGroup centerBaseTo1(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            initialScoreCubeHigh(drive, arm, grabber, source)
                .deadlineWith(intake.setConeFactory(), candle.wantConeFactory()),
            new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid2ToPiece1.wpilib.json"), false)
                .deadlineWith(intake.intakeFactory(), arm.holdTargetState(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelCommandGroup(
                new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece1ToGrid1.wpilib.json"), false),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.holdTargetState(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    new Handoff(intake, arm, grabber, true, true),
                    arm.sendArmToStateAndEnd(ArmStates.scoreConeHigh)
                        .deadlineWith(intake.downFactory(), grabber.centerConeFactory().andThen(grabber.holdConeFactory()), 
                            grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot()))
                )
            ),
            arm.stowFrom(ArmStates.scoreConeHigh).alongWith(grabber.dropConeFactory()).withTimeout(0.75)
        );
    }

    private static SequentialCommandGroup centerBaseTo3(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            initialScoreCubeHigh(drive, arm, grabber, source)
                .deadlineWith(intake.setConeFactory(), candle.wantConeFactory()),
            new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid2ToPiece1.wpilib.json"), false)
                .deadlineWith(intake.intakeFactory(), arm.holdTargetState(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece1ToGrid3-1.wpilib.json"), false),
                    // new WaitCommand(0.25),
                    // new Localize(drive, source).withTimeout(0.25),
                    new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece1ToGrid3-2.wpilib.json"), false)
                ),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.holdTargetState(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    new Handoff(intake, arm, grabber, true, false),
                    arm.holdTargetState().alongWith(intake.stowFactory(), grabber.holdConeFactory()).withTimeout(1),
                    arm.sendArmToStateAndEnd(ArmStates.scoreConeHigh)
                        .deadlineWith(intake.downFactory(), grabber.centerConeFactory().andThen(grabber.holdConeFactory()), 
                            grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot()))
                )
            ),
            arm.stowFrom(ArmStates.scoreConeHigh).alongWith(grabber.dropConeFactory()).withTimeout(0.5)
        );
    }

    private static SequentialCommandGroup centerBalance(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterEngage.wpilib.json"), false)
                .deadlineWith(arm.holdTargetState().until(() -> arm.isAtTarget(ArmStates.stow)).andThen(arm.sendArmToState(ArmStates.flat)), 
                    grabber.holdConeFactory(), intake.stowFactory()),
            arm.holdTargetState().alongWith(grabber.holdConeFactory(), intake.stowFactory())
        );
    }

    private static SequentialCommandGroup initialScoreCubeHigh(SwerveDrive drive, Arm arm, Grabber grabber, BotPoseProvider source) {
        return new SequentialCommandGroup(
            new Localize(drive, source).alongWith(grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot())).withTimeout(0.25),
            arm.sendArmToStateAndEnd(ArmStates.scoreCubeHigh).deadlineWith(grabber.holdCubeFactory()),
            arm.stowFrom(ArmStates.scoreCubeHigh).alongWith(grabber.dropCubeFactory()).withTimeout(0.25)
        );
    }
}
