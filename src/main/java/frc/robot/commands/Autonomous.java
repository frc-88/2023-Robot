// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
import frc.robot.util.arm.ArmState;
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
            center2("Blue", drive, intake, arm, grabber, candle, source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    public static ConditionalCommand center2Balance(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new ConditionalCommand(center2Balance("Red", drive, intake, arm, grabber, candle, source), 
            center2Balance("Blue", drive, intake, arm, grabber, candle, source),
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
                new Handoff(intake, arm, grabber, true),
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

    public static SequentialCommandGroup center2Balance(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            centerBase(alliance, drive, intake, arm, grabber, candle, source, "CenterPiece1ToGrid3.wpilib.json"),
            new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterEngage.wpilib.json"), false)
                .deadlineWith(arm.holdTargetState().until(() -> arm.isAtTarget(ArmStates.stow)).andThen(arm.sendArmToState(ArmStates.flat)), 
                    grabber.holdConeFactory(), intake.stowFactory()),
            arm.holdTargetState().alongWith(grabber.holdConeFactory(), intake.stowFactory())
        );
    }

    public static SequentialCommandGroup center2(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return centerBase(alliance, drive, intake, arm, grabber, candle, source, "CenterPiece1ToGrid1.wpilib.json");
    }

    private static SequentialCommandGroup centerBase(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source, String returnPath) {
        return new SequentialCommandGroup(
            initialScoreCubeHigh(drive, arm, grabber, source)
                .deadlineWith(intake.setConeFactory(), candle.wantConeFactory()),
            new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid2ToPiece1.wpilib.json"), false)
                .deadlineWith(intake.intakeFactory(), arm.holdTargetState(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelCommandGroup(
                new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + returnPath), false),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.holdTargetState(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    new Handoff(intake, arm, grabber, true),
                    arm.sendArmToStateAndEnd(ArmStates.scoreConeHigh)
                        .deadlineWith(intake.downFactory(), grabber.centerConeFactory().andThen(grabber.holdConeFactory()), 
                            grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot()))
                )
            ),
            arm.stowFrom(ArmStates.scoreConeHigh).alongWith(grabber.dropConeFactory(), new Localize(drive, source)).withTimeout(0.5)
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
