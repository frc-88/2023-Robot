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

    public static ConditionalCommand center(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new ConditionalCommand(redCenter(drive, intake, arm, grabber, candle, source), 
            blueCenter(drive, intake, arm, grabber, candle, source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    public static SequentialCommandGroup upAndOver(SwerveDrive drive, Arm arm, Grabber grabber, Intake intake, BotPoseProvider source) {
        return new SequentialCommandGroup(
            new Localize(drive, source).alongWith(grabber.forcePivotBackwardsFactory()),
            grabber.forcePivot(),
            arm.sendArmToStateAndEnd(ArmStates.scoreCubeHigh).deadlineWith(grabber.holdCubeFactory()),
            arm.holdTargetState().alongWith(grabber.dropCubeFactory()).withTimeout(0.5),
            arm.stowFrom(ArmStates.scoreCubeHigh).alongWith(grabber.dropCubeFactory()).withTimeout(0.5),
            new ConditionalCommand(
                new ParallelDeadlineGroup(
                    new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("RedEngageOver.wpilib.json"), false),
                    arm.holdTargetState(),
                    grabber.holdCubeFactory(),
                     new SequentialCommandGroup(
                        new WaitCommand(3.25),
                        intake.intakeFactory()
                     )
                ),
                new ParallelDeadlineGroup(
                    new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("BlueEngageOver.wpilib.json"), false),
                    arm.holdTargetState(),
                    grabber.holdCubeFactory(),
                    new SequentialCommandGroup(
                       new WaitCommand(3.0),
                       intake.intakeFactory()
                    )),
                () -> {return DriverStation.getAlliance() == Alliance.Red;})
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

    public static SequentialCommandGroup redCenter(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            initialScoreCubeHigh(drive, arm, grabber, source)
                .deadlineWith(intake.setConeFactory(), candle.wantConeFactory()),
            new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("RedCenterLeg1.wpilib.json"), false)
                .deadlineWith(intake.intakeFactory(), arm.holdTargetState(), grabber.holdConeFactory(), grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelCommandGroup(
                new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("RedCenterLeg2.wpilib.json"), false),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.holdTargetState(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    new Handoff(intake, arm, grabber, true),
                    arm.sendArmToStateAndEnd(ArmStates.scoreConeHigh).deadlineWith(intake.downFactory(), grabber.centerConeFactory().andThen(grabber.holdConeFactory()), grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot()))
                )
            ),
            arm.stowFrom(ArmStates.scoreConeHigh).alongWith(grabber.dropConeFactory()).withTimeout(0.25)
        );
    }

    public static SequentialCommandGroup blueCenter(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            new Localize(drive, source),
            new WaitCommand(1)
        );
    }

    private static CommandBase initialScoreCubeHigh(SwerveDrive drive, Arm arm, Grabber grabber, BotPoseProvider source) {
        return new SequentialCommandGroup(
            new Localize(drive, source).alongWith(grabber.forcePivotBackwardsFactory()),
            grabber.forcePivot(),
            arm.sendArmToStateAndEnd(ArmStates.scoreCubeHigh).deadlineWith(grabber.holdCubeFactory()),
            arm.stowFrom(ArmStates.scoreCubeHigh).alongWith(grabber.dropCubeFactory()).withTimeout(0.25)
        );
    }
}
