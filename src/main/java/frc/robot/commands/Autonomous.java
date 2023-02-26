// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import frc.robot.util.arm.ArmStates;

/** Add your docs here. */
public class Autonomous {

    public static SequentialCommandGroup simpleAuto(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            new Localize(drive, source),
            // intake.setConeFactory(),
            // candle.wantConeFactory(),
            // grabber.forcePivotBackwardsFactory(),
            // new ParallelDeadlineGroup(
            //     intake.stowFactory(),
            //     arm.sendArmToStateAndEnd(ArmStates.scoreCubeHigh),
            //     grabber.holdCubeFactory()
            // ),
            // new ParallelCommandGroup(
            //     intake.stowFactory(),
            //     arm.sendArmToState(ArmStates.scoreCubeHigh),
            //     grabber.dropCubeFactory()
            // ).withTimeout(0.5),
            // new ParallelDeadlineGroup(
                new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("RedCenterLeg1.wpilib.json"), false),
                // intake.intakeFactory()
                    // arm.sendArmToState(ArmStates.stow),
                    // grabber.holdConeFactory()
            // ),
            new WaitCommand(1.0),
            new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("RedCenterLeg2.wpilib.json"), false)
        );
    }

    public static SequentialCommandGroup redEngage(SwerveDrive drive, Grabber grabber, BotPoseProvider source) {
        return new SequentialCommandGroup(
            // new ParallelDeadlineGroup(
                new Localize(drive, source),
                // grabber.holdCubeFactory()
            // ),
            // new ParallelDeadlineGroup(
                new WaitCommand(1.0),
                // grabber.dropCubeFactory()
            // ),
            new FollowTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("RedEngage.wpilib.json"), false),
            drive.lockCommandFactory()
        );
    }
}
