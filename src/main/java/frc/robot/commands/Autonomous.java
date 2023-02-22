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
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.TrajectoryHelper;
import frc.robot.util.arm.ArmStates;

/** Add your docs here. */
public class Autonomous {

    public static SequentialCommandGroup simpleAuto(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle) {
        return new SequentialCommandGroup(
            intake.setConeFactory(),
            candle.wantConeFactory(),
            grabber.forcePivotBackwardsFactory(),
            new ParallelDeadlineGroup(
                intake.stowFactory(),
                arm.sendArmToStateAndEnd(ArmStates.scoreCubeHigh),
                grabber.holdCubeFactory()
            ),
            new ParallelCommandGroup(
                intake.stowFactory(),
                arm.sendArmToState(ArmStates.scoreCubeHigh),
                grabber.dropCubeFactory()
            ).withTimeout(0.5),
            new ParallelRaceGroup(
                    intake.intakeFactory(),
                    new FollowTrajectory(drive, TrajectoryHelper.generateJSONTrajectory("RedCenterLeg1.wpilib.json"), true),
                    arm.sendArmToState(ArmStates.stow),
                    grabber.holdConeFactory()
            ),
            new WaitCommand(1.0),
            new FollowTrajectory(drive, TrajectoryHelper.generateJSONTrajectory("RedCenterLeg2.wpilib.json"), false)
            );
    }

    public static SequentialCommandGroup redEngage(SwerveDrive drive) {
        return new SequentialCommandGroup(
            new FollowTrajectory(drive, TrajectoryHelper.generateJSONTrajectory("RedEngage.wpilib.json"), true),
            new RunCommand(()->{drive.stop();}, drive)
        );
    }
}
