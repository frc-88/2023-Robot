// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.TrajectoryHelper;

/** Add your docs here. */
public class Autonomous {

    public static SequentialCommandGroup simpleAuto(SwerveDrive drive, Intake intake, Lights candle) {
        return new SequentialCommandGroup(
            intake.setConeFactory(), 
            candle.wantConeFactory(),
            new ParallelRaceGroup(
                    intake.intakeFactory(),
                    new FollowTrajectory(drive, TrajectoryHelper.generateJSONTrajectory("Simple1Red.wpilib.json"), true)
                )
            );
    }

}
