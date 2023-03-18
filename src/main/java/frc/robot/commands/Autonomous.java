// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.AutoBalanceSimple;
import frc.robot.commands.drive.FollowHolonomicTrajectory;
import frc.robot.commands.drive.Localize;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.Aiming;
import frc.robot.util.BotPoseProvider;
import frc.robot.util.TrajectoryHelper;
import frc.robot.util.arm.ArmState;
import frc.robot.util.arm.ArmStates;

/** Add your docs here. */
public class Autonomous {

    public static ConditionalCommand engage(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, BotPoseProvider source) {
        return new ConditionalCommand(engage("Red",drive,intake,arm,grabber,source), 
            engage("Blue",drive,intake,arm, grabber,source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    public static ConditionalCommand wall2(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new ConditionalCommand(wall2("Red", drive, intake, arm, grabber, candle, source), 
            wall2("Blue", drive, intake, arm, grabber, candle, source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    public static ConditionalCommand charge1balance(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new ConditionalCommand(charge1balance("Red", drive, intake, arm, grabber, candle, source), 
            charge1balance("Blue", drive, intake, arm, grabber, candle, source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    public static ConditionalCommand center2Balance(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new ConditionalCommand(center2Balance("Red", drive, intake, arm, grabber, candle, source), 
            center2Balance("Blue", drive, intake, arm, grabber, candle, source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    public static ConditionalCommand center3(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, Aiming aiming, BotPoseProvider source) {
        return new ConditionalCommand(center3("Red", drive, intake, arm, grabber, candle, aiming, source), 
            center3("Blue", drive, intake, arm, grabber, candle, aiming, source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    // keep for now...
    public static ConditionalCommand center3Balance(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, Aiming aiming, BotPoseProvider source) {
        return new ConditionalCommand(center3Balance("Red", drive, intake, arm, grabber, candle, aiming, source), 
            center3Balance("Blue", drive, intake, arm, grabber, candle, aiming, source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    /////////////////////////////////////////////

    private static SequentialCommandGroup engage(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, BotPoseProvider source) {
        return new SequentialCommandGroup(
            initialShootCubeMid(drive, intake, arm, grabber, source),
            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "Engage.wpilib.json"), false).deadlineWith(intake.stowFactory(), arm.holdTargetState(), grabber.holdCubeFactory()),
            new AutoBalanceSimple(drive).alongWith(arm.stowSimple(), grabber.holdCubeFactory())
        );
    }

    private static SequentialCommandGroup center3Balance(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, Aiming aiming, BotPoseProvider source) {
        return new SequentialCommandGroup(
            center3(alliance, drive, intake, arm, grabber, candle, aiming, source),
            centerBalance(alliance, drive, intake, arm, grabber, candle, source)
        );
    }

    private static SequentialCommandGroup center3(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, Aiming aiming, BotPoseProvider source) {
        return new SequentialCommandGroup(
            centerBaseTo1Mid(alliance, drive, intake, arm, grabber, candle, source),
            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid1ToPiece2.wpilib.json"), new Rotation2d(), Rotation2d.fromDegrees(alliance.equals("Blue") ? -35 : 35), false)
                .deadlineWith(intake.intakeFactory(), arm.stowSimple(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelDeadlineGroup(
                new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece2ToGrid3.wpilib.json"), false),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.stowSimple(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    new Handoff(intake, arm, grabber, true, false),
                    arm.sendArmToState(ArmStates.autoPrepScoreCone).alongWith(
                        intake.downFactory(), grabber.grabConeFactory(), grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot()))
                )
            ), 
            arm.sendArmToStateAndEnd(ArmStates.scoreConeMiddle)
                        .deadlineWith(intake.downFactory(), grabber.grabConeFactory().andThen(grabber.holdConeFactory()), 
                            grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot(), grabber.applyAim(alliance.equals("Blue") ? 0 : 50))),
            arm.stowFrom(ArmStates.scoreConeMiddle).alongWith(intake.downFactory(), grabber.dropConeFactory()).withTimeout(0.5),
            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid3ToPiece3.wpilib.json"), false)
                .deadlineWith(intake.stowFactory(), arm.stowSimple(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot(), grabber.applyAim(0)))
        );
    }

    private static SequentialCommandGroup center2Balance(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            centerBaseTo1Mid(alliance, drive, intake, arm, grabber, candle, source),
            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid1ToPiece2.wpilib.json"), new Rotation2d(), Rotation2d.fromDegrees(alliance.equals("Blue") ? -35 : 35), false)
                .deadlineWith(intake.intakeFactory(), arm.stowSimple(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece2ToEngage.wpilib.json"), false),
                    new AutoBalanceSimple(drive)
                ),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.stowSimple(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    arm.sendArmToStateAndEnd(ArmStates.getConeFromIntake1).deadlineWith(grabber.grabConeFactory(), intake.stowFactory()),
                    arm.sendArmToState(ArmStates.stowFlat).alongWith(grabber.grabConeFactory(), intake.handoffFactory())
                )
            )
        );
    }

    private static SequentialCommandGroup centerBaseTo1Mid(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            initialShootCubeMid(drive, intake, arm, grabber, source)
                .deadlineWith(intake.setConeFactory(), candle.wantConeFactory()),
            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid2ToPiece1.wpilib.json"), false)
                .deadlineWith(intake.intakeFactory(), arm.stowSimple(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelDeadlineGroup(
                new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece1ToGrid1.wpilib.json"), false),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.stowSimple(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    new Handoff(intake, arm, grabber, true, true),
                    arm.sendArmToState(ArmStates.autoPrepScoreCone).alongWith(
                        intake.downFactory(), grabber.grabConeFactory(), grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot()))
                )
            ),
            arm.sendArmToStateAndEnd(ArmStates.scoreConeMiddle)
                        .deadlineWith(intake.downFactory(), grabber.grabConeFactory().andThen(grabber.holdConeFactory()), 
                            grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot(), grabber.applyAim(alliance.equals("Blue") ? 0 : 36))),
            arm.stowFrom(ArmStates.scoreConeMiddle).alongWith(grabber.dropConeFactory()).withTimeout(0.5).andThen(grabber.applyAim(0))
        );
    }

    private static SequentialCommandGroup wall2(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            initialShootCubeMid(drive, intake, arm, grabber, source),
            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "WallGrid8ToPiece4.wpilib.json"), false)
                .deadlineWith(intake.intakeFactory(), arm.stowFrom(ArmStates.scoreCubeMiddle), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelDeadlineGroup(
                new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "WallPiece4ToGrid9.wpilib.json"), false),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.stowSimple(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    new Handoff(intake, arm, grabber, true, true),
                    arm.sendArmToState(ArmStates.autoPrepScoreCone).alongWith(
                        intake.downFactory(), grabber.grabConeFactory(), grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot()))
                )
            ),
            arm.sendArmToStateAndEnd(ArmStates.scoreConeMiddle)
                        .deadlineWith(intake.downFactory(), grabber.grabConeFactory().andThen(grabber.holdConeFactory()), 
                            grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot(), grabber.applyAim(alliance.equals("Blue") ? 35 : 0))),
            arm.stowFrom(ArmStates.scoreConeMiddle).alongWith(grabber.dropConeFactory()).withTimeout(0.5).andThen(grabber.applyAim(0))
        );
    }

    private static SequentialCommandGroup charge1balance(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            initialShootCubeMid(drive, intake, arm, grabber, source),
            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "ChargeGrid5ToPiece3.wpilib.json"), false)
                .deadlineWith(intake.intakeFactory(), arm.stowFrom(ArmStates.scoreCubeMiddle), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelDeadlineGroup(
                new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "ChargePiece3ToBalance.wpilib.json"), false),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.stowSimple(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    new Handoff(intake, arm, grabber, true, true),
                    arm.stowSimple()
                        .alongWith(intake.stowFactory(), grabber.grabConeFactory().andThen(grabber.holdConeFactory()),
                            grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot()))
                )
            ),
            new AutoBalanceSimple(drive).alongWith(intake.stowFactory(), grabber.holdConeFactory())
        );
    }

    private static SequentialCommandGroup centerBalance(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterEngage.wpilib.json"), false)
                .deadlineWith(arm.stowSimple(), 
                    grabber.holdConeFactory(), intake.stowFactory()),
            arm.stowSimple().alongWith(grabber.holdConeFactory(), intake.stowFactory())
        );
    }

    private static SequentialCommandGroup initialShootCubeMid(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, BotPoseProvider source) {
        return new SequentialCommandGroup(
            new Localize(drive, source).withTimeout(0.25).alongWith(
                arm.sendArmToState(ArmStates.scoreCubeMiddle).withTimeout(0.3),
                grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot())).deadlineWith(
                intake.downFactory()),
            arm.holdTargetState().alongWith(grabber.dropCubeFactory(), intake.downFactory()).withTimeout(0.15)
        );
    }
}
