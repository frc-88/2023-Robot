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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AutoBalanceSimple;
import frc.robot.commands.drive.FollowHolonomicTrajectory;
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

    public static ConditionalCommand engage(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, BotPoseProvider source) {
        return new ConditionalCommand(engage("Red",drive,intake,arm,grabber,source), 
            engage("Blue",drive,intake,arm, grabber,source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    public static ConditionalCommand center2(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new ConditionalCommand(center2("Red", drive, intake, arm, grabber, candle, source), 
            center2("Blue", drive, intake, arm, grabber, candle, source),
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

    // copied from center3
    public static ConditionalCommand center2Link(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new ConditionalCommand(center2Link("Red", drive, intake, arm, grabber, candle, source), 
            center2Link("Blue", drive, intake, arm, grabber, candle, source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    public static ConditionalCommand center3(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new ConditionalCommand(center3("Red", drive, intake, arm, grabber, candle, source), 
            center3("Blue", drive, intake, arm, grabber, candle, source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    public static ConditionalCommand center3Balance(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new ConditionalCommand(center3Balance("Red", drive, intake, arm, grabber, candle, source), 
            center3Balance("Blue", drive, intake, arm, grabber, candle, source),
            () -> {return DriverStation.getAlliance() == Alliance.Red;});
    }

    public static SequentialCommandGroup upAndOver(SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            initialScoreCubeHigh(drive, arm, grabber, source)
                .deadlineWith(intake.setConeFactory(), candle.wantConeFactory()),
            new ConditionalCommand(
                new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("RedEngageOver.wpilib.json"), false),
                new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("BlueEngageOver.wpilib.json"), false),
                () -> {return DriverStation.getAlliance() == Alliance.Red;})
               .deadlineWith(new WaitCommand(3.5).andThen(intake.intakeFactory()), arm.stowSimple(), grabber.holdConeFactory(), grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new WaitCommand(0.5),
            new SequentialCommandGroup(
                intake.stowFactory().alongWith(arm.stowSimple(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                new Handoff(intake, arm, grabber, true, false),
                new ConditionalCommand(
                            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("RedEngageBack.wpilib.json"), false),
                            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory("BlueEngageBack.wpilib.json"), false),
                            () -> {return DriverStation.getAlliance() == Alliance.Red;})
                // arm.sendArmToStateAndEnd(ArmStates.scoreConeMiddle).deadlineWith(intake.downFactory(), grabber.centerConeFactory().andThen(grabber.holdConeFactory()), grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot()))
            ),
            new AutoBalanceSimple(drive)
            //drive.lockCommandFactory().alongWith(arm.stowSimple(), grabber.holdConeFactory())
            // arm.stowFrom(ArmStates.scoreConeMiddle).alongWith(grabber.dropConeFactory()).withTimeout(0.25)
        );
    }

    public static SequentialCommandGroup engage(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, BotPoseProvider source) {
        return new SequentialCommandGroup(
            initialShootCubeMid(drive, intake, arm, grabber, source),
            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "Engage.wpilib.json"), false).deadlineWith(intake.stowFactory(), arm.holdTargetState(), grabber.holdCubeFactory()),
            new AutoBalanceSimple(drive).alongWith(arm.stowSimple(), grabber.holdCubeFactory())
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
            centerBaseTo1Mid(alliance, drive, intake, arm, grabber, candle, source),
            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid1ToPiece2.wpilib.json"), false)
                .deadlineWith(intake.intakeFactory(), arm.stowSimple(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelCommandGroup(
                new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece2ToGrid1.wpilib.json"), false),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.stowSimple(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    new Handoff(intake, arm, grabber, true, false),
                    arm.sendArmToState(ArmStates.scoreConeMiddle).until(() -> arm.isAtTarget(ArmStates.scoreConeMiddle, 10))
                        .deadlineWith(intake.downFactory(), grabber.grabConeFactory().andThen(grabber.holdConeFactory()))),
                    grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot(), grabber.applyAim(alliance.equals("Blue") ? -60 : 0)
                )
            ), 
            arm.stowFrom(ArmStates.scoreConeMiddle).alongWith(grabber.dropConeFactory(), new Localize(drive, source)).withTimeout(0.75).andThen(grabber.applyAim(0))
        );
    }

    public static SequentialCommandGroup center2Link(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            centerBaseTo1High(alliance, drive, intake, arm, grabber, candle, source),
            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid1ToPiece2.wpilib.json"), false)
                .deadlineWith(intake.intakeFactory(), arm.stowSimple(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelCommandGroup(
                new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece2ToGrid1.wpilib.json"), false),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.stowSimple(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    new Handoff(intake, arm, grabber, true, false),
                    arm.stowSimple()
                        .alongWith(intake.stowFactory(), grabber.grabConeFactory().andThen(grabber.holdConeFactory()),
                            grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot()))
                )
            )
        );
    }

    public static SequentialCommandGroup center2Balance(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            centerBaseTo1Mid(alliance, drive, intake, arm, grabber, candle, source),
            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid1ToPiece2.wpilib.json"), new Rotation2d(), Rotation2d.fromDegrees(35), false)
                .deadlineWith(intake.intakeFactory(), arm.stowSimple(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelCommandGroup(
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

    public static SequentialCommandGroup center2(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return centerBaseTo1High(alliance, drive, intake, arm, grabber, candle, source);
    } 

    public static SequentialCommandGroup wall2(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return wallBaseTo9Mid(alliance, drive, intake, arm, grabber, candle, source);
    }

    private static SequentialCommandGroup centerBaseTo1High(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            initialScoreCubeHigh(drive, arm, grabber, source)
                .deadlineWith(intake.setConeFactory(), candle.wantConeFactory()),
            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid2ToPiece1.wpilib.json"), false)
                .deadlineWith(intake.intakeFactory(), arm.stowSimple(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelCommandGroup(
                new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece1ToGrid1.wpilib.json"), false),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.stowSimple(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    new Handoff(intake, arm, grabber, true, true),
                    arm.sendArmToStateAndEnd(ArmStates.scoreConeHigh)
                        .deadlineWith(intake.downFactory(), grabber.grabConeFactory().andThen(grabber.holdConeFactory()), 
                            grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot(), grabber.applyAim(alliance.equals("Blue") ? -60 : 0)))
                )
            ),
            arm.stowFrom(ArmStates.scoreConeHigh).alongWith(grabber.dropConeFactory()).withTimeout(0.75).andThen(grabber.applyAim(0))
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
                    arm.sendArmToState(ArmStates.autoPrepScoreCone).alongWith(intake.downFactory(), grabber.grabConeFactory(), grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot()))
                )
            ),
            arm.sendArmToStateAndEnd(ArmStates.scoreConeMiddle)
                        .deadlineWith(intake.downFactory(), grabber.grabConeFactory().andThen(grabber.holdConeFactory()), 
                            grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot(), grabber.applyAim(alliance.equals("Blue") ? -60 : 36))),
            arm.stowFrom(ArmStates.scoreConeMiddle).alongWith(grabber.dropConeFactory()).withTimeout(0.75).andThen(grabber.applyAim(0))
        );
    }

    private static SequentialCommandGroup wallBaseTo9Mid(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            initialScoreCubeMid(drive, arm, grabber, source)
                .deadlineWith(intake.setConeFactory(), candle.wantConeFactory()),
            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "WallGrid8ToPiece4.wpilib.json"), false)
                .deadlineWith(intake.intakeFactory(), arm.holdTargetState(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelCommandGroup(
                new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "WallPiece4ToGrid9.wpilib.json"), false),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.stowSimple(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    new Handoff(intake, arm, grabber, true, true),
                    arm.sendArmToStateAndEnd(ArmStates.scoreConeMiddle)
                        .deadlineWith(intake.downFactory(), grabber.grabConeFactory().andThen(grabber.holdConeFactory()), 
                            grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot()))
                )
            ),
            arm.stowFrom(ArmStates.scoreConeMiddle).alongWith(grabber.dropConeFactory()).withTimeout(0.75)
        );
    }

    private static SequentialCommandGroup charge1balance(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            initialScoreCubeMid(drive, arm, grabber, source)
                .deadlineWith(intake.setConeFactory(), candle.wantConeFactory()),
            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "ChargeGrid5ToPiece3.wpilib.json"), false)
                .deadlineWith(intake.intakeFactory(), arm.holdTargetState(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelCommandGroup(
                new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "ChargePiece3ToBalance.wpilib.json"), false),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.holdTargetState(), grabber.holdConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    new Handoff(intake, arm, grabber, true, true)
                )
            ),
            new AutoBalanceSimple(drive)
        );
    }

    private static SequentialCommandGroup centerBaseTo3(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(
            initialScoreCubeHigh(drive, arm, grabber, source)
                .deadlineWith(intake.setConeFactory(), candle.wantConeFactory()),
            new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterGrid2ToPiece1.wpilib.json"), false)
                .deadlineWith(intake.intakeFactory(), arm.stowSimple(), grabber.holdConeFactory(), 
                    grabber.setPivotForwardsFactory().andThen(grabber.forcePivot())),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece1ToGrid3-1.wpilib.json"), false),
                    new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterPiece1ToGrid3-2.wpilib.json"), false)
                ),
                new SequentialCommandGroup(
                    intake.stowFactory().alongWith(arm.stowSimple(), grabber.dropConeFactory()).until(intake::isArmUp).withTimeout(0.5),
                    new Handoff(intake, arm, grabber, true, false),
                    arm.stowSimple().alongWith(intake.stowFactory(), grabber.grabConeFactory()).withTimeout(1),
                    arm.sendArmToStateAndEnd(ArmStates.scoreConeHigh)
                        .deadlineWith(intake.downFactory(), grabber.grabConeFactory().andThen(grabber.holdConeFactory()), 
                            grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot()))
                )
            ),
            arm.stowFrom(ArmStates.scoreConeHigh).alongWith(grabber.dropConeFactory()).withTimeout(0.5)
        );
    }

    private static SequentialCommandGroup centerBalance(String alliance, SwerveDrive drive, Intake intake, Arm arm, Grabber grabber, Lights candle, BotPoseProvider source) {
        return new SequentialCommandGroup(new FollowHolonomicTrajectory(drive, TrajectoryHelper.loadJSONTrajectory(alliance + "CenterEngage.wpilib.json"), false)
                .deadlineWith(arm.stowSimple(), 
                    grabber.holdConeFactory(), intake.stowFactory()),
            arm.stowSimple().alongWith(grabber.holdConeFactory(), intake.stowFactory())
        );
    }

    private static SequentialCommandGroup initialScoreCubeHigh(SwerveDrive drive, Arm arm, Grabber grabber, BotPoseProvider source) {
        return new SequentialCommandGroup(
            new Localize(drive, source).deadlineWith(grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot())).withTimeout(0.25),
            arm.sendArmToStateAndEnd(ArmStates.scoreCubeHigh).deadlineWith(grabber.holdCubeFactory()),
            arm.stowFrom(ArmStates.scoreCubeHigh).alongWith(grabber.dropCubeFactory()).withTimeout(0.4)
        );
    }

    private static SequentialCommandGroup initialScoreCubeMid(SwerveDrive drive, Arm arm, Grabber grabber, BotPoseProvider source) {
        return new SequentialCommandGroup(
            new Localize(drive, source).deadlineWith(grabber.forcePivotBackwardsFactory().andThen(grabber.forcePivot())).withTimeout(0.25),
            arm.sendArmToStateAndEnd(ArmStates.scoreCubeMiddle).deadlineWith(grabber.holdCubeFactory()),
            arm.stowFrom(ArmStates.scoreCubeHigh).alongWith(grabber.dropCubeFactory()).withTimeout(0.25)
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
