// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.util.arm.ArmStates;

public class Handoff extends SequentialCommandGroup {

  public Handoff(Intake intake, Arm arm, Grabber grabber, boolean coneMode, boolean runShort) {
    Supplier<CommandBase> grab = () -> coneMode ? grabber.grabConeFactory() :grabber.grabCubeFactory();
    addCommands(
      arm.sendArmToStateAndEnd(coneMode ? ArmStates.getConeFromIntake1 : ArmStates.getCubeFromIntake1).deadlineWith(grab.get(), intake.stowFactory()),
      arm.sendArmToStateAndEnd(coneMode ? ArmStates.getConeFromIntake2 : ArmStates.getCubeFromIntake2).deadlineWith(grab.get(), intake.handoffFactory()).withTimeout(coneMode ? 0.3 : 1),
      arm.sendArmToState(ArmStates.stowWithPiece).alongWith(grab.get(), intake.handoffFactory()).until(() -> arm.isAtTarget(runShort ? 15 : 0.5))
    );
  }
}
