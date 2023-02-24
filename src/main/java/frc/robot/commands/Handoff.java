// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.util.arm.ArmState;
import frc.robot.util.arm.ArmStates;

public class Handoff extends SequentialCommandGroup {

  public Handoff(Intake intake, Arm arm, Grabber grabber, BooleanSupplier coneMode) {
    Supplier<ArmState> armDown = () -> coneMode.getAsBoolean() ? ArmStates.getConeFromIntake : ArmStates.getCubeFromIntake;
    Supplier<CommandBase> grabCommand = () -> coneMode.getAsBoolean() ? grabber.grabConeFactory() : grabber.grabCubeFactory();
    addCommands(
      arm.sendArmToStateAndEnd(armDown).deadlineWith(grabCommand.get().alongWith(intake.stowFactory())),
      arm.sendArmToState(armDown).alongWith(grabCommand.get()).alongWith(intake.handoffFactory()).withTimeout(0.75),
      arm.sendArmToStateAndEnd(ArmStates.stow).deadlineWith(grabCommand.get().alongWith(intake.handoffFactory()))
    );
  }
}
