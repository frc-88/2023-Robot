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

  public Handoff(Intake intake, Arm arm, Grabber grabber, boolean coneMode) {
    Supplier<CommandBase> grab = () -> coneMode ? grabber.grabConeFactory() :grabber.grabCubeFactory();
    addCommands(
      arm.sendArmToStateAndEnd(coneMode ? ArmStates.getConeFromIntake : ArmStates.getCubeFromIntake).deadlineWith(grab.get(), intake.stowFactory()),
      arm.holdTargetState().alongWith(grab.get(), intake.handoffFactory()).withTimeout(0.5),
      arm.sendArmToStateAndEnd(ArmStates.stow).deadlineWith(grab.get(), intake.handoffFactory())
    );
  }
}
