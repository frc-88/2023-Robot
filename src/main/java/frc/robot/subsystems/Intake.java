// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Intake extends SubsystemBase {
  
  // Cube
  private DoublePreferenceConstant innerRollerCubeIntakeSpeed = 
      new DoublePreferenceConstant("Inner Roller Cube Intake Speed", 0.5);
  private DoublePreferenceConstant outerRollerCubeIntakeSpeed =
      new DoublePreferenceConstant("Outer Roller Cube Intake Speed", 0.5);
  // Cone
  private DoublePreferenceConstant innerRollerConeIntakeSpeed =
      new DoublePreferenceConstant("Inner Roller Cone Intake Speed", -0.5);
  private DoublePreferenceConstant outerRollerConeIntakeSpeed =
      new DoublePreferenceConstant("Outer Roller Cone Intake Speed", 0.5);
  // Outgest
  private DoublePreferenceConstant innerRollerOutgestIntakeSpeed =
      new DoublePreferenceConstant("Inner Roller Outgest Intake Speed", -0.5);
  private DoublePreferenceConstant outerRollerOutgestIntakeSpeed =
      new DoublePreferenceConstant("Outer Roller Outgest Intake Speed", 0.5);
  // Arm
  private DoublePreferenceConstant armUpStallIntakeSpeed =
      new DoublePreferenceConstant("Arm Up Stall Intake Speed", 0.5);
  private DoublePreferenceConstant armDownStallIntakeSpeed =
      new DoublePreferenceConstant("Arm Down Stall Intake Speed", 0.5);
  private DoublePreferenceConstant armUpMoveIntakeSpeed =
      new DoublePreferenceConstant("Arm Up Move Intake Speed", 0.5);  
  private DoublePreferenceConstant armDownMoveIntakeSpeed =
      new DoublePreferenceConstant("Arm Down Move Intake Speed", 0.5);  

  private final WPI_TalonFX m_innerRoller = new WPI_TalonFX(Constants.INTAKE_INNER_ROLLER_ID);
  private final WPI_TalonFX m_outerRoller = new WPI_TalonFX(Constants.INTAKE_OUTER_ROLLER_ID);
  private final WPI_TalonFX m_arm = new WPI_TalonFX(Constants.INTAKE_ARM_ID);

  /** Creates a new Intake. */
  public Intake() {}

    public void intakeCube () {
      m_innerRoller.set(innerRollerCubeIntakeSpeed.getValue());
      m_outerRoller.set(outerRollerCubeIntakeSpeed.getValue());
    }

    public void intakeCone () {
      m_innerRoller.set(innerRollerConeIntakeSpeed.getValue());
      m_outerRoller.set(outerRollerConeIntakeSpeed.getValue());
    }
    public void Outgest () {
      m_innerRoller.set(innerRollerOutgestIntakeSpeed.getValue());
      m_outerRoller.set(outerRollerOutgestIntakeSpeed.getValue());
    }

    public void stopRollers () {
      m_innerRoller.set(0);
      m_outerRoller.set(0);
    }

    public void armUp () {
      if (m_arm.isFwdLimitSwitchClosed() > 0) {
        m_arm.set(armUpStallIntakeSpeed.getValue());
      }
      else {
       m_arm.set(armUpMoveIntakeSpeed.getValue());
      }
     }
  
     public void armDown () {
      if (m_arm.isRevLimitSwitchClosed() > 0) {
        m_arm.set(armDownStallIntakeSpeed.getValue());
      }
      else {
       m_arm.set(armDownMoveIntakeSpeed.getValue());
     }
      
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
