// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  // Hold
  private DoublePreferenceConstant innerRollerHoldCubeIntakeSpeed =
    new DoublePreferenceConstant("Inner Roller Hold Intake Speed", 0.1);
  private DoublePreferenceConstant outerRollerHoldCubeIntakeSpeed =
    new DoublePreferenceConstant("Outer Roller Hold Intake Speed", 0.1);
  private DoublePreferenceConstant innerRollerHoldConeIntakeSpeed =
    new DoublePreferenceConstant("Inner Roller Hold Intake Speed", -0.1);
  private DoublePreferenceConstant outerRollerHoldConeIntakeSpeed =
    new DoublePreferenceConstant("Outer Roller Hold Intake Speed", 0.1);
  // Arm
  private DoublePreferenceConstant armUpStallIntakeSpeed =
      new DoublePreferenceConstant("Arm Up Stall Intake Speed", 0.5);
  private DoublePreferenceConstant armDownStallIntakeSpeed =
      new DoublePreferenceConstant("Arm Down Stall Intake Speed", 0.5);
  private DoublePreferenceConstant armUpMoveIntakeSpeed =
      new DoublePreferenceConstant("Arm Up Move Intake Speed", 0.5);  
  private DoublePreferenceConstant armDownMoveIntakeSpeed =
      new DoublePreferenceConstant("Arm Down Move Intake Speed", 0.5);  

  private final WPI_TalonFX m_innerRoller = new WPI_TalonFX(Constants.INTAKE_INNER_ROLLER_ID, Constants.INTAKE_CANBUS);
  private final WPI_TalonFX m_outerRoller = new WPI_TalonFX(Constants.INTAKE_OUTER_ROLLER_ID, Constants.INTAKE_CANBUS);
  private final WPI_TalonFX m_arm = new WPI_TalonFX(Constants.INTAKE_ARM_ID, Constants.INTAKE_CANBUS); 

  /** Creates a new Intake. */
  public Intake() {
    m_arm.setInverted(true);
    m_arm.overrideLimitSwitchesEnable(false);

    StatorCurrentLimitConfiguration sclc = new StatorCurrentLimitConfiguration(true, 20, 30, .1);

    m_innerRoller.configStatorCurrentLimit(sclc);
    m_outerRoller.configStatorCurrentLimit(sclc);
  }

    public void intakeCube() {
      m_innerRoller.set(innerRollerCubeIntakeSpeed.getValue());
      m_outerRoller.set(outerRollerCubeIntakeSpeed.getValue());
    }

    public void intakeCone() {
      m_innerRoller.set(innerRollerConeIntakeSpeed.getValue());
      m_outerRoller.set(outerRollerConeIntakeSpeed.getValue());
    }

    public void outgest() {
      m_innerRoller.set(innerRollerOutgestIntakeSpeed.getValue());
      m_outerRoller.set(outerRollerOutgestIntakeSpeed.getValue());
    }

    public void stopRollers() {
      m_innerRoller.set(0.);
      m_outerRoller.set(0.);
    }

    public void holdCube() {
      m_innerRoller.set(innerRollerHoldCubeIntakeSpeed.getValue());
      m_outerRoller.set(outerRollerHoldCubeIntakeSpeed.getValue());
    }

    public void holdCone() {
      m_innerRoller.set(innerRollerHoldConeIntakeSpeed.getValue());
      m_outerRoller.set(outerRollerHoldConeIntakeSpeed.getValue());
    }

    public void armUp() {
      if (isArmUp()) {
        m_arm.set(armUpStallIntakeSpeed.getValue());
      } else {
        m_arm.set(armUpMoveIntakeSpeed.getValue()); 
      }
     }
  
     public void armDown() {
      if (isArmDown()) {
        m_arm.set(armDownStallIntakeSpeed.getValue());
      } else {
       m_arm.set(armDownMoveIntakeSpeed.getValue());
     }
      
    }

    private boolean isArmUp() {
      return m_arm.isRevLimitSwitchClosed() > 0;
    }

    private boolean isArmDown() {
      return m_arm.isFwdLimitSwitchClosed() > 0;
    }
    
    ////////// Commands :) /////////

    public CommandBase intakeCubeFactory() {
      return new RunCommand(() -> {intakeCube(); armDown();}, this);
    }

    public CommandBase intakeConeFactory() {
      return new RunCommand(() -> {intakeCone(); armDown();}, this);
    }

    public CommandBase holdCubeFactory() {
      return new RunCommand(() -> {holdCube(); armUp();}, this);
    }

    public CommandBase holdConeFactory() {
      return new RunCommand(() -> {holdCone(); armUp();}, this);
    }

    public CommandBase outgestFactory() {
      return new RunCommand(() -> {outgest(); armDown();}, this); 
    }

    public CommandBase stowFactory() {
      return new RunCommand(() -> {stopRollers(); armUp();}, this);
    }

    public CommandBase handoffFactory() {
      return new RunCommand (() -> {outgest(); armUp();}, this);
    }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Arm Up", isArmUp());
    SmartDashboard.putBoolean("Arm Down", isArmDown());
    SmartDashboard.putNumber("Arm Inner Motor Current", m_innerRoller.getStatorCurrent());
    SmartDashboard.putNumber("Arm Outer Motor Current", m_outerRoller.getStatorCurrent());
  }
}
