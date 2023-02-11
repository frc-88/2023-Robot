// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    new DoublePreferenceConstant("Inner Roller Hold Cube Intake Speed", 0.1);
  private DoublePreferenceConstant outerRollerHoldCubeIntakeSpeed =
    new DoublePreferenceConstant("Outer Roller Hold Cube Intake Speed", 0.1);
  private DoublePreferenceConstant innerRollerHoldConeIntakeSpeed =
    new DoublePreferenceConstant("Inner Roller Hold Cone Intake Speed", -0.1);
  private DoublePreferenceConstant outerRollerHoldConeIntakeSpeed =
    new DoublePreferenceConstant("Outer Roller Hold Cone Intake Speed", 0.1);
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

  private final AnalogInput m_irSensor = new AnalogInput(Constants.INTAKE_IR_ID);

  private boolean coneMode = false;

  private final static double IR_HAS_PIECE_MIN = 1;
  private final static double IR_HAS_PIECE_MAX = 2;

  /** Creates a new Intake. */
  public Intake() {
    m_arm.setInverted(true);
    m_arm.overrideLimitSwitchesEnable(false);

    StatorCurrentLimitConfiguration sclc = new StatorCurrentLimitConfiguration(true, 20, 30, .1);

    m_innerRoller.configStatorCurrentLimit(sclc);
    m_outerRoller.configStatorCurrentLimit(sclc);
  }

    public void setCube() {
      coneMode = false;
    }

    public void setCone() {
      coneMode = true;
    }

    public void intake() {
      if (!hasGamePiece()) {
        if (!coneMode) {
          m_innerRoller.set(innerRollerCubeIntakeSpeed.getValue());
          m_outerRoller.set(outerRollerCubeIntakeSpeed.getValue());
        } else {
          m_innerRoller.set(innerRollerConeIntakeSpeed.getValue());
          m_outerRoller.set(outerRollerConeIntakeSpeed.getValue());
        }
      } else {
        hold();
      }
    }

    public void outgest() {
      m_innerRoller.set(innerRollerOutgestIntakeSpeed.getValue());
      m_outerRoller.set(outerRollerOutgestIntakeSpeed.getValue());
    }

    public void stopRollers() {
      m_innerRoller.set(0.);
      m_outerRoller.set(0.);
    }

    public void hold() {
      if (hasGamePiece()) {
        if (!coneMode) {
          m_innerRoller.set(innerRollerHoldCubeIntakeSpeed.getValue());
          m_outerRoller.set(outerRollerHoldCubeIntakeSpeed.getValue());
        } else {
          m_innerRoller.set(innerRollerHoldConeIntakeSpeed.getValue());
          m_outerRoller.set(outerRollerHoldConeIntakeSpeed.getValue());
        }
      } else {
        stopRollers();
      }
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

    private boolean hasGamePiece() {
      return m_irSensor.getAverageVoltage() > IR_HAS_PIECE_MIN && m_irSensor.getAverageVoltage() < IR_HAS_PIECE_MAX;
    }
    
    public Trigger holdAndHasPiece() {
      return new Trigger(() -> isArmUp() && hasGamePiece());
    }

    ////////// Commands :) /////////

    public CommandBase setCubeFactory() {
      return new InstantCommand(() -> {setCube();}).withName("set cube");
    }

    public CommandBase setConeFactory() {
      return new InstantCommand(() -> {setCone();}).withName("set cone");
    }
    
    public CommandBase intakeFactory() {
      return new RunCommand(() -> {intake(); armDown();}, this).withName("intake");
    }

    public CommandBase holdFactory() {
      return new RunCommand(() -> {hold(); armUp();}, this).withName("hold");
    }

    public CommandBase outgestFactory() {
      return new RunCommand(() -> {outgest(); armDown();}, this).withName("outgest"); 
    }

    public CommandBase stowFactory() {
      return new RunCommand(() -> {stopRollers(); armUp();}, this).withName("stow");
    }

    public CommandBase handoffFactory() {
      return new RunCommand (() -> {outgest(); armUp();}, this).withName("handoff");
    }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Cone Mode", coneMode);
    SmartDashboard.putBoolean("Cube Mode", !coneMode);
    SmartDashboard.putBoolean("Arm Up", isArmUp());
    SmartDashboard.putBoolean("Arm Down", isArmDown());
    SmartDashboard.putNumber("Arm Inner Motor Current", m_innerRoller.getStatorCurrent());
    SmartDashboard.putNumber("Arm Outer Motor Current", m_outerRoller.getStatorCurrent());

    SmartDashboard.putNumber("IR Sensor value", m_irSensor.getAverageVoltage());
    SmartDashboard.putBoolean("Has Game Piece", hasGamePiece());

  }
}
