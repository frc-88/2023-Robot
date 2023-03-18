// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

/*
 * don't tell anyone
 * but my favorite game piece
 * is the one you dropped
 */

public class Intake extends SubsystemBase {
  
  // Cube
  private DoublePreferenceConstant innerRollerCubeIntakeSpeed = 
      new DoublePreferenceConstant("Intake/Speeds/Intake/Inner Cube", -0.5);
  private DoublePreferenceConstant outerRollerCubeIntakeSpeed =
      new DoublePreferenceConstant("Intake/Speeds/Intake/Outer Cube", 0.5);
  // Cone
  private DoublePreferenceConstant innerRollerConeIntakeSpeed =
      new DoublePreferenceConstant("Intake/Speeds/Intake/Inner Cone", 0.5);
  private DoublePreferenceConstant outerRollerConeIntakeSpeed =
      new DoublePreferenceConstant("Intake/Speeds/Intake/Outer Cone", 0.5);
  // Outgest
  private DoublePreferenceConstant innerRollerOutgestConeIntakeSpeed =
      new DoublePreferenceConstant("Intake/Speeds/Outgest/Inner Cone", -1);
  private DoublePreferenceConstant outerRollerOutgestConeIntakeSpeed =
      new DoublePreferenceConstant("Intake/Speeds/Outgest/Outer Cone", -0.1);
      private DoublePreferenceConstant innerRollerOutgestCubeIntakeSpeed =
      new DoublePreferenceConstant("Intake/Speeds/Outgest/Inner Cube", -1);
  private DoublePreferenceConstant outerRollerOutgestCubeIntakeSpeed =
      new DoublePreferenceConstant("Intake/Speeds/Outgest/Outer Cube", -1);
  // Hold
  private DoublePreferenceConstant innerRollerHoldCubeIntakeSpeed =
    new DoublePreferenceConstant("Intake/Speeds/Hold/Inner Cube", 0.025);
  private DoublePreferenceConstant outerRollerHoldCubeIntakeSpeed =
    new DoublePreferenceConstant("Intake/Speeds/Hold/Outer Cube", 0.1);
  private DoublePreferenceConstant innerRollerHoldConeIntakeSpeed =
    new DoublePreferenceConstant("Intake/Speeds/Hold/Inner Cone", 0.07);
  private DoublePreferenceConstant outerRollerHoldConeIntakeSpeed =
    new DoublePreferenceConstant("Intake/Speeds/Hold/Outer Cone", 0.1);
  // Arm
  private DoublePreferenceConstant armUpStallIntakeSpeed =
      new DoublePreferenceConstant("Intake/Arm/Up Stall Speed", -0.03);
  private DoublePreferenceConstant armDownStallIntakeSpeed =
      new DoublePreferenceConstant("Intake/Arm/Down Stall Speed", 0.03);
  private DoublePreferenceConstant armUpMoveIntakeSpeed =
      new DoublePreferenceConstant("Intake/Arm/Up Move Speed", -0.35);  
  private DoublePreferenceConstant armDownMoveIntakeSpeed =
      new DoublePreferenceConstant("Intake/Arm/Down Move Speed", 0.35); 
  // IR Sensor
  private DoublePreferenceConstant irSensorConeMin =
      new DoublePreferenceConstant("Intake/IR/Cone Min", 0.491);
  private DoublePreferenceConstant irSensorConeMax =
      new DoublePreferenceConstant("Intake/IR/Cone Max", 0.698);
  private DoublePreferenceConstant irSensorCubeMin =
      new DoublePreferenceConstant("Intake/IR/Cube Min", 0.491);
  private DoublePreferenceConstant irSensorCubeMax =
      new DoublePreferenceConstant("Intake/IR/Cube Max", 0.698);
  private DoublePreferenceConstant irSensorHasPiece =
      new DoublePreferenceConstant("Intake/IR/Has Piece", 0.25);

  private final WPI_TalonFX m_innerRoller = new WPI_TalonFX(Constants.INTAKE_INNER_ROLLER_ID, Constants.INTAKE_CANBUS);
  private final WPI_TalonFX m_outerRoller = new WPI_TalonFX(Constants.INTAKE_OUTER_ROLLER_ID, Constants.INTAKE_CANBUS);
  private final WPI_TalonFX m_arm = new WPI_TalonFX(Constants.INTAKE_ARM_ID, Constants.INTAKE_CANBUS); 

  private final AnalogInput m_irSensor = new AnalogInput(Constants.INTAKE_IR_ID);

  private final LinearFilter m_irFilter = LinearFilter.singlePoleIIR(0.25, 0.02);
  private final Debouncer m_hasGamePieceDebounce = new Debouncer(0.25, DebounceType.kBoth);
  private final Debouncer m_hasGamePieceCenteredDebounce = new Debouncer(0.25, DebounceType.kBoth);

  private double m_irValue = 0;

  private boolean m_coneMode = false;
  private boolean m_isIntaking = false;

  /** Creates a new Intake. */
  public Intake() {
    m_arm.setInverted(true);
    m_arm.overrideLimitSwitchesEnable(false);

    StatorCurrentLimitConfiguration sclc = new StatorCurrentLimitConfiguration(true, 60, 60, .1);

    m_innerRoller.configStatorCurrentLimit(sclc);
    m_outerRoller.configStatorCurrentLimit(sclc);

    m_irSensor.setAverageBits(8);
  }
    public void addToOrchestra(Orchestra m_orchestra) {
      m_orchestra.addInstrument(m_arm);
      m_orchestra.addInstrument(m_innerRoller);
      m_orchestra.addInstrument(m_outerRoller);
    }

    public boolean isReady() {
      return m_arm.getBusVoltage() > 6 && m_outerRoller.getBusVoltage() > 6 && m_innerRoller.getBusVoltage() > 6 && isArmUp();
    }

    public void setCube() {
      m_coneMode = false;
    }

    public void setCone() {
      m_coneMode = true;
    }

    public void intake() {
      if (!hasGamePieceCentered() || !m_coneMode) {
        if (!m_coneMode) {
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
      if (!m_coneMode) {
        m_innerRoller.set(innerRollerOutgestCubeIntakeSpeed.getValue());
        m_outerRoller.set(outerRollerOutgestCubeIntakeSpeed.getValue());
      } else {
        m_innerRoller.set(innerRollerOutgestConeIntakeSpeed.getValue());
        m_outerRoller.set(outerRollerOutgestConeIntakeSpeed.getValue());
      }
    }

    public void stopRollers() {
      m_innerRoller.set(0.);
      m_outerRoller.set(0.);
    }

    public void hold() {
      if (hasGamePiece()) {
        if (!m_coneMode) {
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

    public boolean isArmUp() {
      return m_arm.isRevLimitSwitchClosed() > 0;
    }

    public boolean isArmDown() {
      return m_arm.isFwdLimitSwitchClosed() > 0;
    }

    private boolean hasGamePieceCentered() {
      return m_hasGamePieceCenteredDebounce.calculate(
        m_irValue > (m_coneMode ? irSensorConeMin.getValue() : irSensorCubeMin.getValue()) 
        && m_irValue < (m_coneMode ? irSensorConeMax.getValue() : irSensorCubeMax.getValue()));
    }

    public boolean hasGamePiece() {
      return m_hasGamePieceDebounce.calculate(m_irValue > irSensorHasPiece.getValue());
    }
    
    public Trigger holdAndHasPiece() {
      return new Trigger(() -> isArmUp() && hasGamePieceCentered());
    }

    public boolean isIntaking() {
      return m_isIntaking;
    }

    ////////// Commands :) /////////

    public CommandBase setCubeFactory() {
      return new InstantCommand(() -> {setCube();}).withName("set cube");
    }

    public CommandBase setConeFactory() {
      return new InstantCommand(() -> {setCone();}).withName("set cone");
    }
    
    public CommandBase intakeFactory() {
      CommandBase coneCommand = new RunCommand(() -> {intake(); armDown();}, this)
        .until(this::hasGamePiece)
        .andThen(new RunCommand(() -> {intake(); armDown();}, this)
          .withTimeout(0.4)
          .andThen(new RunCommand(() -> {hold(); armUp();}, this)
            .until(this::hasGamePieceCentered)));
      CommandBase cubeCommand = new RunCommand(() -> {intake(); armDown();}, this)
        .until(this::hasGamePieceCentered)
        .andThen(new RunCommand(() -> {intake(); armDown();}, this)
          .withTimeout(0.5));
      return new ConditionalCommand(coneCommand, cubeCommand, () -> m_coneMode).alongWith(new InstantCommand(() -> m_isIntaking = true)).andThen(new InstantCommand(() -> m_isIntaking = false))
          .withName("intake");
    }

    public CommandBase outgestFactory() {
      return new RunCommand(() -> {outgest(); armDown();}, this).withName("outgest").alongWith(new InstantCommand(() -> m_isIntaking = false)); 
    }

    public CommandBase stowFactory() {
      return new RunCommand(() -> {hold(); armUp();}, this).withName("stow").alongWith(new InstantCommand(() -> m_isIntaking = false));
    }

    public CommandBase handoffFactory() {
      return new RunCommand (() -> {outgest(); armUp();}, this).withName("handoff").alongWith(new InstantCommand(() -> m_isIntaking = false));
    }

    public CommandBase downFactory() {
      return new RunCommand(() -> {hold(); armDown();}, this).withName("down").alongWith(new InstantCommand(() -> m_isIntaking = false));
    }

  @Override
  public void periodic() {
    m_irValue = m_irFilter.calculate(m_irSensor.getAverageVoltage());

    SmartDashboard.putBoolean("Cone Mode", m_coneMode);
    SmartDashboard.putBoolean("Cube Mode", !m_coneMode);
    SmartDashboard.putBoolean("Arm Up", isArmUp());
    SmartDashboard.putBoolean("Arm Down", isArmDown());
    SmartDashboard.putNumber("Arm Inner Motor Current", m_innerRoller.getStatorCurrent());
    SmartDashboard.putNumber("Arm Outer Motor Current", m_outerRoller.getStatorCurrent());

    SmartDashboard.putNumber("IR Sensor value", m_irValue);
    SmartDashboard.putBoolean("Has Game Piece Centered", hasGamePieceCentered());
    SmartDashboard.putBoolean("Has Game Piece", hasGamePiece());


  }
}
