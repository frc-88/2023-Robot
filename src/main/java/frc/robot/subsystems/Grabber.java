// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

/*
 * don't worry it's cool
 * that cube is only mildy
 * underinflated
 */

public class Grabber extends SubsystemBase {

  private final BooleanSupplier m_coastMode;
  private final BooleanSupplier m_armStowed;

  private final DigitalInput m_gamePieceSense;

  //private final WPI_TalonSRX m_pivot = new WPI_TalonSRX(Constants.GRABBER_PIVOT_ID);
  //private final WPI_TalonSRX m_roller = new WPI_TalonSRX(Constants.GRABBER_ROLLER_ID);

  private final PWMSparkMax m_pivot = new PWMSparkMax(Constants.GRABBER_PIVOT_ID);
  private final PWMSparkMax m_roller = new PWMSparkMax(Constants.GRABBER_ROLLER_ID);
  private final CANCoder m_pivotCoder = new CANCoder(Constants.PIVOT_CANCODER_ID, "1");

 private final PIDController m_pivotPID;

  private boolean m_pivotForwards = true;
  private boolean m_pivotLocked = false;
  private double m_aimAngle = 0;
  private double m_lastPivotPosition = 0;

  private int loopCount = 0;

  private DoublePreferenceConstant p_pivotOffset = 
    new DoublePreferenceConstant("Grabber/Pivot/Offset", 0);

  private final PIDPreferenceConstants p_pivotPID = 
    new PIDPreferenceConstants("Grabber/Pivot/");

  private DoublePreferenceConstant p_grabCubeSpeed =
    new DoublePreferenceConstant("Grabber/Roller/Grab Cube Speed", 0.5);
  private DoublePreferenceConstant p_grabConeSpeed =
    new DoublePreferenceConstant("Grabber/Roller/Grab Cone Speed", -0.5);
  private DoublePreferenceConstant p_dropCubeSpeed =
    new DoublePreferenceConstant("Grabber/Roller/Drop Cube Speed", -0.5);
  private DoublePreferenceConstant p_dropConeSpeed =
    new DoublePreferenceConstant("Grabber/Roller/Drop Cone Speed", 0.5);
  private DoublePreferenceConstant p_holdCubeSpeed =
    new DoublePreferenceConstant("Grabber/Roller/Hold Cube Speed", -0.05);
  private DoublePreferenceConstant p_holdConeSpeed =
    new DoublePreferenceConstant("Grabber/Roller/Hold Cone Speed", 0.05);
    
  private DoublePreferenceConstant p_hasGamePieceDebounceTime =
    new DoublePreferenceConstant("Grabber/Roller/Debounce", 0.5);

  private Debouncer m_hasGamePieceDebounce = new Debouncer(p_hasGamePieceDebounceTime.getValue(), DebounceType.kRising);
  private Debouncer m_extraDebounce = new Debouncer(1, DebounceType.kRising);

  public Grabber(BooleanSupplier coastMode, BooleanSupplier armStowed) {
    this.m_gamePieceSense = new DigitalInput(3);

    m_roller.setInverted(false);
    
    m_coastMode = coastMode;
    m_armStowed = armStowed;

    m_pivotPID = new PIDController(p_pivotPID.getKP().getValue(), p_pivotPID.getKI().getValue(), p_pivotPID.getKD().getValue());
    p_pivotPID.getKP().addChangeHandler(m_pivotPID::setP);
    p_pivotPID.getKI().addChangeHandler(m_pivotPID::setI);
    p_pivotPID.getKD().addChangeHandler(m_pivotPID::setD);

    zeroPivotAngle();
  }

  public boolean isReady() {
    return true;
  }

  public void setPivotForwards() {
    m_pivotForwards = true;
  }

  public void setPivotBackwards() {
    m_pivotForwards = false;
  }

  public boolean isAtZero() {
    return Math.abs(getPivotAngle()) < 5;
  }

  private void movePivot() {
    if (!m_pivotLocked && m_armStowed.getAsBoolean()) {
      m_lastPivotPosition = m_pivotForwards ? 0 : -180;
    }
    m_pivot.set(m_pivotPID.calculate(getPivotAngle(), m_lastPivotPosition - (m_lastPivotPosition < -90 ? m_aimAngle : 0)));
  }

  private void lockPivot() {
    m_pivotLocked = true;
  }

  private void unlockPivot() {
    m_pivotLocked = false;
  }

  public double getPivotAngle() {
    return m_pivotCoder.getPosition();
  }

  public double getPivotSpeed() {
    return m_pivotCoder.getVelocity();
  }

  public double getPivotAbsoluteAngle() {
    return m_pivotCoder.getAbsolutePosition() - p_pivotOffset.getValue();
  }

  public void zeroPivotAngle() {
    m_pivotCoder.setPosition(((getPivotAbsoluteAngle() + 630) % 360) - 270);
  }

  public void calibrateAbsolutePivot() {
    p_pivotOffset.setValue(0.);
    p_pivotOffset.setValue(getPivotAbsoluteAngle());
    zeroPivotAngle();
  }

  public void grabCube() {
    if (!m_extraDebounce.calculate(hasGamePiece())) m_roller.set(p_grabCubeSpeed.getValue());
    else holdCube();
  }

  public void grabCone() {
    if (!m_extraDebounce.calculate(hasGamePiece())) m_roller.set(p_grabConeSpeed.getValue());
    else holdCone();
  }

  public void centerCone() {
    m_roller.set(p_grabConeSpeed.getValue());
  }

  public void dropCube() {
    m_roller.set(p_dropCubeSpeed.getValue());
  }
  
  public void dropCone() {
    m_roller.set(p_dropConeSpeed.getValue());
  }

  public void holdCube() {
    m_roller.set(hasGamePiece() ? p_holdCubeSpeed.getValue() : 0);
  }
  
  public void holdCone() {
    m_roller.set(hasGamePiece() ? p_holdConeSpeed.getValue() : 0);
  }

  public boolean hasGamePiece() {
    return !m_hasGamePieceDebounce.calculate(m_gamePieceSense.get());
  }

  public Trigger hasGamePieceTrigger() {
    return new Trigger(this::hasGamePiece);
  }

  public void aim(double angle) {
    if (angle > 90 || angle < -90) {
      angle = 0;
    }
    m_aimAngle = angle;
  }

  public double getAim() {
    return m_aimAngle;
  }

  // COMMAND FACTORIES

  public CommandBase calibrateAbsolutePivotFactory() {
    return new InstantCommand(this::calibrateAbsolutePivot).ignoringDisable(true);
  }

  public CommandBase grabConeFactory() {
    return new RunCommand(() -> {grabCone(); movePivot(); unlockPivot();}, this).withName("Grab Cone");
  }

  public CommandBase grabCubeFactory() {
    return new RunCommand(() -> {grabCube(); movePivot(); unlockPivot();}, this).withName("Grab Cube");
  }

  public CommandBase dropConeFactory() {
    return new RunCommand(() -> {dropCone(); movePivot(); unlockPivot();}, this).withName("Drop Cone");
  }

  public CommandBase dropCubeFactory() {
    return new RunCommand(() -> {dropCube(); movePivot(); unlockPivot();}, this).withName("Drop Cube");
  }

  public CommandBase holdConeFactory() {
    return new RunCommand(() -> {holdCone(); movePivot(); unlockPivot();}, this).withName("Hold Cone");
  }

  public CommandBase holdCubeFactory() {
    return new RunCommand(() -> {holdCube(); movePivot(); unlockPivot();}, this).withName("Hold Cube");
  }

  public CommandBase centerConeFactory() {
    return new RunCommand(() -> {centerCone(); movePivot(); unlockPivot();}, this).withTimeout(3);
  }

  public CommandBase holdFactory(BooleanSupplier coneMode) {
    return new RunCommand(() -> {
      if (coneMode.getAsBoolean()) {
        holdCone();
      } else {
        holdCube();
      }
      movePivot();
      unlockPivot();
    }, this);
  }

  public CommandBase setPivotForwardsFactory() {
    return new InstantCommand(this::setPivotForwards);
  }

  public CommandBase setPivotBackwardsFactory() {
    return new InstantCommand(() -> {
      if (hasGamePiece()) {
        setPivotBackwards();
      } else {
        setPivotForwards();
      }
    });
  }

  public CommandBase forcePivotBackwardsFactory() {
    return new InstantCommand(this::setPivotBackwards);
  }

  public CommandBase forcePivot() {
    return new InstantCommand(() -> m_lastPivotPosition = m_pivotForwards ? 0 : -180);
  }

  public CommandBase applyAim(double aim) {
    return new InstantCommand(() -> aim(aim));
  }

  @Override
  public void periodic() {
    if (m_pivotCoder.hasResetOccurred() || (DriverStation.isDisabled() && loopCount < 3_000 && (loopCount % 50 == 0)) || (getPivotSpeed() < 5 && (Math.abs(getPivotAbsoluteAngle() - getPivotAngle()) % 360) > 2)){
      zeroPivotAngle();
    }
    loopCount++;

    SmartDashboard.putNumber("Grabber Pivot Angle", getPivotAngle());
    SmartDashboard.putBoolean("Grabber Has Game Piece", hasGamePiece());
    SmartDashboard.putNumber("Grabber Aim", m_aimAngle);
  }

  public boolean isAtTarget() {
    return Math.abs(m_pivotPID.getPositionError()) < 5;
  }

}
