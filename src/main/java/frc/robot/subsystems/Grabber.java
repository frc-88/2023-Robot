// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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

  private final WPI_TalonSRX m_pivot = new WPI_TalonSRX(Constants.GRABBER_PIVOT_ID);
  private final WPI_TalonSRX m_roller = new WPI_TalonSRX(Constants.GRABBER_ROLLER_ID);

  private boolean m_pivotForwards = true;
  private boolean m_pivotLocked = false;
  private double m_aimAngle = 0;
  private double m_lastPivotPosition = 0;

  private DoublePreferenceConstant p_pivotOffset = 
    new DoublePreferenceConstant("Grabber/Pivot/Offset", 0);

  private final DoublePreferenceConstant p_pivotMaxVelocity = 
    new DoublePreferenceConstant("Grabber/Pivot/Max Velocity", 0);
  private final DoublePreferenceConstant p_pivotMaxAcceleration = 
    new DoublePreferenceConstant("Grabber/Pivot/Max Acceleration", 0);
  private final PIDPreferenceConstants p_pivotPID = 
    new PIDPreferenceConstants("Grabber/Pivot/");

  private final DoublePreferenceConstant p_rollerTriggerCurrent =
    new DoublePreferenceConstant("Grabber/Roller/Trigger Current", 60);
  private final DoublePreferenceConstant p_rollerTriggerDuration =
    new DoublePreferenceConstant("Grabber/Roller/Trigger Duration", 0.002);
  private final DoublePreferenceConstant p_rollerContinuousCurrent =
    new DoublePreferenceConstant("Grabber/Roller/Continuous Current", 10);

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

  public Grabber(BooleanSupplier coastMode, BooleanSupplier armStowed) {
    m_coastMode = coastMode;
    m_armStowed = armStowed;

    m_pivot.configFactoryDefault();
    m_roller.configFactoryDefault();

    m_pivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    m_pivot.setInverted(true);
    m_pivot.setSensorPhase(true);
    m_pivot.configNeutralDeadband(0);
    m_pivot.setNeutralMode(NeutralMode.Brake);
    m_pivot.configMotionSCurveStrength(2);
    m_pivot.configClosedloopRamp(0.1);

    Consumer<Double> pivotHandler = (Double unused) -> {
      m_pivot.config_kP(0, p_pivotPID.getKP().getValue());
      m_pivot.config_kI(0, p_pivotPID.getKI().getValue());
      m_pivot.config_kD(0, p_pivotPID.getKD().getValue());
      m_pivot.config_kF(0, p_pivotPID.getKF().getValue());
      m_pivot.config_IntegralZone(0, p_pivotPID.getIZone().getValue());
      m_pivot.configMaxIntegralAccumulator(0, p_pivotPID.getIMax().getValue());
      m_pivot.configMotionCruiseVelocity(convertActualVelocityToSensorVelocity(p_pivotMaxVelocity.getValue()));
      m_pivot.configMotionAcceleration(convertActualVelocityToSensorVelocity(p_pivotMaxAcceleration.getValue()));
    };
    p_pivotPID.addChangeHandler(pivotHandler);
    p_pivotMaxVelocity.addChangeHandler(pivotHandler);
    p_pivotMaxAcceleration.addChangeHandler(pivotHandler);

    zeroRelativePivot();

    m_roller.configNeutralDeadband(0);
    m_roller.overrideLimitSwitchesEnable(false);

    Consumer<Double> rollerHandler = (Double unused) -> {
      SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration(
                true,
                p_rollerContinuousCurrent.getValue(),
                p_rollerTriggerCurrent.getValue(),
                p_rollerTriggerDuration.getValue()
            );
      m_roller.configSupplyCurrentLimit(config);
    };
    p_rollerContinuousCurrent.addChangeHandler(rollerHandler);
    p_rollerTriggerCurrent.addChangeHandler(rollerHandler);
    p_rollerTriggerDuration.addChangeHandler(rollerHandler);
    
    pivotHandler.accept(0.);
    rollerHandler.accept(0.);
  }

  public boolean isReady() {
    return m_pivot.getBusVoltage() > 6 && m_roller.getBusVoltage() > 6;
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
    m_pivot.set(ControlMode.Position, convertActualPositionToSensorPosition(m_lastPivotPosition  - m_aimAngle));
  }

  private void lockPivot() {
    m_pivotLocked = true;
  }

  private void unlockPivot() {
    m_pivotLocked = false;
  }

  public double getPivotAngle() {
    return convertSensorPositionToActualPosition(m_pivot.getSelectedSensorPosition());
  }

  public double getPivotSpeed() {
    return convertSensorVelocityToActualVelocity(m_pivot.getSelectedSensorVelocity());
  }

  public double getPivotAbsoluteAngle() {
    return (m_pivot.getSensorCollection().getPulseWidthPosition() * 360. / 4096. - p_pivotOffset.getValue() + 270. + 360.*5.) % 360. - 270.;
  }

  public void zeroRelativePivot() {
    m_pivot.setSelectedSensorPosition(convertActualPositionToSensorPosition(getPivotAbsoluteAngle()));
  }

  public void calibrateAbsolutePivot() {
    p_pivotOffset.setValue(0.);
    p_pivotOffset.setValue(getPivotAbsoluteAngle());
    zeroRelativePivot();
  }

  public void grabCube() {
    if (!m_hasGamePieceDebounce.calculate(hasGamePiece())) m_roller.set(p_grabCubeSpeed.getValue());
    else holdCube();
  }

  public void grabCone() {
    if (!m_hasGamePieceDebounce.calculate(hasGamePiece())) m_roller.set(p_grabConeSpeed.getValue());
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
    return m_roller.isFwdLimitSwitchClosed() > 0;
  }

  public Trigger hasGamePieceTrigger() {
    return new Trigger(this::hasGamePiece);
  }

  private double convertSensorPositionToActualPosition(double motorPosition) {
    return motorPosition * 360. / 4096.;
  }

  private double convertSensorVelocityToActualVelocity(double motorVelocity) {
      return convertSensorPositionToActualPosition(motorVelocity) * 10.;
  }

  private double convertActualPositionToSensorPosition(double actualPosition) {
      return actualPosition * 4096. / 360.;
  }

  private double convertActualVelocityToSensorVelocity(double actualVelocity) {
      return convertActualPositionToSensorPosition(actualVelocity) * 0.1;
  }

  public void aim(double angle) {
    m_aimAngle = angle;
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
    return new RunCommand(() -> {centerCone(); movePivot(); unlockPivot();}, this).withTimeout(2);
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
    if (m_pivot.hasResetOccurred() || getPivotSpeed() < 5 && Math.abs(getPivotAbsoluteAngle() - getPivotAngle()) > 3) {
      zeroRelativePivot();
    }

    if (m_coastMode.getAsBoolean()) {
      m_pivot.setNeutralMode(NeutralMode.Coast);
    } else {
      m_pivot.setNeutralMode(NeutralMode.Brake);
    }

    SmartDashboard.putNumber("Grabber Pivot Angle", getPivotAngle());
    SmartDashboard.putNumber("Grabber Pivot Absolute Angle", getPivotAbsoluteAngle());
    SmartDashboard.putBoolean("Grabber Has Game Piece", hasGamePiece());
    SmartDashboard.putNumber("Grabber Aim", m_aimAngle);
  }
}
