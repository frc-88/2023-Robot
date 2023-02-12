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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Grabber extends SubsystemBase {

  private final WPI_TalonSRX m_pivot = new WPI_TalonSRX(Constants.GRABBER_PIVOT_ID);
  private final WPI_TalonSRX m_roller = new WPI_TalonSRX(Constants.GRABBER_ROLLER_ID);

  private boolean m_pivotForwards = true;
  private boolean m_pivotLocked = true;

  private DoublePreferenceConstant p_pivotOffset = 
    new DoublePreferenceConstant("Grabber Pivot Offset", 0);

  private final DoublePreferenceConstant p_pivotMaxVelocity = 
    new DoublePreferenceConstant("Grabber Pivot Max Velocity", 0);
  private final DoublePreferenceConstant p_pivotMaxAcceleration = 
    new DoublePreferenceConstant("Grabber Pivot Max Acceleration", 0);
  private final PIDPreferenceConstants p_pivotPID = 
    new PIDPreferenceConstants("Grabber Pivot");

  private final DoublePreferenceConstant p_rollerTriggerCurrent =
    new DoublePreferenceConstant("Grabber Roller Trigger Current", 60);
  private final DoublePreferenceConstant p_rollerTriggerDuration =
    new DoublePreferenceConstant("Grabber Roller Trigger Duration", 0.002);
  private final DoublePreferenceConstant p_rollerContinuousCurrent =
    new DoublePreferenceConstant("Grabber Continuous Current", 10);

  private DoublePreferenceConstant p_grabCubeSpeed =
    new DoublePreferenceConstant("Grab Cube Speed", 0.5);
  private DoublePreferenceConstant p_grabConeSpeed =
    new DoublePreferenceConstant("Grab Cone Speed", -0.5);
  private DoublePreferenceConstant p_dropCubeSpeed =
    new DoublePreferenceConstant("Drop Cube Speed", -0.5);
  private DoublePreferenceConstant p_dropConeSpeed =
    new DoublePreferenceConstant("Drop Cone Speed", 0.5);
  private DoublePreferenceConstant p_holdCubeSpeed =
    new DoublePreferenceConstant("Hold Cube Speed", -0.05);
  private DoublePreferenceConstant p_holdConeSpeed =
    new DoublePreferenceConstant("Hold Cone Speed", 0.05);

  public Grabber() {
    m_pivot.configFactoryDefault();
    m_roller.configFactoryDefault();

    m_pivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    zeroRelativePivot();

    m_pivot.configNeutralDeadband(0);
    m_pivot.setNeutralMode(NeutralMode.Brake);
    m_pivot.configMotionSCurveStrength(4);

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

  public void setPivotForwards() {
    m_pivotForwards = m_pivotLocked ? m_pivotForwards : true;
  }

  public void setPivotBackwards() {
    m_pivotForwards = m_pivotLocked ? m_pivotForwards : false;
  }

  private void movePivot() {
    m_pivot.set(ControlMode.MotionMagic, convertActualPositionToSensorPosition(m_pivotForwards ? 0 : 180));
  }

  private void lockPivot() {
    m_pivotLocked = true;
  }

  private void unlockPivot() {
    m_pivotLocked = true;
  }

  public double getPivotAngle() {
    return convertSensorPositionToActualPosition(m_pivot.getSelectedSensorPosition());
  }

  public double getPivotSpeed() {
    return convertSensorVelocityToActualVelocity(m_pivot.getSelectedSensorVelocity());
  }

  public double getPivotAbsoluteAngle() {
    return m_pivot.getSensorCollection().getPulseWidthPosition() - p_pivotOffset.getValue();
  }

  public void zeroRelativePivot() {
    m_pivot.setSelectedSensorPosition(getPivotAbsoluteAngle());
  }

  public void calibrateAbsolutePivot() {
    p_pivotOffset.setValue(0.);
    p_pivotOffset.setValue(getPivotAbsoluteAngle());
  }

  public void grabCube() {
    m_roller.set(p_grabCubeSpeed.getValue());
  }

  public void grabCone() {
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

  // COMMAND FACTORIES

  public CommandBase calibrateAbsolutePivotFactory() {
    return new InstantCommand(this::calibrateAbsolutePivot).ignoringDisable(true);
  }

  public CommandBase grabConeFactory() {
    return new RunCommand(() -> {grabCone(); movePivot(); lockPivot();}, this).withName("Grab Cone");
  }

  public CommandBase grabCubeFactory() {
    return new RunCommand(() -> {grabCube(); movePivot(); lockPivot();}, this).withName("Grab Cube");
  }

  public CommandBase dropConeFactory() {
    return new RunCommand(() -> {dropCone(); movePivot(); lockPivot();}, this).withName("Drop Cone");
  }

  public CommandBase dropCubeFactory() {
    return new RunCommand(() -> {dropCube(); movePivot(); lockPivot();}, this).withName("Drop Cube");
  }

  public CommandBase holdConeFactory() {
    return new RunCommand(() -> {holdCone(); movePivot(); unlockPivot();}, this).withName("Hold Cone");
  }

  public CommandBase holdCubeFactory() {
    return new RunCommand(() -> {holdCube(); movePivot(); unlockPivot();}, this).withName("Hold Cube");
  }

  public CommandBase holdFactory(BooleanSupplier coneMode) {
    return new RunCommand(() -> {
      if (coneMode.getAsBoolean()) {
        holdCone();
      } else {
        holdCube();
      }
    }, this);
  }

  public CommandBase setPivotForwardsFactory() {
    return new InstantCommand(() -> {
      if (hasGamePiece()) {
        setPivotForwards();
      } else {
        setPivotBackwards();
      }
    });
  }

  public CommandBase setPivotBackwardsFactory() {
    return new InstantCommand(this::setPivotBackwards);
  }

  @Override
  public void periodic() {
    if (m_pivot.hasResetOccurred()) {
      zeroRelativePivot();
    }

    SmartDashboard.putNumber("Grabber Pivot Angle", getPivotAngle());
    SmartDashboard.putNumber("Grabber Pivot Velocity", getPivotSpeed());
    SmartDashboard.putNumber("Grabber Pivot Absolute Angle", getPivotAbsoluteAngle());
  }
}
