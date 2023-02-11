// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Grabber extends SubsystemBase {

  private final WPI_TalonSRX m_pivot = new WPI_TalonSRX(Constants.GRABBER_PIVOT_ID);
  private final WPI_TalonSRX m_roller = new WPI_TalonSRX(Constants.GRABBER_ROLLER_ID);

  private DoublePreferenceConstant p_pivotOffset = 
    new DoublePreferenceConstant("Grabber Pivot Offset", 0);

  private final DoublePreferenceConstant p_pivotMaxVelocity = 
    new DoublePreferenceConstant("Grabber Pivot Max Velocity", 0);
  private final DoublePreferenceConstant p_pivotMaxAcceleration = 
    new DoublePreferenceConstant("Grabber Pivot Max Acceleration", 0);;
  private final PIDPreferenceConstants p_pivotPID = 
    new PIDPreferenceConstants("Grabber Pivot");;

  private DoublePreferenceConstant p_grabCubeSpeed =
    new DoublePreferenceConstant("Grab Cube Speed", 0.5);
  private DoublePreferenceConstant p_grabConeSpeed =
    new DoublePreferenceConstant("Grab Cone Speed", -0.5);
  private DoublePreferenceConstant p_dropCubeSpeed =
    new DoublePreferenceConstant("Drop Cube Speed", -0.5);
  private DoublePreferenceConstant p_dropConeSpeed =
    new DoublePreferenceConstant("Drop Cone Speed", 0.5);

  public Grabber() {
    m_pivot.configFactoryDefault();
    m_roller.configFactoryDefault();

    m_pivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    zeroRelativePivot();

    m_pivot.configNeutralDeadband(0);
    m_pivot.setNeutralMode(NeutralMode.Brake);
    m_pivot.configMotionSCurveStrength(4);

    Consumer<Double> handler = (Double unused) -> {
      m_pivot.config_kP(0, p_pivotPID.getKP().getValue());
      m_pivot.config_kI(0, p_pivotPID.getKI().getValue());
      m_pivot.config_kD(0, p_pivotPID.getKD().getValue());
      m_pivot.config_kF(0, p_pivotPID.getKF().getValue());
      m_pivot.config_IntegralZone(0, p_pivotPID.getIZone().getValue());
      m_pivot.configMaxIntegralAccumulator(0, p_pivotPID.getIMax().getValue());
      m_pivot.configMotionCruiseVelocity(convertActualVelocityToSensorVelocity(p_pivotMaxVelocity.getValue()));
      m_pivot.configMotionAcceleration(convertActualVelocityToSensorVelocity(p_pivotMaxAcceleration.getValue()));
    };
    p_pivotPID.addChangeHandler(handler);
    p_pivotMaxVelocity.addChangeHandler(handler);
    p_pivotMaxAcceleration.addChangeHandler(handler);
    
    handler.accept(0.);
  }

  public void setPivotForwards() {
    m_pivot.set(ControlMode.MotionMagic, convertActualPositionToSensorPosition(0));
  }

  public void setPivotBackwards() {
    m_pivot.set(ControlMode.MotionMagic, convertActualPositionToSensorPosition(180));
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
