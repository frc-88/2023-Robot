// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Grabber extends SubsystemBase {
  private final WPI_TalonFX m_motor = new WPI_TalonFX(Constants.GRABBER_MOTOR_ID);

  private DoublePreferenceConstant intakeCubeSpeed =
    new DoublePreferenceConstant("Intake Cube Speed", 0.5);
  private DoublePreferenceConstant intakeConeSpeed =
    new DoublePreferenceConstant("Intake Cone Speed", -0.5);
  private DoublePreferenceConstant dropCubeSpeed =
    new DoublePreferenceConstant("Drop Cube Speed", -0.5);
  private DoublePreferenceConstant dropConeSpeed =
    new DoublePreferenceConstant("Drop Cone Speed", 0.5);
  
  /** Creates a new Grabber. */
  public Grabber() {

  }

  public void intakeCube() {
    m_motor.set(intakeCubeSpeed.getValue());
  }

  public void intakeCone() {
    m_motor.set(intakeConeSpeed.getValue());
  }

  public void dropCube() {
    m_motor.set(dropCubeSpeed.getValue());
  }
  
  public void dropCone() {
    m_motor.set(dropConeSpeed.getValue());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
