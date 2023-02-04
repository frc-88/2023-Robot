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

  private DoublePreferenceConstant grabCubeSpeed =
    new DoublePreferenceConstant("Grab Cube Speed", 0.5);
  private DoublePreferenceConstant grabConeSpeed =
    new DoublePreferenceConstant("Grab Cone Speed", -0.5);
  private DoublePreferenceConstant dropCubeSpeed =
    new DoublePreferenceConstant("Drop Cube Speed", -0.5);
  private DoublePreferenceConstant dropConeSpeed =
    new DoublePreferenceConstant("Drop Cone Speed", 0.5);

  public Grabber() {

  }

  public void grabCube() {
    m_motor.set(grabCubeSpeed.getValue());
  }

  public void grabCone() {
    m_motor.set(grabConeSpeed.getValue());
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
