package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    double getDistance();

    WPI_TalonFX getMotor();
}
