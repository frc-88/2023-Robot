package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    void checkAngleReset();

    WPI_TalonFX getMotor();

    AbsoluteEncoder getEncoder();
}
