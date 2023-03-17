package com.swervedrivespecialties.swervelib;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    DriveController getDriveController();

    SteerController getSteerController();

    SwerveModulePosition getPosition();

    void zeroModule();

    void set(double driveVoltage, double steerAngle);

    boolean areAllCANDevicesPresent();
}
