// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    ///////////////////////////////////////////////////////
    // DRIVETRAIN
    ///////////////////////////////////////////////////////

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.56515; // 22.25 inches
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.57785; // 22.75 inches
	public static final double DRIVETRAIN_BOUNDARY_WIDTH = 0.5;  // meters
    public static final double DRIVETRAIN_BOUNDARY_LENGTH = 0.5;  // meters
	public static final double DRIVETRAIN_MIN_COLLISION_INFLATE = 0.25;  // meters
    public static final double DRIVETRAIN_MAX_COLLISION_INFLATE = 1.75;  // meters

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 0; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 0; 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 18; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 19; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 18; 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 8; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 9; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 8; 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10;

	public static final double MAX_TRAJ_VELOCITY = 6.0;
	public static final double MAX_TRAJ_ACCELERATION = 4.0;
	public static final double MAX_TRAJ_CENTRIP_ACC = 1.5;

	public static final double XBOX_DEADBAND = 0.12;
	public static final double FRSKY_DEADBAND = 0.07;

	// Intake
	public static final int INTAKE_OUTER_ROLLER_ID = 1;
	public static final int INTAKE_INNER_ROLLER_ID = 0;
	public static final int INTAKE_ARM_ID = 12;

	// Controllers
	public static final int DRIVER_CONTROLLER_ID = 0;
	public static final int BUTTON_BOX_ID = 1;

	// ROS Interface
	public static final String COPROCESSOR_ADDRESS = "10.0.88.44";
	public static final String COPROCESSOR_ADDRESS_SIMULATED = "127.0.0.1";
	public static final int COPROCESSOR_PORT = 5800;
	public static final double COPROCESSOR_TABLE_UPDATE_DELAY = 1.0 / 30.0;
	public static final double COPROCESSOR_PERIODIC_UPDATE_DELAY = 1.0 / 30.0;
	public static final double COPROCESSOR_PERIODIC_UPDATE_OFFSET = 1.0 / 60.0;
	public static final double COPROCESSOR_SLOW_PERIODIC_UPDATE_DELAY = 1.0 / 5.0;
	public static final double COPROCESSOR_SLOW_PERIODIC_UPDATE_OFFSET = 0.025;
	
	public static final class AutoConstants {
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
	
		public static final double kPXController = 0.5;
		public static final double kPYController = 0.5;
		public static final double kPThetaController = 0.5;
	
		// Constraint for the motion profilied robot angle controller
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
			new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
	  }
}