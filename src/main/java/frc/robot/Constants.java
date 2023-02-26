// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

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
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.55245; // 21.75 inches
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.55245; // 21.75 inches

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 18; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 19; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 19; 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 10; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 11; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11; 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 0; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 1; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 1; 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 9; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 9;

	public static final double XBOX_DEADBAND = 0.12;
	public static final double FRSKY_DEADBAND = 0.07;

	// Intake
	public static final String INTAKE_CANBUS = "1";
    public static final int INTAKE_OUTER_ROLLER_ID = 16;
	public static final int INTAKE_INNER_ROLLER_ID = 15;
	public static final int INTAKE_ARM_ID = 17;
    public static final int INTAKE_IR_ID = 0;

    // Lights
    public static final int CANDLE_ID = 0;
    public static final int JoystickId = 0;
    public static final int IncrementAnimButton = XboxController.Button.kRightBumper.value;
    public static final int DecrementAnimButton = XboxController.Button.kLeftBumper.value;
    public static final int BlockButton = XboxController.Button.kStart.value;
    public static final int MaxBrightnessAngle = 90;
    public static final int MidBrightnessAngle = 180;
    public static final int ZeroBrightnessAngle = 270;
    public static final int ChangeDirectionAngle = 0;
    public static final int VbatButton = XboxController.Button.kA.value;
    public static final int V5Button = XboxController.Button.kB.value;
    public static final int CurrentButton = XboxController.Button.kX.value;
    public static final int TemperatureButton = XboxController.Button.kY.value;
	
    // Controllers
    // Arm
    public static final int SHOULDER_ID = 4;
    public static final int ELBOW_ID = 3;
    public static final int WRIST_ID = 5;
    
    public static final int SHOULDER_ENCODER_ID = 4;
    public static final int ELBOW_ENCODER_ID = 3;
    public static final int WRIST_ENCODER_ID = 5;

    public static final int COAST_BUTTON_ID = 0;

    // Grabber
    public static final int GRABBER_PIVOT_ID = 13;
    public static final int GRABBER_ROLLER_ID = 12;

	// Controllers
	public static final int DRIVER_CONTROLLER_ID = 0;
	public static final int BUTTON_BOX_ID = 1;
    public static final int TEST_CONTROLLER_ID = 2;

    // Drive Train
    public static final int CHARGE_STATION_LEVEL = 2;

    // Trajectory
    public static final double MAX_TRAJ_VELOCITY = 4;
    public static final double MAX_TRAJ_CENTRIP_ACC = 3;
    public static final double MAX_TRAJ_ACCELERATION = 3;
	
    // Limelight
    public static final String LIMELIGHT_FRONT_NAME = "limelight";
    public static final String LIMELIGHT_BACK_NAME = "limelight-back";

    // Coprocessor
    public static final String COPROCESSOR_ADDRESS = "10.0.88.35";
    public static final int COPROCESSOR_PORT = 5800;
    public static final double COPROCESSOR_UPDATE_DELAY = 1.0/30;
    public static final double COPROCESSOR_UPDATE_DELAY_OFFSET = 0.01;
    public static final double COPROCESSOR_SLOW_UPDATE_DELAY = 1.0/5;
    public static final double COPROCESSOR_SLOW_UPDATE_DELAY_OFFSET = 0.02;
}