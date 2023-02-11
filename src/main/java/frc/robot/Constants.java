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
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.56515; // 22.25 inches
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.57785; // 22.75 inches

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11; 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 0; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10; 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 6; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9; 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 8;

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
    public static final double DANGER_ANGLE = 5.0;
	
    // Controllers
	public static final int DRIVER_CONTROLLER_ID = 0;
	public static final int BUTTON_BOX_ID = 1;
    public static final int TEST_CONTROLLER_ID = 2;


    // Trajectory
    public static final double MAX_TRAJ_VELOCITY = 4;
    public static final double MAX_TRAJ_CENTRIP_ACC = 3;
    public static final double MAX_TRAJ_ACCELERATION = 3;
	
}