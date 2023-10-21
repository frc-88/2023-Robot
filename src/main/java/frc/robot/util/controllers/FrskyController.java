package frc.robot.util.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class FrskyController extends Joystick {
/* 	private static final int LEFT_HORIZ_AXIS = 3;
	private static final int LEFT_VERT_AXIS = 0;
	private static final int RIGHT_HORIZ_AXIS = 1;
	private static final int RIGHT_VERT_AXIS = 2;
	private static final int TOP_LEFT_SWITCH_AXIS = 5;
    private static final int TOP_RIGHT_SWITCH_AXIS = 4; */
	
	private static final int LEFT_HORIZ_AXIS = 0;
	private static final int LEFT_VERT_AXIS = 1;
	private static final int RIGHT_HORIZ_AXIS = 4;
	private static final int RIGHT_VERT_AXIS = 5;
	private static final int TOP_LEFT_SWITCH_AXIS = 6;
    private static final int TOP_RIGHT_SWITCH_AXIS = 5;
	private static final int BUTTON_A = 1;
	public Trigger buttonA = new JoystickButton(this, BUTTON_A);
	
	public FrskyController(int port) {
		super(port);
	}

	public Trigger topLeftSwitch = new Trigger(() -> getRawAxis(TOP_LEFT_SWITCH_AXIS) > 0.5);	
	public Trigger topRightSwitch = new Trigger(() -> getRawAxis(TOP_RIGHT_SWITCH_AXIS) > 0.5);

	

	public boolean isTopLeftSwitchOn() {
		return topLeftSwitch.getAsBoolean();
	}
	
/* 	public boolean isTopRightSwitchOn() {
		return A_Button.getAsBoolean();
	} */

	public boolean isButtonAPressed() {
		return this.getRawButton(BUTTON_A);
	}
	
	public double getRightStickY() {
		return this.getRawAxis(RIGHT_VERT_AXIS);
	}

	public double getRightStickX() {
		return this.getRawAxis(RIGHT_HORIZ_AXIS)*-1;
	}

	public double getLeftStickY() {
		return this.getRawAxis(LEFT_VERT_AXIS)*-1;
	}

	public double getLeftStickX() {
		return this.getRawAxis(LEFT_HORIZ_AXIS);
	}
}