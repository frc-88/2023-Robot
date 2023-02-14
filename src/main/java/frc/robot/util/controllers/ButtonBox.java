package frc.robot.util.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonBox extends Joystick {
	
    private static int INTAKE = 2;
    private static int OUTGEST = 3;
    private static int GRAB_FROM_SHELF = 5;
    private static int SET_LOW = 10;
    private static int SET_MIDDLE = 11;
    private static int SET_HIGH = 12;
    private static int SCORE = 14;
    private static int GAMEPIECE_SWITCH = 16;

	public ButtonBox(int port) {
		super(port);
	}

	public Trigger intakeButton = new JoystickButton(this, INTAKE);
    public Trigger outgestButton = new JoystickButton(this, OUTGEST);
    public Trigger getFromShelfButton = new JoystickButton(this, GRAB_FROM_SHELF);
    public Trigger setLow = new JoystickButton(this, SET_LOW);
    public Trigger setMiddle = new JoystickButton(this, SET_MIDDLE);
    public Trigger setHigh = new JoystickButton(this, SET_HIGH);    
    public Trigger scoreButton = new JoystickButton(this, SCORE);
    public Trigger gamepieceSwitch = new JoystickButton(this, GAMEPIECE_SWITCH);

	public boolean isIntakeButtonPressed() {
		return intakeButton.getAsBoolean();
	}

    public boolean isOutgestButtonPressed() {
		return outgestButton.getAsBoolean();
	}

    public boolean isConeSelected() {
        return gamepieceSwitch.getAsBoolean();
    }
}
