package frc.robot.util.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonBox extends Joystick {
	
    private static int INTAKE = 10;
    private static int OUTGEST = 18;
    private static int HANDOFF = 8;
    private static int GRAB_FROM_SHELF = 20;
    private static int GRAB_FROM_CHUTE = 12;
    private static int SET_LOW = 11;
    private static int SET_MIDDLE = 2;
    private static int SET_HIGH = 15;
    private static int SCORE = 6;
    private static int SET_FLAT = 19;
    private static int GAMEPIECE_SWITCH = 17;
    private static int FORCE_PIVOT_FORWARDS = 25;
    private static int FORCE_PIVOT_BACKWARDS = 24;
    private static int ENABLE_AIMING = 23;
    private static int HP_MODE = 22;
    private static int INDICATE_MID = 5;
    private static int INDICATE_HIGH = 16;


	public ButtonBox(int port) {
		super(port);
	}

	public Trigger intakeButton = new JoystickButton(this, INTAKE);
    public Trigger outgestButton = new JoystickButton(this, OUTGEST);
    public Trigger handoffButton = new JoystickButton(this, HANDOFF);
    public Trigger getFromShelfButton = new JoystickButton(this, GRAB_FROM_SHELF);
    public Trigger getFromChuteButton = new JoystickButton(this, GRAB_FROM_CHUTE);
    public Trigger setLow = new JoystickButton(this, SET_LOW);
    public Trigger setMiddle = new JoystickButton(this, SET_MIDDLE);
    public Trigger setHigh = new JoystickButton(this, SET_HIGH);    
    public Trigger setFlat = new JoystickButton(this, SET_FLAT);    
    public Trigger scoreButton = new JoystickButton(this, SCORE);
    public Trigger gamepieceSwitch = new JoystickButton(this, GAMEPIECE_SWITCH);
    public Trigger forcePivotForwardsSwitch = new JoystickButton(this, FORCE_PIVOT_FORWARDS);
    public Trigger forcePivotBackwardsSwitch = new JoystickButton(this, FORCE_PIVOT_BACKWARDS);
    public Trigger enableAimingSwitch = new JoystickButton(this, ENABLE_AIMING);
    public Trigger hpModeSwitch = new JoystickButton(this, HP_MODE);
    public Trigger indicateMid = new JoystickButton(this, INDICATE_MID);
    public Trigger indicateHigh = new JoystickButton(this, INDICATE_HIGH);

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
