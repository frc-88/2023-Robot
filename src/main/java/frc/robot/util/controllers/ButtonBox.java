package frc.robot.util.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonBox extends Joystick {
	
    private static int INTAKE = 2;
    private static int OUTGEST = 3;
    private static int GAMEPIECE_SWITCH = 16;

	public ButtonBox(int port) {
		super(port);
	}

	public Trigger intakeButton = new JoystickButton(this, INTAKE);
    public Trigger outgestButton = new JoystickButton(this, OUTGEST);
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
