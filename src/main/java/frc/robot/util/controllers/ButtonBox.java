package frc.robot.util.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonBox extends Joystick {

    public enum ClimbBar {
        LOW,
        MID,
        HIGH,
        TRAVERSAL
    }
    public enum ClimbDirection {
        FORWARDS,
        BACKWARDS
    }
    public enum ClimbAction {
        PREP,
        RAISE,
        CLIMB
    }
	
    private static int INTAKE = 2;
    private static int OUTGEST = 3;
    private static int AUX_1 = 5;
    private static int AUX_2 = 4;
    private static int CENTRALIZER_UP = 6;
    private static int CENTRALIZER_DOWN = 7;
    private static int CHAMBER_UP = 8;
    private static int CHAMBER_DOWN = 9;
    private static int CLIMBER_STOW = 10;
    private static int CLIMBER_PREP = 11;
    private static int CLIMBER_RAISE = 12;
    private static int CLIMBER_CLIMB = 13;
    private static int SHOOT = 14;
    private static int TURRET_TRACK = 15;
    private static int CLIMBER_DIRECTION = 16;
    private static int AUX_3 = 17;
    private static int AUX_4 = 18;
    private static int MID_BAR = 20;
    private static int HIGH_BAR = 21;
    private static int TRAVERSAL_BAR = 19;

	public ButtonBox(int port) {
		super(port);
	}

	public Trigger intakeButton = new JoystickButton(this, INTAKE);
    public Trigger outgestButton = new JoystickButton(this, OUTGEST);
    public Trigger centralizerUp = new JoystickButton(this, CENTRALIZER_UP);
    public Trigger centralizerDown = new JoystickButton(this, CENTRALIZER_DOWN);
    public Trigger chamberUp = new JoystickButton(this, CHAMBER_UP);
    public Trigger chamberDown = new JoystickButton(this, CHAMBER_DOWN);
    public Trigger stowClimberButton = new JoystickButton(this, CLIMBER_STOW);
    public Trigger prepClimberButton = new JoystickButton(this, CLIMBER_PREP);
    public Trigger raiseClimberButton = new JoystickButton(this, CLIMBER_RAISE);
    public Trigger climbButton = new JoystickButton(this, CLIMBER_CLIMB);
    public Trigger shootButton = new JoystickButton(this, SHOOT);
    public Trigger turretTrackSwitch = new JoystickButton(this, CLIMBER_DIRECTION);
    public Trigger climbDirectionSwitch = new JoystickButton(this, TURRET_TRACK);
    public Trigger traversalBarSwitch = new JoystickButton(this, TRAVERSAL_BAR);
    public Trigger highBarSwitch = new JoystickButton(this, HIGH_BAR);
    public Trigger midBarSwitch = new JoystickButton(this, MID_BAR);
    public Trigger rosDisableSwitch = new JoystickButton(this, AUX_4);
    public Trigger autoShootSwitch = new JoystickButton(this, AUX_3);
    public Trigger defaultTurretButton = new JoystickButton(this, AUX_1);
    public Trigger cancelClimb = new JoystickButton(this, AUX_2);

	public boolean isIntakeButtonPressed() {
		return intakeButton.getAsBoolean();
	}

    public boolean isOutgestButtonPressed() {
		return outgestButton.getAsBoolean();
	}

    public boolean isCentralizerUpButtonPressed() {
        return centralizerUp.getAsBoolean();
    }

    public boolean isCentralizerDownButtonPressed() {
        return centralizerDown.getAsBoolean();
    }

    public boolean isChamberUpButtonPressed() {
        return chamberUp.getAsBoolean();
    }

    public boolean isChamberDownButtonPressed() {
        return chamberDown.getAsBoolean();
    }

    public boolean isStowClimberButtonPressed() {
		return stowClimberButton.getAsBoolean();
	}

    public boolean isPrepClimberButtonPressed() {
		return prepClimberButton.getAsBoolean();
	}

    public boolean isRaiseClimberButtonPressed() {
		return raiseClimberButton.getAsBoolean();
	}

    public boolean isClimbButtonPressed() {
		return climbButton.getAsBoolean();
	}

    public boolean isShootButtonPressed() {
		return shootButton.getAsBoolean();
	}

    public boolean isTrackTurretSwitchOn() {
        return turretTrackSwitch.getAsBoolean();
    }

    public ClimbDirection getClimbDirection() {
		// return climbDirectionSwitch.getAsBoolean() ? ClimbDirection.FORWARDS : ClimbDirection.BACKWARDS;
        return ClimbDirection.BACKWARDS;
	}

    public ClimbBar getClimbBar() {
        if (traversalBarSwitch.getAsBoolean()) {
            return ClimbBar.TRAVERSAL;
        } else if  (highBarSwitch.getAsBoolean()) {
            return ClimbBar.HIGH;
        } else if (midBarSwitch.getAsBoolean()) {
            return ClimbBar.MID;
        } else {
            return ClimbBar.LOW;
        }
    }

    public boolean isROSDisableSwitchOn() {
        return rosDisableSwitch.getAsBoolean();
    }

    public boolean isAutoShootSwitchOn() {
        return autoShootSwitch.getAsBoolean();
    }

    public boolean isDefaultTurretButtonPressed() {
        return defaultTurretButton.getAsBoolean();
    }

    public boolean isCancelClimbPressed() {
        return cancelClimb.getAsBoolean();
    }
}
