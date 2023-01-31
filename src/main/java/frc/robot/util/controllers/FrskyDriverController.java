package frc.robot.util.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FrskyDriverController extends FrskyController implements DriverController{

    public FrskyDriverController(int port) {
        super(port);
    }

    @Override
    public double getTranslationX() {
        return -getLeftStickX();
    }

    @Override
    public double getTranslationY() {
        return getLeftStickY();
    }

    @Override
    public double getRotation() {
        return getRightStickX();
    }

    @Override
    public boolean getGyroReset() {
        return isTopRightSwitchOn();
    }

    @Override
    public Trigger getShootButton() {
        return topLeftSwitch;
    }
    
}