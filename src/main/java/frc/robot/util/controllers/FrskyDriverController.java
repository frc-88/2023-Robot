package frc.robot.util.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FrskyDriverController extends FrskyController implements DriverController{

    public FrskyDriverController(int port) {
        super(port);
    }

    @Override
    public double getTranslationX() {
        return getLeftStickX();
    }

    @Override
    public double getTranslationY() {
        return -getLeftStickY();
    }

    @Override
    public double getRotation() {
        return -getRightStickX();
    }

    @Override
    public Trigger getScoreButton() {
        return buttonA;
    }
    
    @Override
    public Trigger getPivotButton() {
        return topLeftSwitch;
    }
    
}