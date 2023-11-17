package frc.robot.util.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;


public class FrskyDriverController extends FrskyController implements DriverController{

    public FrskyDriverController(int port) {
        super(port);
    }

    private static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance)
            return 0.0;

        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    private static double square(double value){
        return Math.copySign(value*value, value);
    }

    @Override
    public double getTranslationX() {
        return -square(deadband(getLeftStickX(), .02));
    }

    @Override
    public double getTranslationY() {
        return square(deadband(getLeftStickY(), .02));
    }

    @Override
    public double getRotation() {
        return deadband(getRightStickX(), .02);
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