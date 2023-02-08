// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
// import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
// import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
// import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
// import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

public class CANdleSystem extends SubsystemBase {
  private final int LEDS_PER_ANIMATION = 128;
    private final CANdle m_candle = new CANdle(Constants.CANDLE_ID);
    private XboxController joystick;
    private boolean m_clearAllAnims = false;
    //private boolean m_last5V = false;
    //private boolean m_animDirection = false;
    private boolean m_setAnim = false;

    private Animation m_toAnimate = null;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll,
        Empty
    }
    //private AnimationTypes m_currentAnimation;

    public CANdleSystem(XboxController joy) {
        this.joystick = joy;
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    // public double getVbat() { return m_candle.getBusVoltage(); }
    // public double get5V() { return m_candle.get5VRailVoltage(); }
    // public double getCurrent() { return m_candle.getCurrent(); }
    // public double getTemperature() { return m_candle.getTemperature(); }
    // public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    // public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    // public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    // public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    public void clearAllAnims() {m_clearAllAnims = true;}

    public void wantCone() {
        m_toAnimate = new StrobeAnimation(255, 200, 0, 0, 0.2, LEDS_PER_ANIMATION, 0);
    }

    public void holdingCone() {
        m_toAnimate = new ColorFlowAnimation(255, 200, 0, 0, 0.2, LEDS_PER_ANIMATION, Direction.Forward);
    }

    public void wantCube() {
        m_toAnimate = new StrobeAnimation(100, 0, 120, 0, 0.2, LEDS_PER_ANIMATION, 0);
    }

    public void holdingCube() {
        m_toAnimate = new ColorFlowAnimation(100, 0, 120, 0, 0.2, LEDS_PER_ANIMATION, Direction.Forward);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(m_toAnimate == null) {
            if(!m_setAnim) {
                /* Only setLEDs once, because every set will transmit a frame */
                m_candle.setLEDs(100, 0, 120);
                m_setAnim = true;
            }
        } else {
            // m_toAnimate.setSpeed((joystick.getRightY() + 1.0) / 2.0);
            // m_candle.animate(m_toAnimate, m_candleChannel);
            m_candle.animate(m_toAnimate);
            m_setAnim = false;
        }
        m_candle.modulateVBatOutput(joystick.getRightY());

        if(m_clearAllAnims) {
            m_clearAllAnims = false;
            for(int i = 0; i < 10; ++i) {
                m_candle.clearAnimation(i);
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
