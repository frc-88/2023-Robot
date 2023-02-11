// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

public class CANdleSystem extends SubsystemBase {
  private final int LEDS_PER_ANIMATION = 22;
  private int m_state = 0;
  private int counter = 0;
    private final CANdle m_candle = new CANdle(Constants.CANDLE_ID);
    private boolean m_clearAllAnims = false;
    private boolean m_animDirection = false;
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

    public CANdleSystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
        rainbow();
    }

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

    public void larsonColor(int r, int g, int b) {
        m_toAnimate = new LarsonAnimation(r, g, b, 0, 0.2, LEDS_PER_ANIMATION, BounceMode.Front, 5, 0);
    }

    public void rainbow() {
        m_toAnimate = new RainbowAnimation(1, 0.7, LEDS_PER_ANIMATION, m_animDirection, 0);
    }

    @Override
    public void periodic() {
        switch(m_state){
            case 0: larsonColor(255,0,0);
            if (counter++>100) {
                m_state ++;
                counter = 0;
            }
            break;
            case 1: larsonColor(0,255,0);
            if (counter++>100) {
                m_state ++;
                counter = 0;
            }
            break;
            case 2: larsonColor(0,0,255);
            if (counter++>100) {
                m_state ++;
                counter = 0;
            }
            break;
            case 3: rainbow();
            // No default
        }
        // This method will be called once per scheduler run
        if(m_toAnimate == null) {
            if(!m_setAnim) {
                /* Only setLEDs once, because every set will transmit a frame */
                m_candle.setLEDs(100, 0, 120);
                m_setAnim = true;
            }
        } else {
            m_candle.animate(m_toAnimate);
            m_setAnim = false;
        }

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

    public InstantCommand wantConeFactory() {return new InstantCommand(() -> {wantCone();});}
    public InstantCommand holdingConeFactory() {return new InstantCommand(() -> {holdingCone();});}
    public InstantCommand wantCubeFactory() {return new InstantCommand(() -> {wantCube();});}
    public InstantCommand holdingCubeFactory() {return new InstantCommand(() -> {holdingCube();});}
}
