// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.coprocessor.networktables.ScorpionTable;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.IntPreferenceConstant;

import java.util.function.Supplier;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

/*
 *     everyone's talkin'
 * 'bout purple and yellow but
 *   tie dye rocks the house
 */

public class Lights extends SubsystemBase {
    private IntPreferenceConstant numLEDs = new IntPreferenceConstant("Number of LEDs", 60);
    private int m_state = 0;
    private int counter = 0;
    private final CANdle m_candle = new CANdle(Constants.CANDLE_ID);
    private boolean m_animDirection = false;
    private boolean m_setAnim = false;
    private DoublePreferenceConstant dangerAngle = new DoublePreferenceConstant("Danger Angle", 5.0);
    private double m_acceptableDifference = 0.25;

    private Animation m_toAnimate = null;
    private Animation m_lastAnimation = null;
    private Animation m_strobe = null;

    private SwerveDrive m_swerve;
    private Intake m_intake;
    private Arm m_arm;
    private Grabber m_grabber;
    private ScorpionTable m_coprocessor;
    private Limelight m_limelight;
    private Supplier<String> m_autoName;

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

    public Lights(SwerveDrive swerve, Intake intake, Arm arm, Grabber grabber, ScorpionTable coprocessor, Limelight limelight, Supplier<String> autoName) {
        m_swerve = swerve;
        m_intake = intake;
        m_arm = arm;
        m_grabber = grabber;
        m_coprocessor = coprocessor;
        m_limelight = limelight;
        m_autoName = autoName;
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 1.0;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    public void wantCone() {
        m_toAnimate = new StrobeAnimation(255, 200, 0, 0, 0.2, numLEDs.getValue(), 0);
        m_setAnim = true;
    }

    public void holdingCone() {
        m_toAnimate = new ColorFlowAnimation(255, 200, 0, 0, 0.2, numLEDs.getValue(), Direction.Forward);
        m_setAnim = true;
    }

    public void wantCube() {
        m_toAnimate = new StrobeAnimation(100, 0, 120, 0, 0.2, numLEDs.getValue(), 0);
        m_setAnim = true;
    }

    public void holdingCube() {
        m_toAnimate = new ColorFlowAnimation(100, 0, 120, 0, 0.2, numLEDs.getValue(), Direction.Forward);
        m_setAnim = true;
    }

    public void larsonColor(int r, int g, int b) {
        m_toAnimate = new LarsonAnimation(r, g, b, 0, 0.1, numLEDs.getValue(), BounceMode.Front, 5, 0);
        m_setAnim = true;
    }

    public void rainbow() {
        m_toAnimate = new RainbowAnimation(1, 0.7, numLEDs.getValue(), m_animDirection, 0);
        m_setAnim = true;
    }

    public void strobe(int r, int g, int b) {
        m_toAnimate = new StrobeAnimation(r, g, b, 0, 0.2, numLEDs.getValue(), 0);
        m_setAnim = true;
    }

    public boolean approximatelyEqual(Pose2d a, Pose2d b) {
        return a.getTranslation().getDistance(b.getTranslation()) < m_acceptableDifference;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            switch (m_state) {
                case 0:
                    larsonColor(255, 0, 0);
                    if (m_swerve.areAllCANDevicesPresent() && counter++ > 75) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                case 1:
                    larsonColor(255, 255, 0);
                    if (m_arm.isReady() && counter++ > 75) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                case 2:
                    larsonColor(0, 255, 0);
                    if (m_grabber.isReady() && m_grabber.hasGamePiece() && counter++ > 75) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                case 3:
                    larsonColor(0, 255, 255);
                    if ((m_intake.isArmDown() || DriverStation.isFMSAttached() || !m_autoName.get().equals("Wait")) && counter++ > 25) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                case 4:
                    larsonColor(0, 255, 255);
                    if ((m_intake.hasGamePiece() || DriverStation.isFMSAttached() || !m_autoName.get().equals("Wait")) && counter++ > 25) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                case 5:
                    larsonColor(0, 255, 255);
                    if (m_intake.isReady() && counter++ > 75) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                case 6:
                    larsonColor(0, 0, 255);
                    if ( m_coprocessor.isConnected() && m_coprocessor.getBotPose() != null && counter++ > 75) {
                        m_state++;
                        counter = 0;
                    }
                    break;

                case 7:
                    larsonColor(255, 0, 255);
                    m_limelight.setAprilTagPipeline();
                    if (approximatelyEqual(m_limelight.getBotPose(), m_coprocessor.getBotPose()) && counter++ > 75) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                case 8:
                    larsonColor(255, 255, 255);
                    m_swerve.resetPosition(m_coprocessor.getBotPose());
                    if (approximatelyEqual(m_swerve.getOdometryPose(), m_coprocessor.getBotPose()) && !m_autoName.get().equals("Wait") && counter++ > 75) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                case 9:
                    rainbow();
                    if (counter++ > 100) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                default:
                    // rainbow();
                    break;
            }
        }
        
        if (Math.abs(SmartDashboard.getNumber("NavX.pitch", 0.0)) > dangerAngle.getValue()
                || Math.abs(SmartDashboard.getNumber("NavX.roll", 0.0)) > dangerAngle.getValue()) {
            // if (m_lastAnimation == null) {
            //     m_lastAnimation = m_toAnimate;
            // }
            // strobe(255, 0, 0);
            // m_strobe = m_toAnimate;
        } else if (m_toAnimate.equals(m_strobe) && m_lastAnimation != null) {
            m_toAnimate = m_lastAnimation;
            m_setAnim = true;
            m_lastAnimation = null;
        }
        
        if (m_setAnim) {
            m_candle.clearAnimation(0);
            m_setAnim = false;
        }
        m_candle.animate(m_toAnimate, 0);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public InstantCommand wantConeFactory() {
        return new InstantCommand(() -> {
            wantCone();
        });
    }

    public InstantCommand holdingConeFactory() {
        return new InstantCommand(() -> {
            holdingCone();
        });
    }

    public InstantCommand wantCubeFactory() {
        return new InstantCommand(() -> {
            wantCube();
        });
    }

    public InstantCommand holdingCubeFactory() {
        return new InstantCommand(() -> {
            holdingCube();
        });
    }
}
