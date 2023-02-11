package frc.robot.util.arm;

import java.util.function.Consumer;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class ArmJoint {
    
    private final WPI_TalonFX m_motor;
    private final WPI_CANCoder m_cancoder;

    private final String m_name;
    private final double m_ratio;
    private final double m_length;
    private final boolean m_encoderInverted;

    private boolean m_zeroed = false;

    private final DoublePreferenceConstant p_triggerCurrent;
    private final DoublePreferenceConstant p_triggerDuration;
    private final DoublePreferenceConstant p_continuousCurrent;
    private final DoublePreferenceConstant p_maxVelocity;
    private final DoublePreferenceConstant p_maxAcceleration;
    private final PIDPreferenceConstants p_pid;
    private final DoublePreferenceConstant p_encoderOffset;
    private final DoublePreferenceConstant p_gravityCompensation;
    
    public ArmJoint(String name, int motorID, int encoderID, boolean motorInverted, boolean encoderInverted, double ratio, double length) {
        m_name = name;
        m_ratio = ratio;
        m_length = length;
        m_encoderInverted = encoderInverted;
        
        m_motor = new WPI_TalonFX(motorID, "1");
        m_cancoder = new WPI_CANCoder(encoderID, "1");

        m_motor.configFactoryDefault();
        m_cancoder.configFactoryDefault();

        m_motor.setInverted(motorInverted);
        m_motor.setNeutralMode(NeutralMode.Brake);
        m_motor.configNeutralDeadband(0);
        m_motor.configMotionSCurveStrength(4);

        p_triggerCurrent = new DoublePreferenceConstant(name + " Trigger Current", 80);
        p_triggerDuration = new DoublePreferenceConstant(name + " Trigger Duration", 0.002);
        p_continuousCurrent = new DoublePreferenceConstant(name + " Continuous Current", 10);
        p_maxVelocity = new DoublePreferenceConstant(name + " Max Velocity", 0);
        p_maxAcceleration = new DoublePreferenceConstant(name + " Max Acceleration", 0);
        p_pid = new PIDPreferenceConstants(name, 0, 0, 0, 0, 0, 0, 0);
        p_encoderOffset = new DoublePreferenceConstant(name + " Offset", 0);
        p_gravityCompensation = new DoublePreferenceConstant(name + " Gravity Compensation", 0);

        Consumer<Double> handler = (Double unused) -> {
            StatorCurrentLimitConfiguration config = new StatorCurrentLimitConfiguration(
                true,
                p_continuousCurrent.getValue(),
                p_triggerCurrent.getValue(),
                p_triggerDuration.getValue()
            );
            m_motor.configStatorCurrentLimit(config);
            m_motor.config_kP(0, p_pid.getKP().getValue());
            m_motor.config_kI(0, p_pid.getKI().getValue());
            m_motor.config_kD(0, p_pid.getKD().getValue());
            m_motor.config_kF(0, p_pid.getKF().getValue());
            m_motor.config_IntegralZone(0, p_pid.getIZone().getValue());
            m_motor.configMaxIntegralAccumulator(0, p_pid.getIMax().getValue());
        };
        p_triggerCurrent.addChangeHandler(handler);
        p_triggerDuration.addChangeHandler(handler);
        p_continuousCurrent.addChangeHandler(handler);
        p_maxVelocity.addChangeHandler(handler);
        p_maxAcceleration.addChangeHandler(handler);
        p_pid.addChangeHandler(handler);

        handler.accept(0.);
    }

    public String getName() {
        return m_name;
    }

    public void setPercentOutput(double percent) {
        m_motor.set(TalonFXControlMode.PercentOutput, percent);
    }

    public void setMotionMagic(double angle, double speed) {
        m_motor.configMotionCruiseVelocity(convertActualVelocityToMotorVelocity(speed));
        m_motor.configMotionAcceleration(convertActualVelocityToMotorVelocity(p_maxAcceleration.getValue() * speed / p_maxVelocity.getValue()));
        
        m_motor.set(TalonFXControlMode.MotionMagic, convertActualPositionToMotorPosition(angle), DemandType.ArbitraryFeedForward, p_gravityCompensation.getValue() * Math.cos(Math.toRadians(getAngle())));
    }

    public void setMotionMagic(double angle) {
        setMotionMagic(angle, p_maxVelocity.getValue());
    }

    public void coast() {
        m_motor.setNeutralMode(NeutralMode.Coast);
    }

    public void brake() {
        m_motor.setNeutralMode(NeutralMode.Brake);
    }

    public double getAngle() {
        return convertMotorPositionToActualPosition(m_motor.getSelectedSensorPosition());
    }

    public Translation2d getPositionVector() {
        return new Translation2d(m_length, new Rotation2d(getAngle()));
    }

    public double getSpeed() {
        return convertMotorVelocityToActualVelocity(m_motor.getSelectedSensorVelocity());
    }

    public double getAbsoluteAngle() {
        return m_cancoder.getAbsolutePosition() * (m_encoderInverted ? -1. : 1.) - p_encoderOffset.getValue();
    }

    public double getMaxVelocity() {
        return p_maxVelocity.getValue();
    }

    public boolean isCancoderPresent() {
        return !(m_cancoder.getMagnetFieldStrength() == MagnetFieldStrength.BadRange_RedLED
                || m_cancoder.getMagnetFieldStrength() == MagnetFieldStrength.Invalid_Unknown
                || m_cancoder.getLastError() == ErrorCode.SensorNotPresent);
    }
    
    public void zeroRelative() {
        if (m_motor.hasResetOccurred()) {
            m_zeroed = false;
        }
        if (isCancoderPresent() && !m_zeroed) {
            m_motor.setSelectedSensorPosition(convertActualPositionToMotorPosition(getAbsoluteAngle()));
            m_zeroed = true;
        }
    }

    public boolean isZeroed() {
        return m_zeroed;
    }

    public void calibrateAbsolute(double angle) {
        p_encoderOffset.setValue(m_cancoder.getAbsolutePosition() * (m_encoderInverted ? -1. : 1.) - angle);
    }

    public void calibrateAbsolute() {
        calibrateAbsolute(0);
    }

    private double convertMotorPositionToActualPosition(double motorPosition) {
        return motorPosition * m_ratio * 360. / 2048.;
    }

    private double convertMotorVelocityToActualVelocity(double motorVelocity) {
        return convertMotorPositionToActualPosition(motorVelocity) * 10.;
    }

    private double convertActualPositionToMotorPosition(double actualPosition) {
        return actualPosition * 2048 / (m_ratio * 360);
    }

    private double convertActualVelocityToMotorVelocity(double actualVelocity) {
        return convertActualPositionToMotorPosition(actualVelocity) * 0.1;
    }
}