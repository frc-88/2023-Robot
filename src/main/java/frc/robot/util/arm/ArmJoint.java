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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class ArmJoint {
    
    private final WPI_TalonFX m_motor;
    private final WPI_CANCoder m_cancoder;

    private final String m_name;
    private final boolean m_motorInverted;
    private final double m_ratio;
    private final double m_length;
    private final boolean m_encoderInverted;
    private final double m_wrapAngle;

    private boolean m_zeroed = false;
    private boolean m_braking = true;

    private final DoublePreferenceConstant p_triggerCurrent;
    private final DoublePreferenceConstant p_triggerDuration;
    private final DoublePreferenceConstant p_continuousCurrent;
    private final DoublePreferenceConstant p_maxVelocity;
    private final DoublePreferenceConstant p_maxAcceleration;
    private final PIDPreferenceConstants p_pid;
    private final DoublePreferenceConstant p_encoderOffset;
    private final DoublePreferenceConstant p_gravityCompensation;
    
    public ArmJoint(String name, int motorID, int encoderID, boolean motorInverted, boolean encoderInverted, double ratio, double length, double wrapAngle) {
        m_name = name;
        m_motorInverted = motorInverted;
        m_ratio = ratio;
        m_length = length;
        m_encoderInverted = encoderInverted;
        m_wrapAngle = wrapAngle;
        
        m_motor = new WPI_TalonFX(motorID, "1");
        m_cancoder = new WPI_CANCoder(encoderID, "1");

        m_motor.configFactoryDefault();
        m_cancoder.configFactoryDefault();

        p_triggerCurrent = new DoublePreferenceConstant("Arm/" + name + "/Trigger Current", 120);
        p_triggerDuration = new DoublePreferenceConstant("Arm/" + name + "/Trigger Duration", 0.002);
        p_continuousCurrent = new DoublePreferenceConstant("Arm/" + name + "/Continuous Current", 80);
        p_maxVelocity = new DoublePreferenceConstant("Arm/" + name + "/Max Velocity", 0);
        p_maxAcceleration = new DoublePreferenceConstant("Arm/" + name + "/Max Acceleration", 0);
        p_pid = new PIDPreferenceConstants("Arm/" + name + "/");
        p_encoderOffset = new DoublePreferenceConstant("Arm/" + name + "/Offset", 0);
        p_gravityCompensation = new DoublePreferenceConstant("Arm/" + name + "/Gravity Compensation", 0);

        p_triggerCurrent.addChangeHandler(this::preferenceHandler);
        p_triggerDuration.addChangeHandler(this::preferenceHandler);
        p_continuousCurrent.addChangeHandler(this::preferenceHandler);
        p_pid.addChangeHandler(this::preferenceHandler);
        p_maxVelocity.addChangeHandler(this::preferenceHandler);
        p_maxAcceleration.addChangeHandler(this::preferenceHandler);

        configureMotor();
    }

    public void configureMotor() {
        m_motor.setInverted(m_motorInverted);
        m_motor.setNeutralMode(NeutralMode.Brake);
        m_motor.configNeutralDeadband(0);
        m_motor.configClosedloopRamp(0.05);
        m_motor.configMotionSCurveStrength(2);

        preferenceHandler(0.);
    }

    private void preferenceHandler(double unused) {
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
            m_motor.configMotionCruiseVelocity(convertActualVelocityToMotorVelocity(p_maxVelocity.getValue()));
            m_motor.configMotionAcceleration(convertActualVelocityToMotorVelocity(p_maxAcceleration.getValue()));
    }

    public String getName() {
        return m_name;
    }

    public void setPercentOutput(double percent) {
        m_motor.set(TalonFXControlMode.PercentOutput, percent);
    }

    public void stop() {
        setPercentOutput(0);
    }

    public WPI_TalonFX getMotor() {
        return m_motor;
    }

    public void setMotionMagic(double angle, double speed, double acceleration) {
        if (!m_zeroed) {
            setPercentOutput(0);
            return;
        }
        
        m_motor.configMotionCruiseVelocity(convertActualVelocityToMotorVelocity(speed));
        m_motor.configMotionAcceleration(convertActualVelocityToMotorVelocity(acceleration));
        
        m_motor.set(TalonFXControlMode.MotionMagic, convertActualPositionToMotorPosition(angle), DemandType.ArbitraryFeedForward, p_gravityCompensation.getValue() * Math.cos(Math.toRadians(getAngle())));
    }

    public void setMotionMagic(double angle) {
        setMotionMagic(angle, p_maxVelocity.getValue(), p_maxAcceleration.getValue());
    }

    public void coast() {
        if (m_braking) {
            m_motor.setNeutralMode(NeutralMode.Coast);
            m_braking = false;
        }
    }

    public void brake() {
        if (!m_braking) {
            m_motor.setNeutralMode(NeutralMode.Brake);
            m_braking = true;
        }
    }

    public double getAngle() {
        return convertMotorPositionToActualPosition(m_motor.getSelectedSensorPosition());
    }

    public Translation2d getPositionVector() {
        return new Translation2d(m_length, new Rotation2d(Math.toRadians(getAngle())));
    }

    public double getLength() {
        return m_length;
    }

    public double getSpeed() {
        return convertMotorVelocityToActualVelocity(m_motor.getSelectedSensorVelocity());
    }

    public double getAbsoluteAngle() {
        return ((m_cancoder.getAbsolutePosition() * (m_encoderInverted ? -1. : 1.) - p_encoderOffset.getValue()) + 720. - m_wrapAngle) % 360. - (360. - m_wrapAngle);
    }

    public double getMaxVelocity() {
        return p_maxVelocity.getValue();
    }

    public boolean isOnTarget(double targetAngle, double tolerance) {
        return (Math.abs(getAngle()-targetAngle)) < tolerance;
    }

    public boolean isOnTarget(double targetAngle) {
        return isOnTarget(targetAngle, p_pid.getTolerance().getValue());
    }

    public boolean isCancoderPresent() {
        return !(m_cancoder.getMagnetFieldStrength() == MagnetFieldStrength.BadRange_RedLED
                || m_cancoder.getMagnetFieldStrength() == MagnetFieldStrength.Invalid_Unknown
                || m_cancoder.getLastError() == ErrorCode.SensorNotPresent);
    }

    public void zeroRelative() {
        if (m_motor.hasResetOccurred()) {
            System.err.println(m_name + " has reset!");
            m_zeroed = false;
        }
        if (!m_zeroed && isCancoderPresent()) {
            System.out.println("Zeroing " + m_name);
            m_motor.setSelectedSensorPosition(convertActualPositionToMotorPosition(getAbsoluteAngle()));
            m_zeroed = true;
        } else if (!m_zeroed) {
            SmartDashboard.putString(getName() + " Last Error", m_cancoder.getLastError().toString());
        }
    }

    public boolean isZeroed() {
        return m_zeroed;
    }

    public void checkZero() {
        if (Math.abs(getSpeed()) < 2. && Math.abs(getAbsoluteAngle() - getAngle()) > 4.) {
            m_zeroed = false;
        }
    }

    public void calibrateAbsolute(double angle) {
        p_encoderOffset.setValue(0.);
        p_encoderOffset.setValue(getAbsoluteAngle() - angle);
        m_zeroed = false;
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
        return actualPosition * 2048. / (m_ratio * 360.);
    }

    private double convertActualVelocityToMotorVelocity(double actualVelocity) {
        return convertActualPositionToMotorPosition(actualVelocity) * 0.1;
    }
}