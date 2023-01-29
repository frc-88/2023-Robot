package frc.robot.util.arm;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Robot;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class ArmJoint {
    
    private final WPI_TalonFX m_motor;

    private static final double RATIO = 360. / (5. * 3. * 46./20. * 72./16. * 2048.);

    private final DoublePreferenceConstant triggerCurrent;
    private final DoublePreferenceConstant triggerDuration;
    private final DoublePreferenceConstant continuousCurrent;
    private final DoublePreferenceConstant maxVelocity;
    private final DoublePreferenceConstant maxAcceleration;
    private final PIDPreferenceConstants pid;
    
    public ArmJoint(String positionLabel, int motorID, boolean motorInverted) {
        m_motor = new WPI_TalonFX(motorID, "1");

        m_motor.configFactoryDefault();

        if (motorInverted || Robot.isSimulation()) {
            m_motor.setInverted(InvertType.InvertMotorOutput);
        } else {
            m_motor.setInverted(InvertType.None);
        }

        m_motor.setNeutralMode(NeutralMode.Brake);

        m_motor.configNeutralDeadband(0);

        m_motor.configMotionSCurveStrength(4);

        triggerCurrent = new DoublePreferenceConstant(positionLabel + " Trigger Current", 80);
        triggerDuration = new DoublePreferenceConstant(positionLabel + " Trigger Duration", 0.002);
        continuousCurrent = new DoublePreferenceConstant(positionLabel + " Continuous Current", 10);
        maxVelocity = new DoublePreferenceConstant(positionLabel + " Max Velocity", 0);
        maxAcceleration = new DoublePreferenceConstant(positionLabel + " Max Acceleration", 0);
        pid = new PIDPreferenceConstants(positionLabel + " PID", 0, 0, 0, 0, 0, 0, 0);

        Consumer<Double> handler = (Double unused) -> {
            StatorCurrentLimitConfiguration config = new StatorCurrentLimitConfiguration(
                true,
                continuousCurrent.getValue(),
                triggerCurrent.getValue(),
                triggerDuration.getValue()
            );
            m_motor.configStatorCurrentLimit(config);
            m_motor.config_kP(0, pid.getKP().getValue());
            m_motor.config_kI(0, pid.getKI().getValue());
            m_motor.config_kD(0, pid.getKD().getValue());
            m_motor.config_kF(0, pid.getKF().getValue());
            m_motor.config_IntegralZone(0, pid.getIZone().getValue());
            m_motor.configMaxIntegralAccumulator(0, pid.getIMax().getValue());
        };
        triggerCurrent.addChangeHandler(handler);
        triggerDuration.addChangeHandler(handler);
        continuousCurrent.addChangeHandler(handler);
        maxVelocity.addChangeHandler(handler);
        maxAcceleration.addChangeHandler(handler);
        pid.addChangeHandler(handler);

        handler.accept(0.);
    }

    public void setPercentOutput(double pivotPercent, double telescopePercent) {
        m_motor.set(TalonFXControlMode.PercentOutput, pivotPercent);
    }

    public void setMotionMagic(double motorAngle, double motorSpeed) {
        m_motor.configMotionCruiseVelocity(convertActualVelocityToMotorVelocity(motorSpeed));
        m_motor.configMotionAcceleration(convertActualVelocityToMotorVelocity(maxAcceleration.getValue() * maxVelocity.getValue() / motorSpeed));
        
        m_motor.set(TalonFXControlMode.MotionMagic, convertActualPositionToMotorPosition(motorAngle));
    }

    public void setMotionMagic(double pivotAngle) {
        setMotionMagic(pivotAngle, maxVelocity.getValue());
    }

    public void coast() {
        m_motor.setNeutralMode(NeutralMode.Coast);
    }

    public void brake() {
        m_motor.setNeutralMode(NeutralMode.Brake);
    }

    public boolean hasResetOccurred() {
        return m_motor.hasResetOccurred();
    }

    private static double convertMotorPositionToActualPosition(double motorPosition) {
        return motorPosition * RATIO;
    }

    private static double convertMotorVelocityToActualVelocity(double motorVelocity) {
        return convertMotorPositionToActualPosition(motorVelocity) * 10.;
    }

    private static double convertActualPositionToMotorPosition(double actualPosition) {
        return actualPosition / RATIO;
    }

    private static double convertActualVelocityToMotorVelocity(double actualVelocity) {
        return convertActualPositionToMotorPosition(actualVelocity) * 0.1;
    }
}