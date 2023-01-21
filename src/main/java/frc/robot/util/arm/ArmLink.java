package frc.robot.util.arm;

import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class ArmLink {
    
    private final WPI_TalonFX m_motor;

    private final String m_positionLabel;

    private static final double RATIO = 360. / (5. * 3. * 46./20. * 72./16. * 2048.);

    private static class MotorPreferences {
        private final DoublePreferenceConstant triggerCurrent;
        private final DoublePreferenceConstant triggerDuration;
        private final DoublePreferenceConstant continuousCurrent;
        private final DoublePreferenceConstant maxVelocity;
        private final DoublePreferenceConstant maxAcceleration;
        private final PIDPreferenceConstants pid;

        private List<WPI_TalonFX> motors;

        double ratio;

        public MotorPreferences(String prefix, double ratio) {
            this.ratio = ratio;

            triggerCurrent = new DoublePreferenceConstant(prefix + " Trigger Current", 80);
            triggerDuration = new DoublePreferenceConstant(prefix + " Trigger Duration", 0.002);
            continuousCurrent = new DoublePreferenceConstant(prefix + " Continuous Current", 10);
            maxVelocity = new DoublePreferenceConstant(prefix + " Max Velocity", 0);
            maxAcceleration = new DoublePreferenceConstant(prefix + " Max Acceleration", 0);
            pid = new PIDPreferenceConstants(prefix + " PID", 0, 0, 0, 0, 0, 0, 0);

            Consumer<Double> handler = (Double unused) -> updateController();
            triggerCurrent.addChangeHandler(handler);
            triggerDuration.addChangeHandler(handler);
            continuousCurrent.addChangeHandler(handler);
            maxVelocity.addChangeHandler(handler);
            maxAcceleration.addChangeHandler(handler);
            pid.addChangeHandler(handler);

            motors = new LinkedList<>();
        }

        public void registerMotor(WPI_TalonFX motor) {
            motors.add(motor);
            updateController();
        }

        private void updateController() {
            StatorCurrentLimitConfiguration config = new StatorCurrentLimitConfiguration(
                true,
                continuousCurrent.getValue(),
                triggerCurrent.getValue(),
                triggerDuration.getValue()
            );
            motors.forEach((WPI_TalonFX motor) -> {
                motor.configStatorCurrentLimit(config);
                motor.config_kP(0, pid.getKP().getValue());
                motor.config_kI(0, pid.getKI().getValue());
                motor.config_kD(0, pid.getKD().getValue());
                motor.config_kF(0, pid.getKF().getValue());
                motor.config_IntegralZone(0, pid.getIZone().getValue());
                motor.configMaxIntegralAccumulator(0, pid.getIMax().getValue());
            });
        }
    }

    private static MotorPreferences pivotPreferences;
    private static MotorPreferences telescopePreferences;

    private static boolean staticInitialized = false;

    private void staticInit() {
        staticInitialized = true;
    }

    public ArmLink(String positionLabel, int pivotID, int telescopeID, boolean pivotInverted, boolean telescopeInverted) {
        if (!staticInitialized) {
            staticInit();
        }

        m_motor = new WPI_TalonFX(pivotID, "1");
        m_positionLabel = positionLabel;

        m_motor.configFactoryDefault();

        if (pivotInverted || Robot.isSimulation()) {
            m_motor.setInverted(InvertType.InvertMotorOutput);
        } else {
            m_motor.setInverted(InvertType.None);
        }

        m_motor.setNeutralMode(NeutralMode.Brake);

        m_motor.configNeutralDeadband(0);

        m_motor.configMotionSCurveStrength(4);

        pivotPreferences.registerMotor(m_motor);
    }


    public void setPercentOutput(double pivotPercent, double telescopePercent) {
        m_motor.set(TalonFXControlMode.PercentOutput, pivotPercent);
    }

    public void coast() {
        m_motor.setNeutralMode(NeutralMode.Coast);
    }

    public void brake() {
        m_motor.setNeutralMode(NeutralMode.Brake);
    }

    public static double getPivotMaxVelocity() {
        return pivotPreferences.maxVelocity.getValue();
    }

    public boolean hasResetOccurred() {
        return m_motor.hasResetOccurred();
    }

    protected void enterCalibration() {
        m_motor.configForwardSoftLimitEnable(false);
        m_motor.configReverseSoftLimitEnable(false);
    }

    private static double convertMotorPositionToActualPosition(double motorPosition, double ratio) {
        return motorPosition * ratio;
    }

    private static double convertMotorVelocityToActualVelocity(double motorVelocity, double ratio) {
        return convertMotorPositionToActualPosition(motorVelocity, ratio) * 10.;
    }

    private static double convertActualPositiontoMotorPosition(double actualPosition, double ratio) {
        return actualPosition / ratio;
    }

    private static double convertActualVelocitytoMotorVelocity(double actualVelocity, double ratio) {
        return convertActualPositiontoMotorPosition(actualVelocity, ratio) * 0.1;
    }
}