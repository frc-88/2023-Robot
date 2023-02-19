package frc.robot.util.arm;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class ArmState {
    private final String m_name;
    private final DoublePreferenceConstant m_shoulderAngle;
    private final DoublePreferenceConstant m_elbowAngle;
    private final DoublePreferenceConstant m_wristAngle;
    private List<ArmState> m_comboIntermediaries = new ArrayList<>();
    private List<ArmState> m_deployIntermediaries = new ArrayList<>();
    private List<ArmState> m_retractIntermediaries = new ArrayList<>();
    private DoublePreferenceConstant p_deployIntermediaryTolerance;
    private DoublePreferenceConstant p_retractIntermediaryTolerance;

    ArmState(String name) {
        m_name = name;

        m_shoulderAngle = new DoublePreferenceConstant("Arm/States/" + name + "/Shoulder Angle", 0);
        m_elbowAngle = new DoublePreferenceConstant("Arm/States/" + name + "/Elbow Angle", 0);
        m_wristAngle = new DoublePreferenceConstant("Arm/States/" + name + "/Wrist Angle", 0);
    }

    public ArmState addComboIntermediaries(int numIntermediaries) {
        for (int n = 1; n <= numIntermediaries; n++) {
            m_comboIntermediaries.add(new ArmState(m_name + "/Combo " + n));
        }
        p_deployIntermediaryTolerance = new DoublePreferenceConstant("Arm/States/" + m_name + "/Deploy Tolerance", 2);
        p_retractIntermediaryTolerance = new DoublePreferenceConstant("Arm/States/" + m_name + "/Retract Tolerance", 2);
        return this;
    }

    public ArmState addDeployIntermediaries(int numIntermediaries) {
        for (int n = 1; n <= numIntermediaries; n++) {
            m_deployIntermediaries.add(new ArmState(m_name + "/Deploy " + n));
        }
        p_deployIntermediaryTolerance = new DoublePreferenceConstant("Arm/States/" + m_name + "/Deploy Tolerance", 2);
        return this;
    }

    public ArmState addRetractIntermediaries(int numIntermediaries) {
        for (int n = 1; n <= numIntermediaries; n++) {
            m_retractIntermediaries.add(new ArmState(m_name + "/Retract " + n));
        }
        p_retractIntermediaryTolerance = new DoublePreferenceConstant("Arm/States/" + m_name + "/Retract Tolerance", 2);
        return this;
    }

    public String getName() {
        return m_name;
    }

    public double getShoulderAngle() {
        return m_shoulderAngle.getValue();
    }

    public double getElbowAngle() {
        return m_elbowAngle.getValue();
    }

    public double getWristAngle() {
        return m_wristAngle.getValue();
    }

    public List<ArmState> getDeployIntermediaries() {
        if (m_comboIntermediaries.size() > 0) {
            return m_comboIntermediaries;
        } else {
            return m_deployIntermediaries;
        }
    }

    public List<ArmState> getRetractIntermediaries() {
        if (m_comboIntermediaries.size() > 0) {
            return m_comboIntermediaries;
        } else {
            return m_retractIntermediaries;
        }
    }
    
    public double getDeployIntermediaryTolerance() {
        return Objects.nonNull(p_deployIntermediaryTolerance) ? p_deployIntermediaryTolerance.getValue() : 0;
    }

    public double getRetractIntermediaryTolerance() {
        return Objects.nonNull(p_retractIntermediaryTolerance) ? p_retractIntermediaryTolerance.getValue() : 0;
    }
}
