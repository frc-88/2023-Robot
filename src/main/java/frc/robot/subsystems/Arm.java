package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.arm.ArmJoint;

public class Arm extends SubsystemBase {
    
    private final BooleanSupplier coastButton;
    private boolean m_isCoasting = false;

    public final ArmJoint shoulder;
    public final ArmJoint elbow;
    public final ArmJoint wrist;
    public final ArmJoint pivot;

    private List<ArmJoint> allArms;

    public Arm(BooleanSupplier coastButton) {
        this.coastButton = coastButton;

        shoulder = new ArmJoint("Shoulder", Constants.SHOULDER_ID, false);
        elbow = new ArmJoint("Elbow", Constants.ELBOW_ID, false);
        wrist = new ArmJoint("Wrist", Constants.WRIST_ID, false);
        pivot = new ArmJoint("Pivot", Constants.PIVOT_ID, false);

        allArms = Arrays.asList(new ArmJoint[]{shoulder, elbow, wrist});
    }

    public void setGrabberUp() {
        pivot.setMotionMagic(0);
    }

    public void setGrabberDown() {
        pivot.setMotionMagic(180);
    }

    @Override
    public void periodic() {
        if (coastButton.getAsBoolean()) {
            if (!m_isCoasting) {
                allArms.forEach(ArmJoint::coast);
                m_isCoasting = true;
            }
        }
        else {
            if (m_isCoasting) {
                allArms.forEach(ArmJoint::brake);
                m_isCoasting = false;
            }
        }
    }
}