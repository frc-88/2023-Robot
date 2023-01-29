package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.arm.ArmLink;

public class Arm extends SubsystemBase {
    
    private final BooleanSupplier coastButton;
    private boolean m_isCoasting = false;

    public final ArmLink shoulder;
    public final ArmLink elbow;
    public final ArmLink wrist;
    public final ArmLink pivot;

    private List<ArmLink> allArms;

    public Arm(BooleanSupplier coastButton) {
        this.coastButton = coastButton;

        shoulder = new ArmLink("Shoulder", Constants.SHOULDER_ID, false);
        elbow = new ArmLink("Elbow", Constants.ELBOW_ID, false);
        wrist = new ArmLink("Wrist", Constants.WRIST_ID, false);
        pivot = new ArmLink("Pivot", Constants.PIVOT_ID, false);

        allArms = Arrays.asList(new ArmLink[]{shoulder, elbow, wrist});
    }

    @Override
    public void periodic() {
        if (coastButton.getAsBoolean()) {
            if (!m_isCoasting) {
                allArms.forEach(ArmLink::coast);
                m_isCoasting = true;
            }
        }
        else {
            if (m_isCoasting) {
                allArms.forEach(ArmLink::brake);
                m_isCoasting = false;
            }
        }
    }
}