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

    public final ArmLink outerArm;
    public final ArmLink innerArm;

    private List<ArmLink> allArms;

    public Arm(BooleanSupplier coastButton) {
        this.coastButton = coastButton;

        outerArm = new ArmLink("Outer Left", Constants.OUTER_CLIMBER_PIVOT_ID, Constants.OUTER_CLIMBER_TELESCOPE_ID, false, false);
        innerArm = new ArmLink("Inner Right", Constants.INNER_CLIMBER_PIVOT_ID, Constants.INNER_CLIMBER_TELESCOPE_ID, true, false);

        allArms = Arrays.asList(new ArmLink[]{outerArm, innerArm});
    }

    public void setInnerPercentOutput(double innerPivotOutput, double innerTelescopeOutput) {
        this.innerArm.setPercentOutput(innerPivotOutput, innerTelescopeOutput);
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