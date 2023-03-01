package frc.robot.util.arm;

public class ArmStates {
    public static final ArmState stow = new ArmState("Stow", 80, -80, 70).makeStow();
    public static final ArmState getConeFromIntake2 = new ArmState("Get Cone From Intake 1", 40, -180.8, 0);
    public static final ArmState getConeFromIntake1 = new ArmState("Get Cone From Intake 2", 65.6, -125.5, 0);
    public static final ArmState getCubeFromIntake1 = new ArmState("Get Cube From Intake 1", 53.2, -120.3, 0);
    public static final ArmState getCubeFromIntake2 = new ArmState("Get Cube From Intake 2", 53.2, -120.3, 0);
    public static final ArmState getConeFromShelf = new ArmState("Get Cone From Shelf", 85.3, 14, 34);
    public static final ArmState getCubeFromShelf = new ArmState("Get Cube From Shelf", 77.6, 4, 31.7).addDeployIntermediaries(1).addRetractIntermediaries(1);
    public static final ArmState scoreConeHigh = new ArmState("Score Cone High", 132.2, -237.3, 149.2).addDeployIntermediaries(1).addRetractIntermediaries(1);
    public static final ArmState scoreConeMiddle = new ArmState("Score Cone Middle", 54.4, -202.5, 165.9).addDeployIntermediaries(1).addRetractIntermediaries(1);
    public static final ArmState scoreConeLow = new ArmState("Score Cone Low", 56.3, -87.4, 156.8);
    public static final ArmState scoreConeMiddleFront = new ArmState("Score Cone Middle Front", 80, -80, 70);
    public static final ArmState scoreConeLowFront = new ArmState("Score Cone Low Front", 80, -80, 70);
    public static final ArmState scoreCubeHigh = new ArmState("Score Cube High", 131, -198.9, 135.9).addDeployIntermediaries(1).addRetractIntermediaries(1);
    public static final ArmState scoreCubeMiddle = new ArmState("Score Cube Middle", 78.2, -151.9, 149.5);
    public static final ArmState scoreCubeLow = new ArmState("Score Cube Low", 64.3, -118.8, 160.7);
    public static final ArmState scoreCubeMiddleFront = new ArmState("Score Cube Middle Front", 80, -80, 70);
    public static final ArmState scoreCubeLowFront = new ArmState("Score Cube Low Front", 80, -80, 70);
}
