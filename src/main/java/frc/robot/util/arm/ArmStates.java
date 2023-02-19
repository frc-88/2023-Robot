package frc.robot.util.arm;

public class ArmStates {
    public static final ArmState stow = new ArmState("Stow");
    public static final ArmState getConeFromIntake = new ArmState("Get Cone From Intake").addDeployIntermediaries(1);
    public static final ArmState getCubeFromIntake = new ArmState("Get Cube From Intake").addDeployIntermediaries(1);
    public static final ArmState getConeFromShelf = new ArmState("Get Cone From Shelf");
    public static final ArmState getCubeFromShelf = new ArmState("Get Cube From Shelf");
    public static final ArmState scoreConeHigh = new ArmState("Score Cone High");
    public static final ArmState scoreConeMiddle = new ArmState("Score Cone Middle");
    public static final ArmState scoreConeLow = new ArmState("Score Cone Low");
    public static final ArmState scoreConeMiddleFront = new ArmState("Score Cone Middle Front");
    public static final ArmState scoreConeLowFront = new ArmState("Score Cone Low Front");
    public static final ArmState scoreCubeHigh = new ArmState("Score Cube High");
    public static final ArmState scoreCubeMiddle = new ArmState("Score Cube Middle");
    public static final ArmState scoreCubeLow = new ArmState("Score Cube Low");
    public static final ArmState scoreCubeMiddleFront = new ArmState("Score Cube Middle Front");
    public static final ArmState scoreCubeLowFront = new ArmState("Score Cube Low Front");
}
