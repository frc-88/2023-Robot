package frc.robot.util.arm;

public class ArmStates {
    public static final ArmState stowGeneric = new ArmState("Unused State", 80, -80, 70, 600).makeStow();
    public static final ArmState stowForHP = new ArmState("Stow For HP", 80, -80, 70, 600).makeStow();
    public static final ArmState stowFlat = new ArmState("Stow Flat", 10, -170, 10, 600).makeStow();
    public static final ArmState stowForHandoff = new ArmState("Stow For Handoff", 10, -170, 10, 600).makeStow();
    public static final ArmState flat = new ArmState("Flat", 10, -170, 10, 400).makeStow();
    public static final ArmState stowWithPiece = new ArmState("Stow With Piece", 10, -170, 10, 600).makeStow();
    public static final ArmState getConeFromIntake1 = new ArmState("Get Cone From Intake 1", 40, -180.8, 0, 600);
    public static final ArmState getConeFromIntake2 = new ArmState("Get Cone From Intake 2", 65.6, -125.5, 0, 600);
    public static final ArmState getCubeFromIntake1 = new ArmState("Get Cube From Intake 1", 53.2, -120.3, 0, 600);
    public static final ArmState getCubeFromIntake2 = new ArmState("Get Cube From Intake 2", 53.2, -120.3, 0, 600);
    public static final ArmState getConeFromShelf = new ArmState("Get Cone From Shelf", 85.3, 14, 34, 300);
    public static final ArmState getCubeFromShelf = new ArmState("Get Cube From Shelf", 77.6, 4, 31.7, 300);
    public static final ArmState getConeFromChute = new ArmState("Get Cone From Chute", 85.3, 14, 34, 300).addDeployIntermediaries(1).addRetractIntermediaries(1);
    public static final ArmState getCubeFromChute = new ArmState("Get Cube From Chute", 85.3, 14, 34, 300).addDeployIntermediaries(1).addRetractIntermediaries(1);
    public static final ArmState scoreConeHigh = new ArmState("Score Cone High", 132.2, -237.3, 149.2, 250);
    public static final ArmState scoreConeMiddle = new ArmState("Score Cone Middle", 54.4, -202.5, 165.9, 400);
    public static final ArmState scoreConeLow = new ArmState("Score Cone Low", 56.3, -87.4, 156.8, 600);
    public static final ArmState scoreConeMiddleFront = new ArmState("Score Cone Middle Front", 80, -80, 70, 300);
    public static final ArmState scoreConeLowFront = new ArmState("Score Cone Low Front", 80, -80, 70, 300);
    public static final ArmState scoreCubeHigh = new ArmState("Score Cube High", 131, -198.9, 135.9, 250).addDeployIntermediaries(1);
    public static final ArmState scoreCubeMiddle = new ArmState("Score Cube Middle", 78.2, -151.9, 149.5, 400);
    public static final ArmState scoreCubeLow = new ArmState("Score Cube Low", 64.3, -118.8, 160.7, 600);
    public static final ArmState scoreCubeMiddleFront = new ArmState("Score Cube Middle Front", 80, -80, 70, 300);
    public static final ArmState scoreCubeLowFront = new ArmState("Score Cube Low Front", 80, -80, 70, 300);
    public static final ArmState wristFlick = new ArmState("Wrist Flick", stowWithPiece.getShoulderAngle(), stowWithPiece.getElbowAngle(), 90, 800);
    public static final ArmState autoPrepScoreCone = new ArmState("Auto Prep Score Cone", 80, -80, 70, 300);
}
