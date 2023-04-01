package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collection;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.coprocessor.GameObject;
import frc.robot.util.coprocessor.GridZone;
import frc.robot.util.coprocessor.detections.Detection;
import frc.robot.util.coprocessor.networktables.ScorpionTable;

public class GameObjectManager extends SubsystemBase {
    public ArrayList<GameObject> gameObjects;
    public ArrayList<GridZone> gridZones;
    private ScorpionTable m_coprocessor;

    public GameObjectManager(ScorpionTable coprocessor) {
        m_coprocessor = coprocessor;

        gameObjects = new ArrayList<>();
        ArrayList<GridZone> blueGridZones = new ArrayList<>();

        blueGridZones.add(new GridZone("CONE", "HIGH", 14.32, 20., 46.));
        blueGridZones.add(new GridZone("CUBE", "HIGH", 14.32, 42., 35.5));
        blueGridZones.add(new GridZone("CONE", "HIGH", 14.32, 64., 46.));
        blueGridZones.add(new GridZone("CONE", "HIGH", 14.32, 86., 46.));
        blueGridZones.add(new GridZone("CUBE", "HIGH", 14.32, 108., 35.5));
        blueGridZones.add(new GridZone("CONE", "HIGH", 14.32, 130., 46.));
        blueGridZones.add(new GridZone("CONE", "HIGH", 14.32, 152., 46.));
        blueGridZones.add(new GridZone("CUBE", "HIGH", 14.32, 174., 35.5));
        blueGridZones.add(new GridZone("CONE", "HIGH", 14.32, 196., 46.));

        blueGridZones.add(new GridZone("CONE", "MIDDLE", 31.35, 20., 34.));
        blueGridZones.add(new GridZone("CUBE", "MIDDLE", 31.35, 42., 23.53));
        blueGridZones.add(new GridZone("CONE", "MIDDLE", 31.35, 64., 34.));
        blueGridZones.add(new GridZone("CONE", "MIDDLE", 31.35, 86., 34.));
        blueGridZones.add(new GridZone("CUBE", "MIDDLE", 31.35, 108., 23.53));
        blueGridZones.add(new GridZone("CONE", "MIDDLE", 31.35, 130., 34.));
        blueGridZones.add(new GridZone("CONE", "MIDDLE", 31.35, 152., 34.));
        blueGridZones.add(new GridZone("CUBE", "MIDDLE", 31.35, 174., 23.53));
        blueGridZones.add(new GridZone("CONE", "MIDDLE", 31.35, 196., 34.));

        blueGridZones.add(new GridZone("EITHER", "LOW", 46.91, 20., 0.));
        blueGridZones.add(new GridZone("EITHER", "LOW", 46.91, 42., 0.));
        blueGridZones.add(new GridZone("EITHER", "LOW", 46.91, 64., 0.));
        blueGridZones.add(new GridZone("EITHER", "LOW", 46.91, 86., 0.));
        blueGridZones.add(new GridZone("EITHER", "LOW", 46.91, 108., 0.));
        blueGridZones.add(new GridZone("EITHER", "LOW", 46.91, 130., 0.));
        blueGridZones.add(new GridZone("EITHER", "LOW", 46.91, 152., 0.));
        blueGridZones.add(new GridZone("EITHER", "LOW", 46.91, 174., 0.));
        blueGridZones.add(new GridZone("EITHER", "LOW", 46.91, 196., 0.));

        ArrayList<GridZone> redGridZones = new ArrayList<>();

        for (GridZone blueGridZone : blueGridZones) {
            redGridZones.add(new GridZone(blueGridZone.getType(), blueGridZone.getLevel(), blueGridZone.getX(),
                    ScorpionTable.switchYAlliance(blueGridZone.getY()), blueGridZone.getZ()));
        }

        // redGridZones = redGridZones.stream().map((GridZone zone) -> new
        // GridZone(zone.getType(), zone.getLevel(), zone.getX(),
        // ScorpionTable.switchYAlliance(zone.getY()),
        // zone.getZ())).collect(Collectors.toCollection(ArrayList::new));

        if (DriverStation.getAlliance() == Alliance.Blue) {
            gridZones = blueGridZones;
        } else if (DriverStation.getAlliance() == Alliance.Red) {
            gridZones = redGridZones;
        }
    }

    public void addGameObject(String name, double x, double y, double z, double yaw) {
        gameObjects.add(new GameObject(name, Units.metersToInches(x), Units.metersToInches(y), Units.metersToInches(z), yaw)); 
    }

    public void removeInactiveGameObjects() {
        for (int i = gameObjects.size() - 1; i >= 0; i--) {
            if (!gameObjects.get(i).isValid()) {
                gameObjects.remove(i);
            }
        }
    }

    public void fillGridZones() {
        for (GridZone gridZone : gridZones) {
            gridZone.filled = false;
            for (GameObject gameObject : gameObjects) {
                if (gridZone.contains(gameObject)
                        && (gridZone.getType() == gameObject.getName() || gridZone.getLevel() == "LOW")) {
                    gridZone.filled = true;
                }
            }
        }
    }

    public void fillGridZonesColumn(int columnIndex) {
        ArrayList<GridZone> columnGridZones = new ArrayList<GridZone>();

        columnGridZones.add(gridZones.get(columnIndex));
        columnGridZones.add(gridZones.get(columnIndex + 9));
        columnGridZones.add(gridZones.get(columnIndex + 18));
        for (GridZone gridZone : columnGridZones) {
            gridZone.filled = false;
            for (GameObject gameObject : gameObjects) {
                if (gridZone.contains(gameObject)
                        && (gridZone.getType() == gameObject.getName() || gridZone.getLevel() == "LOW")) {
                    gridZone.filled = true;
                }
            }
        }
    }

    public ArrayList<Integer> getLinkIndices() {
        ArrayList<Integer> links = new ArrayList<>();
        for (int i = 1; i < 8; i++) {
            if (gridZones.get(i).filled) {
                if (gridZones.get(i - 1).filled && !gridZones.get(i + 1).filled && !links.contains(i + 1)) {
                    links.add(i + 1);
                } else if (gridZones.get(i + 1).filled && !gridZones.get(i - 1).filled && !links.contains(i - 1)) {
                    links.add(i - 1);
                }
            }
        }
        for (int i = 9; i < 17; i++) {
            if (gridZones.get(i).filled) {
                if (gridZones.get(i - 1).filled && !gridZones.get(i + 1).filled && !links.contains(i + 1)) {
                    links.add(i + 1);
                } else if (gridZones.get(i + 1).filled && !gridZones.get(i - 1).filled && !links.contains(i - 1)) {
                    links.add(i - 1);
                }
            }
        }
        for (int i = 19; i < 26; i++) {
            if (gridZones.get(i).filled) {
                if (gridZones.get(i - 1).filled && !gridZones.get(i + 1).filled && !links.contains(i + 1)) {
                    links.add(i + 1);
                } else if (gridZones.get(i + 1).filled && !gridZones.get(i - 1).filled && !links.contains(i - 1)) {
                    links.add(i - 1);
                }
            }
        }
        return links;
    }

    public int bestPlace() {
        ArrayList<Integer> links = getLinkIndices();
        if (links.size() > 0) {
            for (Integer i : links) {
                if (gridZones.get(i).getLevel() == "HIGH") {
                    return i;
                }
            }
            for (Integer i : links) {
                if (gridZones.get(i).getLevel() == "MIDDLE") {
                    return i;
                }
            }
            return links.get(0);
        }
        for (int i = 0; i < gridZones.size(); i++) {
            if (!gridZones.get(i).filled && gridZones.get(i).getType() == "HIGH") {
                return i;
            }
        }
        for (int i = 0; i < gridZones.size(); i++) {
            if (!gridZones.get(i).filled && gridZones.get(i).getType() == "MIDDLE") {
                return i;
            }
        }
        for (int i = 0; i < gridZones.size(); i++) {
            if (!gridZones.get(i).filled) {
                return i;
            }
        }
        return 0;
    }

    private Pose2d toCornerCoordinates(Pose2d pose) {
        Pose2d transformPose;
        if (DriverStation.getAlliance() == Alliance.Blue) {
            transformPose = new Pose2d(8.27,4.01, Rotation2d.fromDegrees(180));
        } else  {
            transformPose = new Pose2d(-8.27,-4.01, new Rotation2d());
        }

        return pose.relativeTo(transformPose);
    }

    @Override
    public void periodic() {
        if (m_coprocessor.isTagGlobalPoseActive()) {
            Collection<Detection> detections = m_coprocessor.getAllDetections();
            for (Detection d : detections) {
                Pose2d pos = toCornerCoordinates(new Pose2d(d.getX(), d.getY(), new Rotation2d(0.)));
                addGameObject(d.getName(), pos.getX(), pos.getY(), d.getZ(), 0);
            }
            removeInactiveGameObjects();

            int closestColumnIndex = 0;
            
            double distance = Double.POSITIVE_INFINITY;
            for (int i = 0; i < 9; i++) {
                if (Math.abs(gridZones.get(i).getY() - m_coprocessor.getTagGlobalPoseInches().getY()) < distance) {
                    closestColumnIndex = i;
                    distance = gridZones.get(i).getY() - m_coprocessor.getTagGlobalPoseInches().getY();
                }
            }
            
            fillGridZonesColumn(closestColumnIndex);

            GridZone low = gridZones.get(closestColumnIndex);
            GridZone mid = gridZones.get(closestColumnIndex + 9);
            GridZone high = gridZones.get(closestColumnIndex + 18);
            SmartDashboard.putBoolean("Low Zone Filled", low.filled);
            SmartDashboard.putBoolean("Mid Zone Filled", mid.filled);
            SmartDashboard.putBoolean("High Zone Filled", high.filled);
            SmartDashboard.putNumber("Number of seen game pieces", gameObjects.size());
            if (gameObjects.size() > 0) {
                SmartDashboard.putNumber("Random Game Object X", gameObjects.get(gameObjects.size()-1).getX());
                SmartDashboard.putNumber("Random Game Object Y", gameObjects.get(gameObjects.size()-1).getY());
            }
            // SmartDashboard.putNumber("Optimal piece placement index", bestPlace());
            SmartDashboard.putNumber("Closest Column Index", closestColumnIndex);
        }
    }
}
