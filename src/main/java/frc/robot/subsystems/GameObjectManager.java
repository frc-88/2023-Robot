package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collection;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

        redGridZones = redGridZones.stream().map((GridZone zone) -> new GridZone(zone.getType(), zone.getLevel(), zone.getX(), 
            ScorpionTable.switchYAlliance(zone.getY()), zone.getZ())).collect(Collectors.toCollection(ArrayList::new));

        if(DriverStation.getAlliance() == Alliance.Blue) {
            gridZones = blueGridZones;
        }
        else if(DriverStation.getAlliance() == Alliance.Red) { 
            gridZones = redGridZones;
        }
    }

    public void addGameObject(String name, double x, double y, double z, double yaw) {
        // gameObjects.add(new GameObject(name, x, y, z, yaw));
    }

    public void removeInactiveGameObjects() {
        // for (GameObject gameObject : gameObjects) {
        //     if (!gameObject.isValid()) {
        //         gameObjects.remove(gameObject);
        //     }
        // }
    }

    public void fillGridZones() {
        for (GridZone gridZone : gridZones) {
            gridZone.filled = false;
            for (GameObject gameObject : gameObjects) {
                if (gridZone.contains(gameObject) && (gridZone.getType() == gameObject.getName() || gridZone.getLevel() == "LOW")) {
                    gridZone.filled = true;
                }
            }
        }
    }

    public ArrayList<Integer> getLinkIndices() {
        ArrayList<Integer> links = new ArrayList<>();
        for (int i = 1; i < 8; i++) {
            if (gridZones.get(i).filled) {
                if (gridZones.get(i-1).filled && !gridZones.get(i+1).filled && !links.contains(i+1)) {
                    links.add(i+1);
                } else if (gridZones.get(i+1).filled && !gridZones.get(i-1).filled && !links.contains(i-1)) {
                    links.add(i-1);
                }
            }
        }
        for (int i = 9; i < 17; i++) {
            if (gridZones.get(i).filled) {
                if (gridZones.get(i-1).filled && !gridZones.get(i+1).filled && !links.contains(i+1)) {
                    links.add(i+1);
                } else if (gridZones.get(i+1).filled && !gridZones.get(i-1).filled && !links.contains(i-1)) {
                    links.add(i-1);
                }
            }
        }
        for (int i = 19; i < 26; i++) {
            if (gridZones.get(i).filled) {
                if (gridZones.get(i-1).filled && !gridZones.get(i+1).filled && !links.contains(i+1)) {
                    links.add(i+1);
                } else if (gridZones.get(i+1).filled && !gridZones.get(i-1).filled && !links.contains(i-1)) {
                    links.add(i-1);
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

    @Override
    public void periodic() {
        Collection<Detection> detections = m_coprocessor.getAllDetections();
        for (Detection d : detections) {
            Pose2d pos = new Pose2d(d.getPosition().x, d.getPosition().y, new Rotation2d(0.));
            Pose2d globalpos = pos.transformBy(new Transform2d(m_coprocessor.getTagGlobalPose(), new Pose2d()));
            addGameObject(d.getName(), globalpos.getX(), globalpos.getY(), d.getPosition().z, 0);
        }
        removeInactiveGameObjects();
        fillGridZones();
    }
}
