package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.coprocessor.GameObject;
import frc.robot.util.coprocessor.GridZone;
import frc.robot.util.coprocessor.detections.Detection;
import frc.robot.util.coprocessor.detections.Pose3d;
import frc.robot.util.coprocessor.networktables.ScorpionTable;

public class GameObjectManager extends SubsystemBase {
    public ArrayList<GameObject> gameObjects;
    public ArrayList<GridZone> gridZones;
    private ScorpionTable m_coprocessor;

    public GameObjectManager(ScorpionTable coprocessor) {
        m_coprocessor = coprocessor;
        gameObjects = new ArrayList<>();
        gridZones = new ArrayList<>();

        gridZones.add(new GridZone("CONE", "HIGH", 0., 0., 1.17));
        gridZones.add(new GridZone("CUBE", "HIGH", 0., 0., 1.17));
        gridZones.add(new GridZone("CONE", "HIGH", 0., 0., 1.17));
        gridZones.add(new GridZone("CONE", "HIGH", 0., 0., 1.17));
        gridZones.add(new GridZone("CUBE", "HIGH", 0., 0., 1.17));
        gridZones.add(new GridZone("CONE", "HIGH", 0., 0., 1.17));
        gridZones.add(new GridZone("CONE", "HIGH", 0., 0., 1.17));
        gridZones.add(new GridZone("CUBE", "HIGH", 0., 0., 1.17));
        gridZones.add(new GridZone("CONE", "HIGH", 0., 0., 1.17));

        gridZones.add(new GridZone("CONE", "MIDDLE", 0., 0., 0.9));
        gridZones.add(new GridZone("CUBE", "MIDDLE", 0., 0., 0.9));
        gridZones.add(new GridZone("CONE", "MIDDLE", 0., 0., 0.9));
        gridZones.add(new GridZone("CONE", "MIDDLE", 0., 0., 0.9));
        gridZones.add(new GridZone("CUBE", "MIDDLE", 0., 0., 0.9));
        gridZones.add(new GridZone("CONE", "MIDDLE", 0., 0., 0.9));
        gridZones.add(new GridZone("CONE", "MIDDLE", 0., 0., 0.9));
        gridZones.add(new GridZone("CUBE", "MIDDLE", 0., 0., 0.9));
        gridZones.add(new GridZone("CONE", "MIDDLE", 0., 0., 0.9));

        gridZones.add(new GridZone("EITHER", "LOW", 0., 0., 0.));
        gridZones.add(new GridZone("EITHER", "LOW", 0., 0., 0.));
        gridZones.add(new GridZone("EITHER", "LOW", 0., 0., 0.));
        gridZones.add(new GridZone("EITHER", "LOW", 0., 0., 0.));
        gridZones.add(new GridZone("EITHER", "LOW", 0., 0., 0.));
        gridZones.add(new GridZone("EITHER", "LOW", 0., 0., 0.));
        gridZones.add(new GridZone("EITHER", "LOW", 0., 0., 0.));
        gridZones.add(new GridZone("EITHER", "LOW", 0., 0., 0.));
        gridZones.add(new GridZone("EITHER", "LOW", 0., 0., 0.));
    }

    public void addGameObject(String name, double x, double y, double z, double yaw) {
        gameObjects.add(new GameObject(name, x, y, z, yaw));
    }

    public void removeInactiveGameObjects() {
        for (GameObject gameObject : gameObjects) {
            if (!gameObject.isValid()) {
                gameObjects.remove(gameObject);
            }
        }
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
            return links.get(0);
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
