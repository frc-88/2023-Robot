package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.coprocessor.GameObject;
import frc.robot.util.coprocessor.GridZone;

public class GameObjectManager extends SubsystemBase {
    public Set<GameObject> gameObjects;
    public Set<GridZone> gridZones;

    public GameObjectManager() {
        gameObjects = new HashSet<>();
        gridZones = new HashSet<>();
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
            for (GameObject gameObject : gameObjects) {
                if (gridZone.contains(gameObject) && (gridZone.getType() == gameObject.getName() || gridZone.getLevel() == "LOW")) {
                    gridZone.filled = true;
                }
            }
        }
    }

    @Override
    public void periodic() {
        removeInactiveGameObjects();

    }
}
