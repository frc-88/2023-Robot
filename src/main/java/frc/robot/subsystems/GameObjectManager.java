package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.coprocessor.GameObject;

public class GameObjectManager extends SubsystemBase {
    public Set<GameObject> gameObjects;

    public GameObjectManager() {
        gameObjects = new HashSet<>();
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

    @Override
    public void periodic() {
        removeInactiveGameObjects();
    }
}
