package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.coprocessor.GameObject;
import frc.robot.util.coprocessor.GridZone;

public class GameObjectManager extends SubsystemBase {
    public ArrayList<GameObject> gameObjects;
    public ArrayList<GridZone> gridZones;
    public DriverStation driverStation; 
    public GameObjectManager() {
        if(driverStation.getAlliance() == Alliance.Blue) {

            gameObjects = new ArrayList<>();
            gridZones = new ArrayList<>();

            gridZones.add(new GridZone("CONE", "HIGH", 14.32, 20., 46.));
            gridZones.add(new GridZone("CUBE", "HIGH", 14.32, 42., 35.5));
            gridZones.add(new GridZone("CONE", "HIGH", 14.32, 64., 46.));
            gridZones.add(new GridZone("CONE", "HIGH", 14.32, 86., 46.));
            gridZones.add(new GridZone("CUBE", "HIGH", 14.32, 108., 35.5));
            gridZones.add(new GridZone("CONE", "HIGH", 14.32, 130., 46.));
            gridZones.add(new GridZone("CONE", "HIGH", 14.32, 152., 46.));
            gridZones.add(new GridZone("CUBE", "HIGH", 14.32, 174., 35.5));
            gridZones.add(new GridZone("CONE", "HIGH", 14.32, 196., 46.));

            gridZones.add(new GridZone("CONE", "MIDDLE", 31.35, 20., 34.));
            gridZones.add(new GridZone("CUBE", "MIDDLE", 31.35, 42., 23.53));
            gridZones.add(new GridZone("CONE", "MIDDLE", 31.35, 64., 34.));
            gridZones.add(new GridZone("CONE", "MIDDLE", 31.35, 86., 34.));
            gridZones.add(new GridZone("CUBE", "MIDDLE", 31.35, 108., 23.53));
            gridZones.add(new GridZone("CONE", "MIDDLE", 31.35, 130., 34.));
            gridZones.add(new GridZone("CONE", "MIDDLE", 31.35, 152., 34.));
            gridZones.add(new GridZone("CUBE", "MIDDLE", 31.35, 174., 23.53));
            gridZones.add(new GridZone("CONE", "MIDDLE", 31.35, 196., 34.));

            gridZones.add(new GridZone("EITHER", "LOW", 46.91, 20., 0.));
            gridZones.add(new GridZone("EITHER", "LOW", 46.91, 42., 0.));
            gridZones.add(new GridZone("EITHER", "LOW", 46.91, 64., 0.));
            gridZones.add(new GridZone("EITHER", "LOW", 46.91, 86., 0.));
            gridZones.add(new GridZone("EITHER", "LOW", 46.91, 108., 0.));
            gridZones.add(new GridZone("EITHER", "LOW", 46.91, 130., 0.));
            gridZones.add(new GridZone("EITHER", "LOW", 46.91, 152., 0.));
            gridZones.add(new GridZone("EITHER", "LOW", 46.91, 174., 0.));
            gridZones.add(new GridZone("EITHER", "LOW", 46.91, 196., 0.));
        }
        else if(driverStation.getAlliance() == Alliance.Red) {
            gameObjects = new ArrayList<>();
            gridZones = new ArrayList<>();

            gridZones.add(new GridZone("CONE", "HIGH", 0., 0., 0.));
            gridZones.add(new GridZone("CUBE", "HIGH", 0., 0., 0.));
            gridZones.add(new GridZone("CONE", "HIGH", 0., 0., 0.));
            gridZones.add(new GridZone("CONE", "HIGH", 0., 0., 0.));
            gridZones.add(new GridZone("CUBE", "HIGH", 0., 0., 0.));
            gridZones.add(new GridZone("CONE", "HIGH", 0., 0., 0.));
            gridZones.add(new GridZone("CONE", "HIGH", 0., 0., 0.));
            gridZones.add(new GridZone("CUBE", "HIGH", 0., 0., 0.));
            gridZones.add(new GridZone("CONE", "HIGH", 0., 0., 0.));

            gridZones.add(new GridZone("CONE", "MIDDLE", 0., 0., 0.));
            gridZones.add(new GridZone("CUBE", "MIDDLE", 0., 0., 0.));
            gridZones.add(new GridZone("CONE", "MIDDLE", 0., 0., 0.));
            gridZones.add(new GridZone("CONE", "MIDDLE", 0., 0., 0.));
            gridZones.add(new GridZone("CUBE", "MIDDLE", 0., 0., 0.));
            gridZones.add(new GridZone("CONE", "MIDDLE", 0., 0., 0.));
            gridZones.add(new GridZone("CONE", "MIDDLE", 0., 0., 0.));
            gridZones.add(new GridZone("CUBE", "MIDDLE", 0., 0., 0.));
            gridZones.add(new GridZone("CONE", "MIDDLE", 0., 0., 0.));

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
        removeInactiveGameObjects();
        fillGridZones();
    }
}
