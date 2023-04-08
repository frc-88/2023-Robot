package frc.robot.ros.messages;

import java.util.ArrayList;

import frc.robot.grid_tracking.networktables.GameObject;
import frc.robot.grid_tracking.networktables.GridZone;

public class FrcGridTrackingMarkers {
    public double checkRadius = 0.0;
    public ArrayList<GridZone> grid_zones = new ArrayList<>();
    public ArrayList<GameObject> grid_objects = new ArrayList<>();

    public FrcGridTrackingMarkers(double checkRadius, ArrayList<GridZone> grid_zones,
            ArrayList<GameObject> grid_objects) {
        this.checkRadius = checkRadius;
        this.grid_zones = grid_zones;
        this.grid_objects = grid_objects;
    }
}
