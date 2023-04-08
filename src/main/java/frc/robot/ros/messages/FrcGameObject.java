package frc.robot.ros.messages;

import edu.wpi.first.math.geometry.Pose3d;

public class FrcGameObject {
    public Pose3d pose = new Pose3d();
    public String label = "";
    public int object_index = 0;
    public int class_index = 0;
    public double confidence = 0.0;

    public FrcGameObject() {

    }

    public FrcGameObject(Pose3d pose, String label, int object_index, int class_index, double confidence) {
        this.pose = pose;
        this.label = label;
        this.object_index = object_index;
        this.class_index = class_index;
        this.confidence = confidence;
    }
}
