package frc.robot.ros.messages;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

public class FrcGameObjectsStamped {
    public long stamp = 0;
    public Collection<FrcGameObject> objects = new ArrayList<>();

    public FrcGameObjectsStamped() {

    }

    public FrcGameObjectsStamped(long stamp, ArrayList<FrcGameObject> objects) {
        this.stamp = stamp;
        this.objects = objects;
    }

    public FrcGameObjectsStamped(long stamp, FrcGameObject... objects) {
        this.stamp = stamp;
        this.objects = new ArrayList<FrcGameObject>(Arrays.asList(objects));
    }
}
