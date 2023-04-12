package frc.robot.util.coprocessor.detections;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;

public class DetectionManager {
    private Map<String, List<Detection>> detections;
    private Map<String, Integer> detectionCounts;

    public DetectionManager() {
        detections = new HashMap<>();
        detectionCounts = new HashMap<>();
    }

    public void setDetection(String name, int index, Detection detection) {
        if (detections.containsKey(name)) {
            List<Detection> detectionList = detections.get(name);
            while (index >= detectionList.size()) {
                detectionList.add(new Detection(name, index, new Pose3d()));
            }
            detectionList.set(index, detection);
        } else {
            detections.put(name, new ArrayList<>());
        }
    }

    public void setCount(String name, int count) {
        detectionCounts.put(name, count);
    }

    public boolean doesNameExist(String name) {
        return detections.containsKey(name);
    }

    public boolean doesDetectionExist(String name, int index) {
        if (doesNameExist(name)) {
            return index < detections.get(name).size();
        } else {
            return false;
        }
    }

    public Detection getDetection(String name, int index) {
        return detections.get(name).get(index);
    }

    public List<Detection> getAllDetectionsNamed(String name) {
        return detections.get(name);
    }

    public Collection<Detection> getAllDetections() {
        Collection<Detection> all = new ArrayList<Detection>();
        for (String name : detections.keySet()) {
            if (!detectionCounts.containsKey(name)) {
                continue;
            }
            for (int index = 0; index < Math.min(detectionCounts.get(name), detections.get(name).size()); index++) {
                all.add(detections.get(name).get(index));
            }
        }
        return all;
    }
}
