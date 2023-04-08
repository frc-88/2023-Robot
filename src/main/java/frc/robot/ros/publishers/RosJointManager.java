package frc.robot.ros.publishers;

import java.util.HashMap;
import java.util.Map;

import org.ros.node.ConnectedNode;

import frc.robot.ros.Constants.Joints;

public class RosJointManager {
    private final ConnectedNode connectedNode;
    private Map<String, RosJointPublisher> joints = new HashMap<>();
    private Map<String, Double> values = new HashMap<>();

    public RosJointManager(ConnectedNode node) {
        connectedNode = node;
    }

    public void addFromEnum() {
        for (Joints joint_id : Joints.values()) {
            addJoint(joint_id.value);
        }
    }
    
    public void addJoint(String name) {
        joints.put(name, new RosJointPublisher(connectedNode, name));
        values.put(name, 0.0);
    }

    public void setJoint(String name, double value) {
        values.put(name, value);
    }

    public void publish() {
        for (String name : joints.keySet()) {
            joints.get(name).publish(values.get(name));
        }
    }
}
