package frc.robot.ros.subscribers;

import java.util.HashMap;
import java.util.Map;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.ros.RosConversions;
import tj2_interfaces.Waypoint;
import tj2_interfaces.WaypointArray;

public class RosWaypointManager {
    private Map<String, Pose2d> waypoints = new HashMap<>();

    private final String topicName = "/tj2/waypoints";
    private final Subscriber<WaypointArray> subscriber;
    private final ConnectedNode connectedNode;
    private long timestamp = 0;
    protected static final long MESSAGE_TIMEOUT = 500_000;
    
    public RosWaypointManager(ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        this.subscriber = this.connectedNode.newSubscriber(topicName, WaypointArray._TYPE);
        this.subscriber.addMessageListener(message -> this.callback(message));
    }

    private void callback(WaypointArray msg) {
        timestamp = getTime();
        
        for (Waypoint waypoint : msg.getWaypoints()) {
            Rotation3d rotation = new Rotation3d(RosConversions.rosQuaternionToQuaternion(waypoint.getPose().getOrientation()));
            
            Pose2d pose = new Pose2d(
                waypoint.getPose().getPosition().getX(),
                waypoint.getPose().getPosition().getY(),
                rotation.toRotation2d()
            );
            waypoints.put(waypoint.getName(), pose);
        }
    }

    public Pose2d getWaypoint(String name) {
        return waypoints.get(name);
    }

    public boolean isWaypoint(String name) {
        return waypoints.containsKey(name);
    }

    public void setWaypoint(String name, Pose2d pose) {
        // TODO: add waypoint service
    }

    public boolean isActive() {
        return getTime() - timestamp > MESSAGE_TIMEOUT;
    }

    protected long getTime() { 
        return RobotController.getFPGATime();
    }

}
