package frc.robot.ros.subscribers;

import java.util.HashMap;
import java.util.Map;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.ros.RosConversions;
import frc.robot.ros.RosTimeManager;
import frc.robot.ros.messages.Pose3dStamped;
import geometry_msgs.TransformStamped;
import tf2_msgs.TFMessage;

public class RosTfManager {
    private Map<Pair<String, String>, Pose3dStamped> tree = new HashMap<>();

    private final String topicName = "/tf_compact";
    private final Subscriber<TFMessage> subscriber;
    private final ConnectedNode connectedNode;
    
    public RosTfManager(ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        this.subscriber = this.connectedNode.newSubscriber(topicName, TFMessage._TYPE);
        this.subscriber.addMessageListener(message -> this.callback(message));
    }

    private void callback(TFMessage msg) {
        for (TransformStamped transform : msg.getTransforms()) {
            Pair<String, String> key = new Pair<String,String>(transform.getHeader().getFrameId(), transform.getChildFrameId());
            Rotation3d rotation = new Rotation3d(RosConversions.rosQuaternionToQuaternion(transform.getTransform().getRotation()));
            
            Pose3d pose = new Pose3d(
                transform.getTransform().getTranslation().getX(),
                transform.getTransform().getTranslation().getY(),
                transform.getTransform().getTranslation().getZ(),
                rotation
            );
            long localTime = RosTimeManager.asLocalTime(transform.getHeader().getStamp());
            tree.put(key, new Pose3dStamped(localTime, pose));
        }
    }

    public Pose3dStamped getTF(String parent_frame, String child_frame) {
        return tree.get(new Pair<String, String>(parent_frame, child_frame));
    }

    public boolean isTF(String parent_frame, String child_frame) {
        return tree.containsKey(new Pair<String, String>(parent_frame, child_frame));
    }
}
