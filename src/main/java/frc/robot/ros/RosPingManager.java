package frc.robot.ros;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import std_msgs.Float64;

public class RosPingManager {
    private final Publisher<Float64> ping_return_pub;
    private final Subscriber<Float64> ping_receive_sub;
    private final ConnectedNode connectedNode;
    private final String pingReturnTopicName = "/tj2/ping_return";
    private final String pingReceiveTopicName = "/tj2/ping_send";
    
    public RosPingManager(ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        ping_return_pub = this.connectedNode.newPublisher(pingReturnTopicName, Float64._TYPE);
        ping_receive_sub = this.connectedNode.newSubscriber(pingReceiveTopicName, Float64._TYPE);
        ping_receive_sub.addMessageListener(message -> this.pingReceiveCallback(message));
    }

    private void pingReceiveCallback(Float64 msg) {
        RosTimeManager.sync(msg.getData());
        ping_return_pub.publish(msg);
    }
}
