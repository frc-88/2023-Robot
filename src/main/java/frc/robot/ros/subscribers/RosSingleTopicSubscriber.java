package frc.robot.ros.subscribers;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import edu.wpi.first.wpilibj.RobotController;

public abstract class RosSingleTopicSubscriber<RosMsg, ReturnMsg> {
    private final Subscriber<RosMsg> subscriber;
    private final ConnectedNode connectedNode;
    private final long message_timeout;
    private long timestamp = 0;
    protected static final long DEFAULT_MESSAGE_TIMEOUT = 500_000;
    
    public RosSingleTopicSubscriber(ConnectedNode connectedNode, String topicName, String topicType, long message_timeout) {
        this.connectedNode = connectedNode;
        this.message_timeout = message_timeout;
        this.subscriber = this.connectedNode.newSubscriber(topicName, topicType);
        this.subscriber.addMessageListener(message -> this.callback(message));
    }

    private void callback(RosMsg msg) {
        timestamp = getTime();
        messageCallback(msg);
    }

    protected abstract void messageCallback(RosMsg msg);

    protected long getTime() { 
        return RobotController.getFPGATime();
    }

    public boolean isActive() {
        return getTime() - timestamp > message_timeout;
    }

    public abstract ReturnMsg getData();
}
