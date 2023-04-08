package frc.robot.ros.publishers;

import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import frc.robot.ros.RosTimeManager;

public abstract class RosSingleTopicPublisher<SendMsg, RosMsg> {
    protected final Publisher<RosMsg> publisher;
    protected final ConnectedNode connectedNode;
    private int seq = 0;

    public RosSingleTopicPublisher(ConnectedNode connectedNode, String topicName, String topicType) {
        this.connectedNode = connectedNode;
        publisher = this.connectedNode.newPublisher(topicName, topicType);
    }

    public void publish(SendMsg data) {
        RosMsg msg = convert(data);
        publisher.publish(msg);
    }

    protected RosMsg makeMsg() {
        return this.publisher.newMessage();
    }

    protected Time getRosTime() {
        return RosTimeManager.getTime();
    }

    protected int getSeq() {
        return seq++;
    }

    abstract protected RosMsg convert(SendMsg data);
}
