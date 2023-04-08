package frc.robot.ros.subscribers;

import org.ros.node.ConnectedNode;

import frc.robot.ros.RosConversions;
import frc.robot.ros.RosTimeManager;
import frc.robot.ros.messages.FrcGameObject;
import frc.robot.ros.messages.FrcGameObjectsStamped;
import tj2_interfaces.GameObject;
import tj2_interfaces.GameObjectsStamped;

public class RosGameObjectsStampedSubscriber
        extends RosSingleTopicSubscriber<GameObjectsStamped, FrcGameObjectsStamped> {
    private FrcGameObjectsStamped data = new FrcGameObjectsStamped();

    public RosGameObjectsStampedSubscriber(ConnectedNode connectedNode) {
        super(connectedNode, "/tj2/detections", GameObjectsStamped._TYPE, DEFAULT_MESSAGE_TIMEOUT);
    }

    @Override
    protected void messageCallback(GameObjectsStamped msg) {
        long localTime = RosTimeManager.asLocalTime(msg.getHeader().getStamp());
        data = new FrcGameObjectsStamped();
        data.stamp = localTime;
        for (GameObject obj : msg.getObjects()) {
            data.objects.add(new FrcGameObject(
                    RosConversions.rosPose3dToPose3d(obj.getPose()),
                    obj.getLabel(),
                    obj.getObjectIndex(),
                    obj.getClassIndex(),
                    obj.getConfidence()));
        }
    }

    @Override
    public FrcGameObjectsStamped getData() {
        return data;
    }
}
