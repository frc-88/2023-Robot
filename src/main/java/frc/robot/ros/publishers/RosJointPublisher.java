package frc.robot.ros.publishers;

import org.ros.node.ConnectedNode;

import std_msgs.Float64;

public class RosJointPublisher extends RosSingleTopicPublisher<Double, Float64> {

    public RosJointPublisher(ConnectedNode connectedNode, String jointName) {
        super(connectedNode, "/tj2/" + jointName, Float64._TYPE);
    }

    @Override
    protected Float64 convert(Double data) {
        Float64 msg = makeMsg();
        msg.setData(data);
        return msg;
    }
    
}
