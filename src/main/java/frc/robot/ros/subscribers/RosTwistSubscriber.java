package frc.robot.ros.subscribers;

import org.ros.node.ConnectedNode;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import geometry_msgs.Twist;

public class RosTwistSubscriber extends RosSingleTopicSubscriber<Twist, ChassisSpeeds> {
    private final ChassisSpeeds command = new ChassisSpeeds();
    
    public RosTwistSubscriber(ConnectedNode connectedNode) {
        super(connectedNode, "/tj2/cmd_vel", Twist._TYPE, DEFAULT_MESSAGE_TIMEOUT);
    }

    protected void messageCallback(Twist msg) {
        command.vxMetersPerSecond = msg.getLinear().getX();
        command.vyMetersPerSecond = msg.getLinear().getY();
        command.omegaRadiansPerSecond = msg.getAngular().getZ();
    }

    public ChassisSpeeds getData() {
        return command;
    }
}
