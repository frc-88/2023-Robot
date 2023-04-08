package frc.robot.ros.publishers;

import org.ros.node.ConnectedNode;

import frc.robot.ros.Constants;
import frc.robot.ros.messages.FrcImu;
import frc.robot.ros.RosConversions;
import tj2_interfaces.NavX;

public class RosNavXPublisher extends RosSingleTopicPublisher<FrcImu, NavX> {
    public static final Constants.Frames imuFrameId = Constants.Frames.IMU;

    public RosNavXPublisher(ConnectedNode connectedNode) {
        super(connectedNode, "/tj2/imu", NavX._TYPE);
    }

    @Override
    protected NavX convert(FrcImu data) {
        NavX msg = makeMsg();
        msg.getHeader().setSeq(getSeq());
        msg.getHeader().setStamp(getRosTime());
        msg.getHeader().setFrameId(imuFrameId.value);

        RosConversions.rpyToRosQuaternion(
                data.roll, data.pitch, data.yaw,
                msg.getOrientation());
        msg.getAngularVelocity().setZ(data.yaw_rate);
        msg.getLinearAcceleration().setX(data.accel_x);
        msg.getLinearAcceleration().setY(data.accel_y);
        msg.getLinearAcceleration().setZ(data.accel_z);

        return msg;
    }

}
