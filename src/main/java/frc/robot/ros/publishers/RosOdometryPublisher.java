package frc.robot.ros.publishers;

import org.ros.node.ConnectedNode;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.ros.Constants;
import frc.robot.ros.RosConversions;
import frc.robot.ros.messages.Odometry2d;
import nav_msgs.Odometry;


public class RosOdometryPublisher extends RosSingleTopicPublisher<Odometry2d, Odometry> {
    private final double[] poseCovariance = new double[] {
        5e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 5e-2, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 5e-2, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 5e-2, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 5e-2, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 5e-2
    };
    private final double[] twistCovariance = new double[] {
        10e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 10e-2, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 10e-2, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 10e-2, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 10e-2, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 10e-2
    };
    public static final Constants.Frames odomFrameId = Constants.Frames.ODOM;
    public static final Constants.Frames baseLinkFrameId = Constants.Frames.BASE_LINK;


    public RosOdometryPublisher(ConnectedNode connectedNode) {
        super(connectedNode, "/tj2/odom", Odometry._TYPE);
    }

    @Override
    protected Odometry convert(Odometry2d data) {
        Pose3d pose3d = new Pose3d(data.pose);

        Odometry odomMsg = makeMsg();
        odomMsg.getHeader().setSeq(getSeq());
        odomMsg.getHeader().setStamp(getRosTime());
        odomMsg.getHeader().setFrameId(odomFrameId.value);
        odomMsg.setChildFrameId(baseLinkFrameId.value);

        odomMsg.getPose().setCovariance(poseCovariance);
        odomMsg.getPose().getPose().getPosition().setX(pose3d.getX());
        odomMsg.getPose().getPose().getPosition().setY(pose3d.getY());
        odomMsg.getPose().getPose().getPosition().setZ(pose3d.getZ());
        RosConversions.quaternionToRosQuaternion(
            pose3d.getRotation().getQuaternion(),
            odomMsg.getPose().getPose().getOrientation()
        );
        
        odomMsg.getTwist().setCovariance(twistCovariance);
        odomMsg.getTwist().getTwist().getLinear().setX(data.twist.vxMetersPerSecond);
        odomMsg.getTwist().getTwist().getLinear().setY(data.twist.vyMetersPerSecond);
        odomMsg.getTwist().getTwist().getLinear().setZ(0.0);
        odomMsg.getTwist().getTwist().getAngular().setX(0.0);
        odomMsg.getTwist().getTwist().getAngular().setY(0.0);
        odomMsg.getTwist().getTwist().getAngular().setZ(data.twist.omegaRadiansPerSecond);

        return odomMsg;
    }
}
