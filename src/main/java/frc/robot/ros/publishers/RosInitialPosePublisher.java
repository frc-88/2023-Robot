package frc.robot.ros.publishers;

import org.ros.node.ConnectedNode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import frc.robot.ros.Constants;
import frc.robot.ros.messages.Pose2dWithCovariance;
import geometry_msgs.PoseWithCovarianceStamped;


public class RosInitialPosePublisher extends RosSingleTopicPublisher<Pose2dWithCovariance, PoseWithCovarianceStamped> {
    public static final Constants.Frames mapFrameId = Constants.Frames.MAP;

    public RosInitialPosePublisher(ConnectedNode connectedNode) {
        super(connectedNode, "/initialpose", PoseWithCovarianceStamped._TYPE);
    }

    @Override
    protected PoseWithCovarianceStamped convert(Pose2dWithCovariance data) {
        Pose3d pose3d = new Pose3d(data.pose);
        Quaternion quat = pose3d.getRotation().getQuaternion();

        PoseWithCovarianceStamped poseMsg = makeMsg();
        poseMsg.getHeader().setSeq(getSeq());
        poseMsg.getHeader().setStamp(getRosTime());
        poseMsg.getHeader().setFrameId(mapFrameId.value);

        double cx = data.covariances[0];
        double cy = data.covariances[1];
        double ct = data.covariances[2];
        poseMsg.getPose().setCovariance(new double[] {
            cx, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, cy, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, ct
        });
        poseMsg.getPose().getPose().getPosition().setX(pose3d.getX());
        poseMsg.getPose().getPose().getPosition().setY(pose3d.getY());
        poseMsg.getPose().getPose().getPosition().setZ(pose3d.getZ());
        poseMsg.getPose().getPose().getOrientation().setW(quat.getW());
        poseMsg.getPose().getPose().getOrientation().setX(quat.getX());
        poseMsg.getPose().getPose().getOrientation().setY(quat.getY());
        poseMsg.getPose().getPose().getOrientation().setZ(quat.getZ());
        
        return poseMsg;
    }
}
