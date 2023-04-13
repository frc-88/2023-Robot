package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.BotPoseProvider;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.ROSConversions;
import frc.team88.ros.conversions.TFListenerCompact;
import frc.team88.ros.conversions.Transform3dStamped;
import frc.team88.ros.messages.Time;
import frc.team88.ros.messages.geometry_msgs.Point;
import frc.team88.ros.messages.geometry_msgs.Pose;
import frc.team88.ros.messages.geometry_msgs.PoseWithCovariance;
import frc.team88.ros.messages.geometry_msgs.Quaternion;
import frc.team88.ros.messages.geometry_msgs.Twist;
import frc.team88.ros.messages.geometry_msgs.TwistWithCovariance;
import frc.team88.ros.messages.geometry_msgs.Vector3;
import frc.team88.ros.messages.nav_msgs.Odometry;
import frc.team88.ros.messages.std_msgs.Float64;
import frc.team88.ros.messages.std_msgs.Header;

public class ScorpionCoprocessorBridge extends SubsystemBase implements BotPoseProvider {
    private final SwerveDrive m_drive;
    private final ROSNetworkTablesBridge m_ros_interface;
    private final BridgeSubscriber<Twist> m_twistSub;
    private final BridgePublisher<Odometry> m_odomPub;
    private final BridgeSubscriber<Float64> m_pingSendSub;
    private final BridgePublisher<Float64> m_pingReturnPub;
    private final TFListenerCompact m_tfListenerCompact;

    private long prevPingTime = 0;
    private final long PING_TIMEOUT = 1_000_000;

    private long prevTagGlobalTime = 0;
    private final long TAG_GLOBAL_POSE_TIMEOUT = 250_000;

    public final String MAP_FRAME = "map";
    public final String ODOM_FRAME = "odom";
    public final String BASE_FRAME = "base_link";
    public final String TAG_FIELD_FRAME = "field";
    private final Odometry odomMsg = new Odometry(new Header(0, new Time(), ODOM_FRAME), BASE_FRAME,
            new PoseWithCovariance(new Pose(new Point(0, 0, 0), new Quaternion(0, 0, 0, 1)), new Double[] {
                    5e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 5e-2, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 5e-2, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 5e-2, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 5e-2, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 5e-2
            }),
            new TwistWithCovariance(new Twist(new Vector3(0, 0, 0), new Vector3(0, 0, 0)), new Double[] {
                    10e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 10e-2, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 10e-2, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 10e-2, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 10e-2, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 10e-2
            }));

    public ScorpionCoprocessorBridge(
            SwerveDrive drive) {
        NetworkTableInstance instance = NetworkTableInstance.create();
        instance.startClient3("bridge");
        instance.setServer(Constants.COPROCESSOR_ADDRESS, Constants.COPROCESSOR_PORT);
        m_ros_interface = new ROSNetworkTablesBridge(instance.getTable(""), Constants.COPROCESSOR_UPDATE_DELAY);

        m_twistSub = new BridgeSubscriber<>(m_ros_interface, "/tj2/cmd_vel",
                Twist.class);
        m_odomPub = new BridgePublisher<>(m_ros_interface, "/tj2/odom");
        m_pingSendSub = new BridgeSubscriber<>(m_ros_interface,
                "/tj2/ping_send",
                Float64.class);
        m_pingReturnPub = new BridgePublisher<>(m_ros_interface,
                "/tj2/ping_return");
        m_tfListenerCompact = new TFListenerCompact(m_ros_interface, "/tf_compact");

        m_drive = drive;
    }

    public BridgeSubscriber<Twist> getTwistSub() {
        return m_twistSub;
    }

    public TFListenerCompact getTFListener() {
        return m_tfListenerCompact;
    }

    private void checkPing() {
        Float64 ping;
        if ((ping = m_pingSendSub.receive()) != null) {
            prevPingTime = getTime();
            m_pingReturnPub.send(ping);
        }
    }

    private long getTime() {
        return RobotController.getFPGATime();
    }

    private void sendOdom() {
        Pose2d pose = m_drive.getOdometryPose();
        ChassisSpeeds velocity = m_drive.getChassisSpeeds();

        odomMsg.setHeader(m_odomPub.getHeader(ODOM_FRAME));
        odomMsg.getPose().setPose(ROSConversions.wpiToRosPose(new Pose3d(pose)));
        odomMsg.getTwist().getTwist()
                .setLinear(new Vector3(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, 0.0));
        odomMsg.getTwist().getTwist().setAngular(new Vector3(0.0, 0.0, velocity.omegaRadiansPerSecond));

        m_odomPub.send(odomMsg);
    }

    public boolean isConnected() {
        return prevPingTime != 0 && getTime() - prevPingTime < PING_TIMEOUT;
    }

    private boolean isBlue() {
        return DriverStation.getAlliance() == Alliance.Blue;
    }

    public Pose2d getTagGlobalPose() {
        if (m_tfListenerCompact.canTransform(TAG_FIELD_FRAME, BASE_FRAME)) {
            Transform3dStamped tfStamped = m_tfListenerCompact.lookupTransform(TAG_FIELD_FRAME, BASE_FRAME);
            prevTagGlobalTime = tfStamped.timestamp;
            Pose3d pose = new Pose3d(tfStamped.transform.getTranslation(), tfStamped.transform.getRotation());
            return pose.toPose2d();

        } else {
            return new Pose2d();
        }
    }

    public boolean isGlobalPoseActive() {
        return prevTagGlobalTime != 0 && getTime() - prevTagGlobalTime < TAG_GLOBAL_POSE_TIMEOUT;
    }

    public Pose2d getBotPose() {
        Pose2d transformPose;
        if (isBlue()) {
            transformPose = new Pose2d(8.27, 4.01, Rotation2d.fromDegrees(180));
        } else {
            transformPose = new Pose2d(-8.27, -4.01, new Rotation2d());
        }

        return getTagGlobalPose().relativeTo(transformPose);
    }

    @Override
    public void periodic() {
        super.periodic();
        sendOdom();
        checkPing();
        m_tfListenerCompact.update();
    }
}
