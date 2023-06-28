package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ros.messages.tj2_interfaces.NavX;
import frc.robot.util.BotPoseProvider;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.ROSConversions;
import frc.team88.ros.conversions.TFListenerCompact;
import frc.team88.ros.conversions.Transform3dStamped;
import frc.team88.ros.messages.TimePrimitive;
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
import frc.team88.ros.messages.std_msgs.Time;

public class ScorpionCoprocessorBridge extends SubsystemBase implements BotPoseProvider {
    private final SwerveDrive m_drive;
    private final AHRS m_imu;
    private final ROSNetworkTablesBridge m_ros_interface;

    private final BridgeSubscriber<Float64> m_pingSendSub;

    private final BridgePublisher<Odometry> m_odomPub;
    private final BridgePublisher<NavX> m_imuPub;
    private final BridgePublisher<Float64> m_pingReturnPub;
    private final BridgePublisher<Time> m_startBagPub;

    private final TFListenerCompact m_tfListenerCompact;

    private final double GRAVITY = 9.81;

    private long prevPingTime = 0;
    private final long PING_TIMEOUT = 1_000_000;

    private long prevTagGlobalTime = 0;
    private final long TAG_GLOBAL_POSE_TIMEOUT = 250_000;

    private final double UPDATE_INTERVAL = 0.02;

    public final String MAP_FRAME = "map";
    public final String ODOM_FRAME = "odom";
    public final String BASE_FRAME = "base_link";
    public static final String IMU_FRAME = "imu";
    public final String TAG_FIELD_FRAME = "field";
    private final Odometry m_odomMsg = new Odometry(new Header(0, new TimePrimitive(), ODOM_FRAME), BASE_FRAME,
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

    public ScorpionCoprocessorBridge(SwerveDrive drive) {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();

        m_ros_interface = new ROSNetworkTablesBridge(instance.getTable(""), UPDATE_INTERVAL);

        m_pingSendSub = new BridgeSubscriber<>(m_ros_interface, "ping_send", Float64.class);

        m_odomPub = new BridgePublisher<>(m_ros_interface, "odom");
        m_imuPub = new BridgePublisher<>(m_ros_interface, "imu");
        m_pingReturnPub = new BridgePublisher<>(m_ros_interface, "ping_return");
        m_startBagPub = new BridgePublisher<>(m_ros_interface, "start_bag");

        m_tfListenerCompact = new TFListenerCompact(m_ros_interface, "/tf_compact");

        m_drive = drive;
        m_imu = drive.getNavX();
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

        m_odomMsg.setHeader(m_odomPub.getHeader(ODOM_FRAME));
        m_odomMsg.getPose().setPose(ROSConversions.wpiToRosPose(new Pose3d(pose)));
        m_odomMsg.getTwist().getTwist()
                .setLinear(new Vector3(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, 0.0));
        m_odomMsg.getTwist().getTwist().setAngular(new Vector3(0.0, 0.0, velocity.omegaRadiansPerSecond));

        m_odomPub.send(m_odomMsg);
    }

    private void sendImu() {
        double roll = Units.degreesToRadians(m_imu.getRoll());
        double pitch = Units.degreesToRadians(m_imu.getPitch());
        double yaw = Units.degreesToRadians(m_imu.getYaw());
        double yaw_rate = Units.degreesToRadians(-m_imu.getRate());
        double accel_x = m_imu.getWorldLinearAccelX() * GRAVITY;
        double accel_y = m_imu.getWorldLinearAccelY() * GRAVITY;
        double accel_z = m_imu.getWorldLinearAccelZ() * GRAVITY;
        m_imuPub.send(new NavX(m_imuPub.getHeader(IMU_FRAME),
                ROSConversions.wpiToRosRotation(new Rotation3d(roll, pitch, yaw)),
                new Vector3(0.0, 0.0, yaw_rate),
                new Vector3(accel_x, accel_y, accel_z)));
    }

    public void startBag() {
        TimePrimitive now = m_startBagPub.getNow();
        m_startBagPub.send(new Time(now));
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
        sendImu();
        checkPing();
        m_tfListenerCompact.update();
    }
}
