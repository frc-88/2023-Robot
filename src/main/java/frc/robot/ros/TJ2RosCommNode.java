package frc.robot.ros;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.ros.messages.FrcGameObjectsStamped;
import frc.robot.ros.messages.FrcGridTrackingMarkers;
import frc.robot.ros.messages.FrcMatch;
import frc.robot.ros.messages.FrcMatchPeriod;
import frc.robot.ros.messages.Odometry2d;
import frc.robot.ros.messages.Pose2dWithCovariance;
import frc.robot.ros.messages.Pose3dStamped;
import frc.robot.ros.publishers.RosGridTrackingMarkerPublisher;
import frc.robot.ros.publishers.RosInitialPosePublisher;
import frc.robot.ros.publishers.RosJointManager;
import frc.robot.ros.publishers.RosMatchEventPublisher;
import frc.robot.ros.publishers.RosMatchPublisher;
import frc.robot.ros.publishers.RosNavXPublisher;
import frc.robot.ros.publishers.RosOdometryPublisher;
import frc.robot.ros.subscribers.RosGameObjectsStampedSubscriber;
import frc.robot.ros.subscribers.RosTfManager;
import frc.robot.ros.subscribers.RosTwistSubscriber;
import frc.robot.ros.subscribers.RosWaypointManager;
import frc.robot.subsystems.DriveSubsystem;

public final class TJ2RosCommNode extends AbstractNodeMain {
    private final DriveSubsystem drive;
    private final String rosNodeName;
    private final long UPDATE_DELAY = 20;
    private final int SLOW_UPDATE_SKIP_COUNT = 5; // How many fast updates to run for each slow update
    private int slow_update_counter = 0;
    private FrcMatchPeriod match_phase = FrcMatchPeriod.DISABLED;

    private RosOdometryPublisher odometry = null;
    private RosInitialPosePublisher initial_pose = null;
    private RosMatchPublisher match = null;
    private RosMatchEventPublisher match_event = null;
    private RosNavXPublisher imu = null;
    private RosGridTrackingMarkerPublisher grid_tracking_markers = null;

    private RosTwistSubscriber twist = null;
    private RosGameObjectsStampedSubscriber game_objects = null;

    private RosPingManager ping = null;
    private RosJointManager joints = null;
    private RosWaypointManager waypoints = null;
    private RosTfManager tf_tree = null;
    // TODO: zones manager

    /**
     * @param rosTopicName the name of the topic to publish
     * @param rosNodeName  the name of the node
     */
    public TJ2RosCommNode(String rosNodeName, DriveSubsystem drive) {
        this.rosNodeName = rosNodeName;
        this.drive = drive;
    }

    @Override
    public final GraphName getDefaultNodeName() {
        return GraphName.of(this.rosNodeName);
    }
    // ---
    // Getters
    // ---

    public ChassisSpeeds getTwist() {
        return twist.getData();
    }

    public RosPingManager getPingManager() {
        return ping;
    }

    public RosWaypointManager getWaypointsManager() {
        return waypoints;
    }

    public Pose3dStamped getTF(String parent_frame_id, String child_frame_id) {
        return tf_tree.getTF(parent_frame_id, child_frame_id);
    }

    public FrcGameObjectsStamped getGameObjects() {
        return game_objects.getData();
    }

    // ---
    // Setters
    // ---

    public void publishInitialPose(Pose2dWithCovariance pose) {
        initial_pose.publish(pose);
    }

    private void publishMatch() {
        Alliance alliance = DriverStation.getAlliance();

        match.publish(new FrcMatch(
                DriverStation.getMatchTime(),
                RosConversions.getTeamColorName(alliance),
                DriverStation.getLocation(),
                match_phase));
    }

    public void publishMatchEvent(FrcMatchPeriod match_phase) {
        if (match_event == null) {
            return;
        }
        this.match_phase = match_phase;
        match_event.publish(match_phase);
    }

    public void setJointPosition(String name, double position) {
        joints.setJoint(name, position);
    }

    public void publishGridTrackingMarkers(FrcGridTrackingMarkers markers) {
        grid_tracking_markers.publish(markers);
    }

    // ---
    // Updates
    // ---

    private void fastUpdate() {
        odometry.publish(new Odometry2d(
                drive.getSwerve().getOdometryPose(),
                drive.getSwerve().getChassisSpeeds()));
        imu.publish(drive.getImu().getMessage());
    }

    private void slowUpdate() {
        SwerveModuleState[] states = drive.getSwerve().getModuleStates();
        for (int index = 0; index < drive.getSwerve().getNumModules(); index++) {
            double angle = states[index].angle.getRadians();
            String name = String.format("base_link_to_wheel_%d_joint", index);
            setJointPosition(name, angle);
        }
        publishMatch();
        joints.publish();
    }

    @Override
    public final void onStart(final ConnectedNode connectedNode) {
        odometry = new RosOdometryPublisher(connectedNode);
        initial_pose = new RosInitialPosePublisher(connectedNode);
        match = new RosMatchPublisher(connectedNode);
        imu = new RosNavXPublisher(connectedNode);
        match_event = new RosMatchEventPublisher(connectedNode);
        grid_tracking_markers = new RosGridTrackingMarkerPublisher(connectedNode);

        twist = new RosTwistSubscriber(connectedNode);
        game_objects = new RosGameObjectsStampedSubscriber(connectedNode);

        ping = new RosPingManager(connectedNode);
        joints = new RosJointManager(connectedNode);
        joints.addFromEnum();
        waypoints = new RosWaypointManager(connectedNode);
        tf_tree = new RosTfManager(connectedNode);

        connectedNode.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                fastUpdate();
                if (slow_update_counter >= SLOW_UPDATE_SKIP_COUNT) {
                    slow_update_counter = 0;
                    slowUpdate();
                }
                Thread.sleep(UPDATE_DELAY);
                slow_update_counter++;
            }
        });
    }
}
