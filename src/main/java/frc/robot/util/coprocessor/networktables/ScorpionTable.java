package frc.robot.util.coprocessor.networktables;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.coprocessor.ChassisInterface;
import frc.robot.util.coprocessor.MessageTimer;
import frc.robot.util.preferenceconstants.StringPreferenceConstant;


public class ScorpionTable extends CoprocessorTable {
    private SwerveDrive swerve;
    private AHRS imu;
    protected Pose2d tagGlobalPose = new Pose2d();
    protected MessageTimer tagGlobalPoseTimer = new MessageTimer(DEFAULT_MESSAGE_TIMEOUT);
    private DoubleArraySubscriber tagGlobalPoseSub;
    private StringPreferenceConstant coprocessorIPAddress =
        new StringPreferenceConstant("Jetson IP Address", "10.0.88.44");

    private final double kGravity = 9.81;

    public ScorpionTable(SwerveDrive swerve, AHRS imu, int port, double updateInterval) {
        super((ChassisInterface)swerve, updateInterval);
        start(coprocessorIPAddress.getValue(), port);
        this.swerve = swerve;
        this.imu = imu;

        tagGlobalPoseSub = rootTable.getDoubleArrayTopic("tag_global").subscribe(new double []{0.0, 0.0, 0.0, 0.0}, PubSubOption.sendAll(true), PubSubOption.periodic(this.updateInterval));
    }

    // ---
    // Tag global pose
    // ---

    public Pose2d getTagGlobalPose() {
        return tagGlobalPose;
    }

    public Pose2d getBotPose() {
        Pose2d transformPose;
        if (DriverStation.getAlliance() == Alliance.Blue) {
            transformPose = new Pose2d(8.27,4.01, Rotation2d.fromDegrees(180));
        } else  {
            transformPose = new Pose2d(-8.27,-4.01, new Rotation2d());
        }

        return getTagGlobalPose().relativeTo(transformPose);
    }

    public InstantCommand rosLocalize(SwerveDrive drive) {
        return new InstantCommand (
          () -> {
            drive.resetPosition(getBotPose());},
          drive);
    }

    public boolean isTagGlobalPoseActive() {
        return tagGlobalPoseTimer.isActive();
    }

    private void updateTagGlobalPose() {
        TimestampedDoubleArray pose = tagGlobalPoseSub.getAtomic();
        if (pose.timestamp == 0.0) {
            return;
        }
        if (pose.value.length != 5) {
            System.out.println("Warning: Received tag global pose is not of length 5. Ignoring.");
            return;
        }
        // index 0 is timestamp
        double x = pose.value[1];
        double y = pose.value[2];
        // double z = pose.value[3];
        double theta = pose.value[4];
        tagGlobalPose = new Pose2d(x, y, new Rotation2d(theta));
        tagGlobalPoseTimer.reset();
    }

    @Override
    public void update() {
        super.update();
        sendImu(
            Units.degreesToRadians(imu.getRoll()),
            Units.degreesToRadians(imu.getPitch()),
            Units.degreesToRadians(-imu.getYaw()),
            Units.degreesToRadians(-imu.getRate()),
            imu.getWorldLinearAccelX() * kGravity,
            imu.getWorldLinearAccelY() * kGravity
        );
        updateTagGlobalPose();

        Pose2d botPose = getTagGlobalPose();
        SmartDashboard.putNumber("ROS:X",botPose.getX());
        SmartDashboard.putNumber("ROS:Y",botPose.getY());
        SmartDashboard.putNumber("ROS:Rotation",botPose.getRotation().getDegrees());

        Pose2d botPoseTransform = getBotPose();
        SmartDashboard.putNumber("nix:X",botPoseTransform.getX());
        SmartDashboard.putNumber("nix:Y",botPoseTransform.getY());
        SmartDashboard.putNumber("nix:Rotation",botPoseTransform.getRotation().getDegrees());

    }

    public void updateSlow() {
        setNoGoZones(new String[] {"<!team>_score_zone", "<!team>_zone", "<!team>_safe_zone"});
    }
}
