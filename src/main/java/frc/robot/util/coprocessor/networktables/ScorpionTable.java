package frc.robot.util.coprocessor.networktables;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
import frc.robot.util.BotPoseProvider;
import frc.robot.util.coprocessor.ChassisInterface;
import frc.robot.util.coprocessor.MessageTimer;


public class ScorpionTable extends CoprocessorTable implements BotPoseProvider {
    private SwerveDrive swerve;
    private AHRS imu;
    protected Pose2d tagGlobalPose = new Pose2d();
    protected MessageTimer tagGlobalPoseTimer = new MessageTimer(DEFAULT_MESSAGE_TIMEOUT);
    private DoubleArraySubscriber tagGlobalPoseSub;

    private Debouncer inCommunityDebouncer = new Debouncer(.75, DebounceType.kRising);

    private final double kGravity = 9.81;

    public ScorpionTable(SwerveDrive swerve, AHRS imu, String address, int port, double updateInterval) {
        super((ChassisInterface)swerve, address, port, updateInterval);
        this.swerve = swerve;
        this.imu = imu;

        tagGlobalPoseSub = rootTable.getDoubleArrayTopic("tag_global").subscribe(new double []{0.0, 0.0, 0.0, 0.0}, PubSubOption.sendAll(true), PubSubOption.periodic(this.updateInterval));
    }

    // ---
    // Tag global pose
    // ---

    // Red is to left, origin at center of field
    public Pose2d getTagGlobalPose() {
        return tagGlobalPose;
    }

    public Pose2d getTagGlobalPoseInches() {
        return tagGlobalPose.times(39.3701);
    }

    // Origin is at the right corner near the driver station from the persepective of the driver
    @Override
    public Pose2d getBotPose() {
        Pose2d transformPose;
        if (isBlue()) {
            transformPose = new Pose2d(8.27,4.01, Rotation2d.fromDegrees(180));
        } else  {
            transformPose = new Pose2d(-8.27,-4.01, new Rotation2d());
        }

        return getTagGlobalPose().relativeTo(transformPose);
    }

    public Pose2d getBotPoseInches() {
        return getBotPose().times(39.3701);
    }

    public InstantCommand rosLocalize(SwerveDrive drive) {
        return new InstantCommand (
          () -> {
            drive.resetPosition(getBotPose());},
          drive);
    }

    @Override
    public boolean isConnected() {
        return tagGlobalPoseTimer.isActive();
    }

    public boolean isInCommunity(Pose2d pos) {
        pos = pos.times(39.3701);
        double shortXLimit = 106;
        double longXLimit = 164;
        double communityYStartRed = 100;
        double communityYBumpRed = 160;
        double communityYStartBlue = switchYAlliance(communityYStartRed);
        double communityYBumpBlue = switchYAlliance(communityYBumpRed);

        boolean result;
        if (isBlue()) {
            result = pos.getY() < communityYStartBlue
                    && (pos.getX() < shortXLimit
                        || (pos.getY() < communityYBumpBlue && pos.getX() < longXLimit));
        } else {
            result = pos.getY() > communityYStartRed
                    && (pos.getX() < shortXLimit
                        || (pos.getY() > communityYBumpRed && pos.getX() < longXLimit));
        }

        return inCommunityDebouncer.calculate(result);
    }

    public static double switchYAlliance(double y) {
        return 27.*12. - y;
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
        swerve.addVisionPoseUpdate(tagGlobalPose);
        tagGlobalPoseTimer.reset();
    }

    private boolean isBlue() {
        return DriverStation.getAlliance() == Alliance.Blue;
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

        Pose2d botPose = getBotPose();
        SmartDashboard.putNumber("ROS:X",botPose.getX());
        SmartDashboard.putNumber("ROS:Y",botPose.getY());
        SmartDashboard.putNumber("ROS:Rotation",botPose.getRotation().getDegrees());
        SmartDashboard.putBoolean("ROS:TagGlobalPoseActive", isConnected());
    }

    public void updateSlow() {
        setNoGoZones(new String[] {"<!team>_score_zone", "<!team>_zone", "<!team>_safe_zone"});
    }
}
