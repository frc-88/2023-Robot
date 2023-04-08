package frc.robot.ros;

import org.ros.internal.message.RawMessage;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import geometry_msgs.Point;

public class RosConversions {
    public static void rpyToRosQuaternion(double roll, double pitch, double yaw, geometry_msgs.Quaternion out) {
        Quaternion quat = new Rotation3d(roll, pitch, yaw).getQuaternion();
        quaternionToRosQuaternion(quat, out);
    }

    public static geometry_msgs.Quaternion rpyToRosQuaternion(double roll, double pitch, double yaw) {
        Quaternion quat = new Rotation3d(roll, pitch, yaw).getQuaternion();
        return quaternionToRosQuaternion(quat);
    }

    public static void quaternionToRosQuaternion(Quaternion quat, geometry_msgs.Quaternion out) {
        out.setW(quat.getW());
        out.setX(quat.getX());
        out.setY(quat.getY());
        out.setZ(quat.getZ());
    }

    public static geometry_msgs.Quaternion quaternionToRosQuaternion(Quaternion quat) {
        return new geometry_msgs.Quaternion() {

            @Override
            public RawMessage toRawMessage() {
                return null;
            }

            @Override
            public double getW() {
                return quat.getW();
            }

            @Override
            public double getX() {
                return quat.getX();
            }

            @Override
            public double getY() {
                return quat.getY();
            }

            @Override
            public double getZ() {
                return quat.getZ();
            }

            @Override
            public void setW(double arg0) {
            }

            @Override
            public void setX(double arg0) {
            }

            @Override
            public void setY(double arg0) {
            }

            @Override
            public void setZ(double arg0) {
            }

        };
    }

    public static Quaternion rosQuaternionToQuaternion(geometry_msgs.Quaternion quat) {
        return new Quaternion(
                quat.getW(),
                quat.getX(),
                quat.getY(),
                quat.getZ());
    }

    public static Pose3d rosPose3dToPose3d(geometry_msgs.Pose ros_pose) {
        return new Pose3d(
                ros_pose.getPosition().getX(),
                ros_pose.getPosition().getY(),
                ros_pose.getPosition().getZ(),
                new Rotation3d(rosQuaternionToQuaternion(ros_pose.getOrientation())));
    }

    public static geometry_msgs.Pose pose3dToRosPose3d(Pose3d pose) {
        return new geometry_msgs.Pose() {

            @Override
            public RawMessage toRawMessage() {
                // TODO Auto-generated method stub
                return null;
            }

            @Override
            public geometry_msgs.Quaternion getOrientation() {
                return quaternionToRosQuaternion(pose.getRotation().getQuaternion());
            }

            @Override
            public Point getPosition() {
                return new Point() {

                    @Override
                    public RawMessage toRawMessage() {
                        // TODO Auto-generated method stub
                        return null;
                    }

                    @Override
                    public double getX() {
                        return pose.getX();
                    }

                    @Override
                    public double getY() {
                        return pose.getY();
                    }

                    @Override
                    public double getZ() {
                        return pose.getZ();
                    }

                    @Override
                    public void setX(double arg0) {
                    }

                    @Override
                    public void setY(double arg0) {
                    }

                    @Override
                    public void setZ(double arg0) {
                    }

                };
            }

            @Override
            public void setOrientation(geometry_msgs.Quaternion arg0) {
            }

            @Override
            public void setPosition(Point arg0) {
            }

        };
    }

    public static String getTeamColorName(DriverStation.Alliance team_color, boolean invert) {
        String team_name = "";
        if (team_color == Alliance.Red) {
            if (invert) {
                team_name = "blue";
            } else {
                team_name = "red";
            }
        } else if (team_color == Alliance.Blue) {
            if (invert) {
                team_name = "red";
            } else {
                team_name = "blue";
            }
        }
        return team_name;
    }

    public static String getTeamColorName(DriverStation.Alliance team_color) {
        return getTeamColorName(team_color, false);
    }

    public static String parseName(String waypointName) {
        String parsed = waypointName.replaceAll("<team>", getTeamColorName(DriverStation.getAlliance(), false));
        parsed = parsed.replaceAll("<!team>", getTeamColorName(DriverStation.getAlliance(), true));
        return parsed;
    }
}
