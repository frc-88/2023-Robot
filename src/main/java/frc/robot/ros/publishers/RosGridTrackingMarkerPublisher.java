package frc.robot.ros.publishers;

import java.util.ArrayList;
import java.util.List;

import org.ros.internal.message.RawMessage;
import org.ros.message.Duration;
import org.ros.message.Time;
import org.ros.node.ConnectedNode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.grid_tracking.networktables.GameObject;
import frc.robot.grid_tracking.networktables.GridZone;
import frc.robot.ros.RosConversions;
import frc.robot.ros.RosTimeManager;
import frc.robot.ros.messages.FrcColorRGBA;
import frc.robot.ros.messages.FrcGridTrackingMarkers;
import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Vector3;
import std_msgs.ColorRGBA;
import std_msgs.Header;
import visualization_msgs.Marker;
import visualization_msgs.MarkerArray;

public class RosGridTrackingMarkerPublisher extends RosSingleTopicPublisher<FrcGridTrackingMarkers, MarkerArray> {
    private long prev_grid_publish_time = 0;
    private final long GRID_PUBLISH_INTERVAL = 500_000;
    private final double GRID_ZONE_LIFETIME = 1.0;
    private final double GRID_OBJ_LIFETIME = 0.1;
    private final double GRID_OBJ_RADIUS = 0.05;
    private final double FIELD_MIDPOINT = 27. * 12.;
    private int zone_id_counter = 0;
    private int obj_id_counter = 0;

    public RosGridTrackingMarkerPublisher(ConnectedNode connectedNode) {
        super(connectedNode, "/tj2/grid_tracking/markers", MarkerArray._TYPE);
    }

    @Override
    protected MarkerArray convert(FrcGridTrackingMarkers data) {
        zone_id_counter = 0;
        obj_id_counter = 0;
        MarkerArray markers = makeMsg();
        addGridObjects(markers, data);
        addGridZones(markers, data);
        return markers;
    }

    private void addGridZones(MarkerArray markers, FrcGridTrackingMarkers data) {
        long grid_publish_time = RosTimeManager.getLocalTime();
        if (grid_publish_time - prev_grid_publish_time < GRID_PUBLISH_INTERVAL) {
            return;
        }

        double checkRadius = Units.inchesToMeters(data.checkRadius);

        for (GridZone zone : data.grid_zones) {
            FrcColorRGBA color;
            if (zone.getX() < FIELD_MIDPOINT) {
                color = new FrcColorRGBA(1.0f, 0.0f, 0.0f, 1.0f);
            } else {
                color = new FrcColorRGBA(0.0f, 0.0f, 1.0f, 1.0f);
            }
            if (zone.filled) {
                color = new FrcColorRGBA(0.0f, 0.0f, 0.0f, 1.0f);
            }
            markers.getMarkers().add(
                    makeMarker(
                            new Pose3d(
                                    Units.inchesToMeters(zone.getX()),
                                    Units.inchesToMeters(zone.getY()),
                                    Units.inchesToMeters(zone.getZ()),
                                    new Rotation3d()),
                            Marker.SPHERE,
                            zone_id_counter++,
                            "grid_zone",
                            GRID_ZONE_LIFETIME,
                            color,
                            checkRadius,
                            checkRadius,
                            checkRadius,
                            ""));
        }
    }

    private void addGridObjects(MarkerArray markers, FrcGridTrackingMarkers data) {
        for (GameObject obj : data.grid_objects) {

            FrcColorRGBA color;
            if (obj.getName().equals("cone")) {
                color = new FrcColorRGBA(220 / 255, 174 / 255, 51 / 255, 1.0f);
            } else if (obj.getName().equals("cube")) {
                color = new FrcColorRGBA(71 / 255, 58 / 255, 164 / 255, 1.0f);
            } else {
                color = new FrcColorRGBA(0.5f, 0.5f, 0.5f, 1.0f);
            }
            markers.getMarkers().add(
                    makeMarker(
                            new Pose3d(
                                    Units.inchesToMeters(obj.getX()),
                                    Units.inchesToMeters(obj.getY()),
                                    Units.inchesToMeters(obj.getZ()),
                                    new Rotation3d()),
                            Marker.SPHERE,
                            obj_id_counter++,
                            "grid_obj",
                            GRID_OBJ_LIFETIME,
                            color,
                            GRID_OBJ_RADIUS,
                            GRID_OBJ_RADIUS,
                            GRID_OBJ_RADIUS,
                            ""));
        }
    }

    private Marker makeMarker(
            Pose3d pose,
            int marker_type,
            int id,
            String namespace,
            double lifetime_seconds,
            FrcColorRGBA color,
            double scale_x,
            double scale_y,
            double scale_z,
            String text) {
        return new Marker() {

            @Override
            public RawMessage toRawMessage() {
                // TODO Auto-generated method stub
                return null;
            }

            @Override
            public int getAction() {
                return Marker.ADD;
            }

            @Override
            public ColorRGBA getColor() {
                return new ColorRGBA() {

                    @Override
                    public RawMessage toRawMessage() {
                        // TODO Auto-generated method stub
                        return null;
                    }

                    @Override
                    public float getA() {
                        return color.a;
                    }

                    @Override
                    public float getB() {
                        return color.b;
                    }

                    @Override
                    public float getG() {
                        return color.g;
                    }

                    @Override
                    public float getR() {
                        return color.r;
                    }

                    @Override
                    public void setA(float arg0) {
                    }

                    @Override
                    public void setB(float arg0) {
                    }

                    @Override
                    public void setG(float arg0) {
                    }

                    @Override
                    public void setR(float arg0) {
                    }

                };
            }

            @Override
            public List<ColorRGBA> getColors() {
                return new ArrayList<>();
            }

            @Override
            public boolean getFrameLocked() {
                return false;
            }

            @Override
            public Header getHeader() {
                // TODO Auto-generated method stub
                return new Header() {

                    @Override
                    public RawMessage toRawMessage() {
                        // TODO Auto-generated method stub
                        return null;
                    }

                    @Override
                    public String getFrameId() {
                        return "field";
                    }

                    @Override
                    public int getSeq() {
                        return this.getSeq();
                    }

                    @Override
                    public Time getStamp() {
                        return RosTimeManager.getTime();
                    }

                    @Override
                    public void setFrameId(String arg0) {
                    }

                    @Override
                    public void setSeq(int arg0) {
                    }

                    @Override
                    public void setStamp(Time arg0) {
                    }

                };
            }

            @Override
            public int getId() {
                return id;
            }

            @Override
            public Duration getLifetime() {
                return new Duration(lifetime_seconds);
            }

            @Override
            public String getMeshResource() {
                return "";
            }

            @Override
            public boolean getMeshUseEmbeddedMaterials() {
                return false;
            }

            @Override
            public String getNs() {
                return namespace;
            }

            @Override
            public List<Point> getPoints() {
                return new ArrayList<>();
            }

            @Override
            public Pose getPose() {
                return RosConversions.pose3dToRosPose3d(pose);
            }

            @Override
            public Vector3 getScale() {
                return new Vector3() {

                    @Override
                    public RawMessage toRawMessage() {
                        // TODO Auto-generated method stub
                        return null;
                    }

                    @Override
                    public double getX() {
                        return scale_x;
                    }

                    @Override
                    public double getY() {
                        return scale_y;
                    }

                    @Override
                    public double getZ() {
                        return scale_z;
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
            public String getText() {
                return text;
            }

            @Override
            public int getType() {
                return marker_type;
            }

            @Override
            public void setAction(int arg0) {
            }

            @Override
            public void setColor(ColorRGBA arg0) {
                this.getColor().setA(arg0.getA());
                this.getColor().setB(arg0.getB());
                this.getColor().setG(arg0.getG());
                this.getColor().setR(arg0.getR());
            }

            @Override
            public void setColors(List<ColorRGBA> arg0) {
            }

            @Override
            public void setFrameLocked(boolean arg0) {
            }

            @Override
            public void setHeader(Header arg0) {
            }

            @Override
            public void setId(int arg0) {
            }

            @Override
            public void setLifetime(Duration arg0) {
            }

            @Override
            public void setMeshResource(String arg0) {
            }

            @Override
            public void setMeshUseEmbeddedMaterials(boolean arg0) {
            }

            @Override
            public void setNs(String arg0) {
            }

            @Override
            public void setPoints(List<Point> arg0) {
            }

            @Override
            public void setPose(Pose arg0) {
            }

            @Override
            public void setScale(Vector3 arg0) {
            }

            @Override
            public void setText(String arg0) {
            }

            @Override
            public void setType(int arg0) {
            }

        };
    }
}
