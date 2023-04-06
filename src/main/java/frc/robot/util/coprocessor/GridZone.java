package frc.robot.util.coprocessor;

public class GridZone {
    private String type = "";
    private String level = "";
    private String contents = "";
    private double x = 0.0;
    private double y = 0.0;
    private double z = 0.0;
    public boolean filled = false;

    public GridZone(String type, String level, String contents, double x, double y, double z) {
        this.type = type;
        this.level = level;
        this.contents = contents;

        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void setContents(String contents) {
        this.contents = contents;
    }

    public String getType() {
        return type;
    }

    public String getLevel() {
        return level;
    }

    public String getContents() {
        return contents;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double getDistance(GameObject gameObject) {
        return Math.sqrt(
                Math.pow((gameObject.getX() - x), 2) + 
                Math.pow((gameObject.getY() - y), 2) +
                Math.pow((gameObject.getZ() - z), 2)
            );
    }
}
