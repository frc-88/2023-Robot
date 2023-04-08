package frc.robot.ros.messages;

public class FrcColorRGBA {
    public float r = 0.0f;
    public float g = 0.0f;
    public float b = 0.0f;
    public float a = 1.0f;

    public FrcColorRGBA() {

    }

    public FrcColorRGBA(float r, float g, float b, float a) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.a = a;
    }
}
