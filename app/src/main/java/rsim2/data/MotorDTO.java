package rsim2.data;

public class MotorDTO {
    public String type = "SERVO_PWM";
    public float maxTorque = 100.0f;
    public int pin = -1;
    public boolean inverted = false;
    public float zeroOffsetDeg = 0.0f;
    public float acceleration = 8.0f;
    public float speedRatio = 1.0f;

    public MotorDTO() {}

    public MotorDTO(String type, float maxTorque) {
        this.type = type;
        this.maxTorque = maxTorque;
    }

    public MotorDTO(String type, float maxTorque, int pin, boolean inverted, float zeroOffsetDeg) {
        this(type, maxTorque, pin, inverted, zeroOffsetDeg, 8.0f, 1.0f);
    }

    public MotorDTO(String type, float maxTorque, int pin, boolean inverted, float zeroOffsetDeg, float acceleration, float speedRatio) {
        this.type = type;
        this.maxTorque = maxTorque;
        this.pin = pin;
        this.inverted = inverted;
        this.zeroOffsetDeg = zeroOffsetDeg;
        this.acceleration = acceleration > 0 ? acceleration : 8.0f;
        this.speedRatio = speedRatio > 0 ? speedRatio : 1.0f;
    }
}

