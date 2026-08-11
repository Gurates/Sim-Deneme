package rsim2.data;

public class MotorDTO {
    public String type;
    public float maxTorque;

    public MotorDTO() {}

    public MotorDTO(String type, float maxTorque) {
        this.type = type;
        this.maxTorque = maxTorque;
    }
}
