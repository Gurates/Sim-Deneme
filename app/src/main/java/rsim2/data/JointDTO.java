package rsim2.data;

public class JointDTO {
    public String id;
    public String type = "revolute";
    public String parent;
    public String child;
    public float[] axis;
    public float[] originPosition;
    public LimitsDTO limits;
    public MotorDTO motor;

    public JointDTO() {}
}
