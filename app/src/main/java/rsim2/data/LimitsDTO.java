package rsim2.data;

public class LimitsDTO {
    public float min;
    public float max;
    public float maxSpeed;

    public LimitsDTO() {}

    public LimitsDTO(float min, float max, float maxSpeed) {
        this.min = min;
        this.max = max;
        this.maxSpeed = maxSpeed;
    }
}
