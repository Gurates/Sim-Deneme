package rsim2.scene;

public class MotorController {
    private final Joint targetJoint;
    private float targetAngleRadians;
    private float maxSpeedRadiansPerSecond;
    private float minLimitRadians;
    private float maxLimitRadians;

    public MotorController(Joint targetJoint) {
        this(targetJoint, 2.0f, -(float) Math.PI, (float) Math.PI);
    }

    public MotorController(Joint targetJoint, float maxSpeedRadiansPerSecond, float minLimitRadians, float maxLimitRadians) {
        this.targetJoint = targetJoint;
        this.maxSpeedRadiansPerSecond = Math.max(0.001f, maxSpeedRadiansPerSecond);
        this.minLimitRadians = minLimitRadians;
        this.maxLimitRadians = maxLimitRadians;
        this.targetAngleRadians = clamp(targetJoint != null ? targetJoint.getCurrentAngleRadians() : 0.0f, minLimitRadians, maxLimitRadians);
    }

    public void update(float deltaTime) {
        if (targetJoint == null || deltaTime <= 0.0f) {
            return;
        }

        float current = targetJoint.getCurrentAngleRadians();
        float target = clamp(targetAngleRadians, minLimitRadians, maxLimitRadians);
        float diff = target - current;

        if (Math.abs(diff) < 0.00001f) {
            return;
        }

        float maxStep = maxSpeedRadiansPerSecond * deltaTime;

        if (Math.abs(diff) <= maxStep) {
            targetJoint.setAngle(target);
        } else {
            float step = Math.signum(diff) * maxStep;
            targetJoint.setAngle(current + step);
        }
    }

    private float clamp(float value, float min, float max) {
        if (min > max) {
            float tmp = min;
            min = max;
            max = tmp;
        }
        return Math.max(min, Math.min(max, value));
    }

    public Joint getTargetJoint() {
        return targetJoint;
    }

    public float getTargetAngleRadians() {
        return targetAngleRadians;
    }

    public void setTargetAngleRadians(float targetAngleRadians) {
        this.targetAngleRadians = clamp(targetAngleRadians, minLimitRadians, maxLimitRadians);
    }

    public float getMaxSpeedRadiansPerSecond() {
        return maxSpeedRadiansPerSecond;
    }

    public void setMaxSpeedRadiansPerSecond(float maxSpeedRadiansPerSecond) {
        this.maxSpeedRadiansPerSecond = Math.max(0.001f, maxSpeedRadiansPerSecond);
    }

    public float getMinLimitRadians() {
        return minLimitRadians;
    }

    public void setMinLimitRadians(float minLimitRadians) {
        this.minLimitRadians = minLimitRadians;
        this.targetAngleRadians = clamp(this.targetAngleRadians, minLimitRadians, maxLimitRadians);
    }

    public float getMaxLimitRadians() {
        return maxLimitRadians;
    }

    public void setMaxLimitRadians(float maxLimitRadians) {
        this.maxLimitRadians = maxLimitRadians;
        this.targetAngleRadians = clamp(this.targetAngleRadians, minLimitRadians, maxLimitRadians);
    }
}
