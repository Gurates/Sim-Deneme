package rsim2.scene;

public class MotorController {
    private final Joint targetJoint;
    private float targetAngleRadians;
    private float maxSpeedRadiansPerSecond;
    private float minLimitRadians;
    private float maxLimitRadians;

    private float targetSpeedRatio = 1.0f;

    private float acceleration = 8.0f;

    private float currentVelocity = 0.0f;

    public MotorController(Joint targetJoint) {
        this(targetJoint, 2.0f, -(float) Math.PI, (float) Math.PI, 8.0f);
    }

    public MotorController(Joint targetJoint, float maxSpeedRadiansPerSecond, float minLimitRadians, float maxLimitRadians) {
        this(targetJoint, maxSpeedRadiansPerSecond, minLimitRadians, maxLimitRadians, 8.0f);
    }

    public MotorController(Joint targetJoint, float maxSpeedRadiansPerSecond, float minLimitRadians, float maxLimitRadians, float acceleration) {
        this.targetJoint = targetJoint;
        this.maxSpeedRadiansPerSecond = Math.max(0.001f, maxSpeedRadiansPerSecond);
        this.minLimitRadians = minLimitRadians;
        this.maxLimitRadians = maxLimitRadians;
        this.acceleration = Math.max(0.01f, acceleration);
        this.targetAngleRadians = clamp(targetJoint != null ? targetJoint.getCurrentAngleRadians() : 0.0f, minLimitRadians, maxLimitRadians);
    }

    public void update(float deltaTime) {
        if (targetJoint == null || deltaTime <= 0.0f) {
            return;
        }

        deltaTime = Math.min(deltaTime, 0.05f);

        float current = targetJoint.getCurrentAngleRadians();
        float target = clamp(targetAngleRadians, minLimitRadians, maxLimitRadians);
        float distance = target - current;
        float absDist = Math.abs(distance);

        if (absDist < 0.0001f && Math.abs(currentVelocity) < 0.005f) {
            currentVelocity = 0.0f;
            targetJoint.setAngle(target);
            return;
        }

        float dir = Math.signum(distance);
        float vCruise = Math.max(0.001f, maxSpeedRadiansPerSecond * targetSpeedRatio);
        float safeAcc = Math.max(0.01f, acceleration);

        float vMaxBraking = (float) Math.sqrt(2.0f * safeAcc * absDist);
        float desiredSpeed = Math.min(vCruise, vMaxBraking);
        float targetV = dir * desiredSpeed;

        float vDiff = targetV - currentVelocity;
        float maxDeltaV = safeAcc * deltaTime;

        if (Math.abs(vDiff) <= maxDeltaV) {
            currentVelocity = targetV;
        } else {
            currentVelocity += Math.signum(vDiff) * maxDeltaV;
        }

        float step = currentVelocity * deltaTime;

        if (Math.abs(step) >= absDist || (Math.signum(distance) != Math.signum(distance - step))) {
            targetJoint.setAngle(target);
            currentVelocity = 0.0f;
        } else {
            targetJoint.setAngle(current + step);
        }
    }

    public String getMotionStatus() {
        float absV = Math.abs(currentVelocity);
        float distance = Math.abs(targetAngleRadians - (targetJoint != null ? targetJoint.getCurrentAngleRadians() : 0.0f));

        if (absV < 0.01f && distance < 0.001f) {
            return "IDLE";
        }

        float vCruise = maxSpeedRadiansPerSecond * targetSpeedRatio;
        float safeAcc = Math.max(0.01f, acceleration);
        float brakingDist = (absV * absV) / (2.0f * safeAcc);

        if (distance <= brakingDist * 1.1f && absV > 0.05f) {
            return "BRAKING";
        }
        if (absV >= vCruise * 0.95f) {
            return "CRUISING";
        }
        return "ACCELERATING";
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

    public float getTargetSpeedRatio() {
        return targetSpeedRatio;
    }

    public void setTargetSpeedRatio(float targetSpeedRatio) {
        this.targetSpeedRatio = Math.max(0.05f, Math.min(1.0f, targetSpeedRatio));
    }

    public float getEffectiveCruiseSpeed() {
        return maxSpeedRadiansPerSecond * targetSpeedRatio;
    }

    public float getAcceleration() {
        return acceleration;
    }

    public void setAcceleration(float acceleration) {
        this.acceleration = Math.max(0.01f, acceleration);
    }

    public float getCurrentVelocity() {
        return currentVelocity;
    }

    public void setCurrentVelocity(float currentVelocity) {
        this.currentVelocity = currentVelocity;
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

