package rsim2.safety;

import rsim2.scene.Joint;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class SafetyFilter {
    private boolean emergencyStopActive = false;
    private boolean collisionStopActive = false;
    private boolean preventCollisionStreaming = true;
    private final Map<String, Float> lastSentAngles = new HashMap<>();

    private float maxDegreesPerSecond = 180.0f;
    private float maxDegreesPerStep = 45.0f;
    private boolean enforceServo0To180 = false;

    private long lastFilterTimeNanos = 0;
    private boolean softStartActive = false;
    private long softStartBeginNanos = 0;
    private static final float SOFT_START_DURATION_SEC = 1.5f;

    public SafetyFilter() {
    }

    public synchronized Map<String, Float> processAndFilter(Map<String, Float> rawAnglesDegrees, List<Joint> joints) {
        Map<String, Float> safeAngles = new HashMap<>();

        if (emergencyStopActive || (preventCollisionStreaming && collisionStopActive) || rawAnglesDegrees == null) {
            return safeAngles;
        }

        long nowNanos = System.nanoTime();
        float deltaSec;
        if (lastFilterTimeNanos == 0) {
            deltaSec = 0.02f;
        } else {
            deltaSec = (nowNanos - lastFilterTimeNanos) / 1_000_000_000.0f;
        }
        lastFilterTimeNanos = nowNanos;

        deltaSec = Math.max(0.001f, Math.min(deltaSec, 0.1f));

        float effectiveMaxDegPerSec = maxDegreesPerSecond;

        if (softStartActive) {
            float elapsed = (nowNanos - softStartBeginNanos) / 1_000_000_000.0f;
            if (elapsed < SOFT_START_DURATION_SEC) {
                float ramp = elapsed / SOFT_START_DURATION_SEC;
                ramp = ramp * ramp * (3.0f - 2.0f * ramp);
                effectiveMaxDegPerSec *= ramp;
            } else {
                softStartActive = false;
            }
        }

        float maxDeltaThisStep = effectiveMaxDegPerSec * deltaSec;

        for (Map.Entry<String, Float> entry : rawAnglesDegrees.entrySet()) {
            String jointId = entry.getKey();
            float rawDeg = entry.getValue();

            if (!Float.isFinite(rawDeg)) {
                continue;
            }

            Joint joint = findJoint(joints, jointId);
            float minDeg = -180.0f;
            float maxDeg = 180.0f;

            if (joint != null) {
                minDeg = (float) Math.toDegrees(joint.getMinLimit());
                maxDeg = (float) Math.toDegrees(joint.getMaxLimit());
            }

            float clampedDeg = Math.max(minDeg, Math.min(maxDeg, rawDeg));

            if (lastSentAngles.containsKey(jointId)) {
                float lastDeg = lastSentAngles.get(jointId);
                float delta = clampedDeg - lastDeg;
                if (Math.abs(delta) > maxDeltaThisStep) {
                    clampedDeg = lastDeg + Math.signum(delta) * maxDeltaThisStep;
                }
            }

            if (enforceServo0To180) {
                clampedDeg = Math.max(0.0f, Math.min(180.0f, clampedDeg + 90.0f));
            }

            safeAngles.put(jointId, clampedDeg);
            lastSentAngles.put(jointId, clampedDeg);
        }

        return safeAngles;
    }

    private Joint findJoint(List<Joint> joints, String jointId) {
        if (joints == null || jointId == null) return null;
        for (Joint j : joints) {
            if (j.getId().equalsIgnoreCase(jointId)) return j;
        }
        for (Joint j : joints) {
            if (j.getId().toLowerCase().contains(jointId.toLowerCase()) ||
                jointId.toLowerCase().contains(j.getId().toLowerCase())) {
                return j;
            }
        }
        return null;
    }

    public synchronized void triggerEmergencyStop() {
        this.emergencyStopActive = true;
    }

    public synchronized void resetEmergencyStop() {
        this.emergencyStopActive = false;
        this.softStartActive = true;
        this.softStartBeginNanos = System.nanoTime();
        this.lastFilterTimeNanos = 0;
    }

    public synchronized boolean isEmergencyStopActive() {
        return emergencyStopActive;
    }

    public synchronized void triggerCollisionStop() {
        this.collisionStopActive = true;
    }

    public synchronized void resetCollisionStop() {
        this.collisionStopActive = false;
    }

    public synchronized void setCollisionStopActive(boolean active) {
        this.collisionStopActive = active;
    }

    public synchronized boolean isCollisionStopActive() {
        return collisionStopActive;
    }

    public synchronized boolean isPreventCollisionStreaming() {
        return preventCollisionStreaming;
    }

    public synchronized void setPreventCollisionStreaming(boolean prevent) {
        this.preventCollisionStreaming = prevent;
    }

    public synchronized void setMaxDegreesPerSecond(float maxDegPerSec) {
        this.maxDegreesPerSecond = Math.max(10.0f, maxDegPerSec);
    }

    public synchronized float getMaxDegreesPerSecond() {
        return maxDegreesPerSecond;
    }

    public synchronized void setMaxDegreesPerStep(float maxDeg) {
        this.maxDegreesPerStep = Math.max(1.0f, maxDeg);
        this.maxDegreesPerSecond = Math.min(this.maxDegreesPerStep * 50.0f, 720.0f);
    }

    public synchronized float getMaxDegreesPerStep() {
        return maxDegreesPerStep;
    }

    public synchronized boolean isEnforceServo0To180() {
        return enforceServo0To180;
    }

    public synchronized void setEnforceServo0To180(boolean enforce) {
        this.enforceServo0To180 = enforce;
    }
}
