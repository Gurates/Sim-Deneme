package rsim2.safety;

import rsim2.scene.Joint;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class SafetyFilter {
    private boolean emergencyStopActive = false;
    private final Map<String, Float> lastSentAngles = new HashMap<>();
    private float maxDegreesPerStep = 45.0f;
    private boolean enforceServo0To180 = false;

    public SafetyFilter() {
    }

    public synchronized Map<String, Float> processAndFilter(Map<String, Float> rawAnglesDegrees, List<Joint> joints) {
        Map<String, Float> safeAngles = new HashMap<>();

        if (emergencyStopActive || rawAnglesDegrees == null) {
            return safeAngles;
        }

        for (Map.Entry<String, Float> entry : rawAnglesDegrees.entrySet()) {
            String jointId = entry.getKey();
            float rawDeg = entry.getValue();

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
                if (Math.abs(delta) > maxDegreesPerStep) {
                    clampedDeg = lastDeg + Math.signum(delta) * maxDegreesPerStep;
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
        this.lastSentAngles.clear();
    }

    public synchronized boolean isEmergencyStopActive() {
        return emergencyStopActive;
    }

    public synchronized void setMaxDegreesPerStep(float maxDeg) {
        this.maxDegreesPerStep = Math.max(1.0f, maxDeg);
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
