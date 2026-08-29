package rsim2.motion;

import java.util.HashMap;
import java.util.Map;

public class Keyframe {
    private float timeSeconds;
    private Map<String, Float> jointAnglesDegrees;

    public Keyframe() {
        this(0.0f);
    }

    public Keyframe(float timeSeconds) {
        this.timeSeconds = Math.max(0.0f, timeSeconds);
        this.jointAnglesDegrees = new HashMap<>();
    }

    public Keyframe(float timeSeconds, Map<String, Float> jointAnglesDegrees) {
        this.timeSeconds = Math.max(0.0f, timeSeconds);
        this.jointAnglesDegrees = (jointAnglesDegrees != null) ? new HashMap<>(jointAnglesDegrees) : new HashMap<>();
    }

    public float getTimeSeconds() {
        return timeSeconds;
    }

    public void setTimeSeconds(float timeSeconds) {
        this.timeSeconds = Math.max(0.0f, timeSeconds);
    }

    public Map<String, Float> getJointAnglesDegrees() {
        return jointAnglesDegrees;
    }

    public void setJointAnglesDegrees(Map<String, Float> jointAnglesDegrees) {
        this.jointAnglesDegrees = (jointAnglesDegrees != null) ? new HashMap<>(jointAnglesDegrees) : new HashMap<>();
    }

    public void addJointAngle(String jointId, float angleDegrees) {
        if (jointId != null && !jointId.trim().isEmpty()) {
            jointAnglesDegrees.put(jointId.trim(), angleDegrees);
        }
    }

    public Float getJointAngle(String jointId) {
        return jointAnglesDegrees.get(jointId);
    }
}
