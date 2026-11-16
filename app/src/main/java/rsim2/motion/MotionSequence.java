package rsim2.motion;

import java.util.*;
import rsim2.scene.ObjectInteractionController;
import rsim2.scene.SceneNode;

public class MotionSequence {
    private String name = "Unnamed Motion";
    private float durationSeconds = 1.0f;
    private boolean loop = false;
    private final List<Keyframe> keyframes = new ArrayList<>();
    private final List<MotionEvent> events = new ArrayList<>();
    private transient boolean incomplete;

    public MotionSequence() {
    }

    public MotionSequence(String name, float durationSeconds, boolean loop) {
        this.name = (name != null && !name.trim().isEmpty()) ? name.trim() : "Unnamed Motion";
        this.durationSeconds = Math.max(0.01f, durationSeconds);
        this.loop = loop;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = (name != null && !name.trim().isEmpty()) ? name.trim() : "Unnamed Motion";
    }

    public float getDurationSeconds() {
        return durationSeconds;
    }

    public void setDurationSeconds(float durationSeconds) {
        this.durationSeconds = Math.max(0.01f, durationSeconds);
    }

    public boolean isLoop() {
        return loop;
    }

    public void setLoop(boolean loop) {
        this.loop = loop && events.isEmpty();
    }

    public List<MotionEvent> getEvents() {
        return Collections.unmodifiableList(events);
    }

    public void addEvent(MotionEvent event) {
        events.add(event);
        events.sort(Comparator.comparingDouble(MotionEvent::timeSeconds));
        loop = false;
    }

    public boolean isIncomplete() { return incomplete; }

    public void setIncomplete(boolean incomplete) { this.incomplete = incomplete; }

    public void validateReplay(SceneNode root, ObjectInteractionController interactions) {
        Set<String> checkedObjects = new HashSet<>();
        for (MotionEvent event : events) {
            SceneNode object = ObjectInteractionController.findNode(root, event.objectId());
            if (object == null || object == root || object.isRobotPart()) {
                throw new IllegalArgumentException("Motion object is missing: " + event.objectId());
            }
            if (event.toolId() != null) {
                SceneNode tool = ObjectInteractionController.findNode(root, event.toolId());
                if (tool == null || tool == root || !tool.isRobotPart()) {
                    throw new IllegalArgumentException("Motion tool is missing: " + event.toolId());
                }
            }
            if (checkedObjects.add(event.objectId()) && (event.toolId() == null) != interactions.isHeld(object)) {
                throw new IllegalArgumentException(event.toolId() == null ? "Hold " + event.objectId() + " before playback."
                        : "Release " + event.objectId() + " and return it to its starting position before playback.");
            }
        }
    }

    public void appendKeyframes(MotionSequence continuation) {
        float start = durationSeconds;
        for (Keyframe frame : continuation.keyframes) {
            if (frame.getTimeSeconds() > 0) {
                addKeyframe(new Keyframe(start + frame.getTimeSeconds(), frame.getJointAnglesDegrees()));
            }
        }
        setDurationSeconds(start + continuation.durationSeconds);
    }

    public List<Keyframe> getKeyframes() {
        return keyframes;
    }

    public boolean hasKeyframes() {
        return !keyframes.isEmpty();
    }

    public void addKeyframe(Keyframe keyframe) {
        if (keyframe != null) {
            keyframes.add(keyframe);
            sortKeyframes();
            if (keyframe.getTimeSeconds() > durationSeconds) {
                durationSeconds = keyframe.getTimeSeconds();
            }
        }
    }

    public void sortKeyframes() {
        keyframes.sort(Comparator.comparingDouble(Keyframe::getTimeSeconds));
    }

    public void sample(float timeSeconds, Map<String, Float> outAngles) {
        float t = timeSeconds;
        if (loop && durationSeconds > 0.0f) {
            t = t % durationSeconds;
            if (t < 0) t += durationSeconds;
        }
        sampleClamped(t, outAngles);
    }

    public void sampleClamped(float timeSeconds, Map<String, Float> outAngles) {
        if (outAngles == null) return;
        outAngles.clear();

        if (keyframes.isEmpty()) {
            return;
        }

        if (keyframes.size() == 1) {
            outAngles.putAll(keyframes.get(0).getJointAnglesDegrees());
            return;
        }

        float t = Math.max(0.0f, Math.min(durationSeconds, timeSeconds));
        Keyframe prevKf = keyframes.get(0);
        Keyframe nextKf = keyframes.get(keyframes.size() - 1);

        if (t <= prevKf.getTimeSeconds()) {
            outAngles.putAll(prevKf.getJointAnglesDegrees());
            return;
        }

        if (t >= nextKf.getTimeSeconds()) {
            outAngles.putAll(nextKf.getJointAnglesDegrees());
            return;
        }

        for (int i = 0; i < keyframes.size() - 1; i++) {
            Keyframe kfA = keyframes.get(i);
            Keyframe kfB = keyframes.get(i + 1);

            if (t >= kfA.getTimeSeconds() && t <= kfB.getTimeSeconds()) {
                prevKf = kfA;
                nextKf = kfB;
                break;
            }
        }

        float dt = nextKf.getTimeSeconds() - prevKf.getTimeSeconds();
        float alpha = (dt > 0.00001f) ? (t - prevKf.getTimeSeconds()) / dt : 0.0f;
        alpha = Math.max(0.0f, Math.min(1.0f, alpha));

        float smoothAlpha = alpha * alpha * (3.0f - 2.0f * alpha);

        Set<String> allJoints = new HashSet<>();
        allJoints.addAll(prevKf.getJointAnglesDegrees().keySet());
        allJoints.addAll(nextKf.getJointAnglesDegrees().keySet());

        for (String jointId : allJoints) {
            Float angleA = prevKf.getJointAngle(jointId);
            Float angleB = nextKf.getJointAngle(jointId);

            if (angleA != null && angleB != null) {
                float interpolated = angleA + smoothAlpha * (angleB - angleA);
                outAngles.put(jointId, interpolated);
            } else if (angleA != null) {
                outAngles.put(jointId, angleA);
            } else if (angleB != null) {
                outAngles.put(jointId, angleB);
            }
        }
    }
}
