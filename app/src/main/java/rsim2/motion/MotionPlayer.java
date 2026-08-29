package rsim2.motion;

import rsim2.scene.Joint;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MotionPlayer {
    private MotionSequence sequence;
    private float currentTime = 0.0f;
    private float playbackSpeed = 1.0f;
    private boolean isPlaying = false;
    private boolean isLooping = false;

    private final Map<String, Float> sampledAngles = new HashMap<>();

    public MotionPlayer() {
    }

    public void loadSequence(MotionSequence sequence) {
        this.sequence = sequence;
        this.currentTime = 0.0f;
        if (sequence != null) {
            this.isLooping = sequence.isLoop();
        }
    }

    public MotionSequence getSequence() {
        return sequence;
    }

    public boolean hasSequence() {
        return sequence != null && !sequence.getKeyframes().isEmpty();
    }

    public void play() {
        if (!hasSequence()) return;
        if (currentTime >= sequence.getDurationSeconds() && !isLooping) {
            currentTime = 0.0f;
        }
        isPlaying = true;
    }

    public void pause() {
        isPlaying = false;
    }

    public void togglePlay() {
        if (isPlaying) {
            pause();
        } else {
            play();
        }
    }

    public void stop() {
        isPlaying = false;
        currentTime = 0.0f;
    }

    public void seek(float timeSeconds, List<Joint> joints) {
        if (!hasSequence()) return;
        this.currentTime = Math.max(0.0f, Math.min(sequence.getDurationSeconds(), timeSeconds));
        applyCurrentFrame(joints);
    }

    public void update(float deltaTime, List<Joint> joints) {
        if (!isPlaying || !hasSequence() || joints == null) {
            return;
        }

        currentTime += deltaTime * playbackSpeed;

        float duration = sequence.getDurationSeconds();
        if (currentTime >= duration) {
            if (isLooping || sequence.isLoop()) {
                currentTime = currentTime % duration;
            } else {
                currentTime = duration;
                isPlaying = false;
            }
        }

        applyCurrentFrame(joints);
    }

    private void applyCurrentFrame(List<Joint> joints) {
        if (sequence == null || joints == null) return;

        sequence.sample(currentTime, sampledAngles);

        for (Map.Entry<String, Float> entry : sampledAngles.entrySet()) {
            String jointId = entry.getKey();
            float deg = entry.getValue();
            float rad = (float) Math.toRadians(deg);

            Joint j = findJoint(joints, jointId);
            if (j != null) {
                float clampedRad = Math.max(j.getMinLimit(), Math.min(j.getMaxLimit(), rad));
                j.setAngle(clampedRad);
                if (j.getMotor() != null) {
                    j.getMotor().setTargetAngleRadians(clampedRad);
                }
            }
        }
    }

    private Joint findJoint(List<Joint> joints, String jointId) {
        for (Joint j : joints) {
            if (j.getId().equalsIgnoreCase(jointId)) {
                return j;
            }
        }
        for (Joint j : joints) {
            if (j.getId().toLowerCase().contains(jointId.toLowerCase()) ||
                jointId.toLowerCase().contains(j.getId().toLowerCase())) {
                return j;
            }
        }
        return null;
    }

    public float getCurrentTime() {
        return currentTime;
    }

    public void setCurrentTime(float currentTime) {
        this.currentTime = currentTime;
    }

    public float getDuration() {
        return (sequence != null) ? sequence.getDurationSeconds() : 0.0f;
    }

    public float getPlaybackSpeed() {
        return playbackSpeed;
    }

    public void setPlaybackSpeed(float playbackSpeed) {
        this.playbackSpeed = Math.max(0.1f, Math.min(5.0f, playbackSpeed));
    }

    public boolean isPlaying() {
        return isPlaying;
    }

    public boolean isLooping() {
        return isLooping;
    }

    public void setLooping(boolean looping) {
        this.isLooping = looping;
        if (sequence != null) {
            sequence.setLoop(looping);
        }
    }
}
