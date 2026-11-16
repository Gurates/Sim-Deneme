package rsim2.motion;

import rsim2.scene.Joint;
import rsim2.scene.ObjectInteractionController;
import rsim2.scene.SceneNode;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.ArrayList;

public class MotionPlayer {
    private MotionSequence sequence;
    private float currentTime = 0.0f;
    private float playbackSpeed = 1.0f;
    private boolean isPlaying = false;
    private boolean isLooping = false;
    private long stopGeneration;
    private long seekGeneration;
    private boolean replayingEvents;
    private boolean previewingIncompleteMotion;
    private int nextEvent;
    private int nextReplayFrame;
    private List<Joint> controlledJoints = List.of();

    private final Map<String, Float> sampledAngles = new HashMap<>();

    public MotionPlayer() {
    }

    public void loadSequence(MotionSequence sequence) {
        isPlaying = false;
        this.sequence = sequence;
        this.currentTime = 0.0f;
        this.isLooping = sequence != null && sequence.isLoop();
        replayingEvents = false;
        previewingIncompleteMotion = false;
        nextEvent = 0;
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
        if (!replayingEvents && currentTime == 0.0f && !sequence.isIncomplete() && !sequence.getEvents().isEmpty()) {
            replayingEvents = true;
            nextEvent = 0;
            nextReplayFrame = 0;
        }
        isPlaying = true;
    }

    /** Preview the joint keyframes of an interrupted interaction without replaying its events. */
    public void playIncompletePreview() {
        if (!hasSequence() || !sequence.isIncomplete()) {
            throw new IllegalArgumentException("No incomplete motion is loaded.");
        }
        if (!previewingIncompleteMotion) {
            currentTime = 0;
            nextReplayFrame = 0;
            replayingEvents = false;
            previewingIncompleteMotion = true;
        }
        isPlaying = true;
    }

    public void pause() {
        isPlaying = false;
        for (Joint joint : controlledJoints) if (joint.getMotor() != null) {
            joint.getMotor().setTargetAngleRadians(joint.getCurrentAngleRadians());
            joint.getMotor().setCurrentVelocity(0);
        }
    }

    public void bindJoints(List<Joint> joints) {
        controlledJoints = joints == null ? List.of() : new ArrayList<>(joints);
    }

    public void togglePlay() {
        if (isPlaying) {
            pause();
        } else {
            play();
        }
    }

    public void stop() {
        stopGeneration++;
        pause();
        replayingEvents = false;
        previewingIncompleteMotion = false;
        nextEvent = 0;
        currentTime = 0.0f;
    }

    public long getStopGeneration() { return stopGeneration; }
    public long getSeekGeneration() { return seekGeneration; }

    public void seek(float timeSeconds, List<Joint> joints) {
        bindJoints(joints);
        if (!hasSequence() || sequence.isIncomplete() || !sequence.getEvents().isEmpty()) return;
        seekGeneration++;
        this.currentTime = Math.max(0.0f, Math.min(sequence.getDurationSeconds(), timeSeconds));
        applyCurrentFrame(joints);
    }

    public void update(float deltaTime, List<Joint> joints) {
        bindJoints(joints);
        if (!isPlaying || !hasSequence() || joints == null) {
            return;
        }

        float nextTime = currentTime + deltaTime * playbackSpeed;
        if (replayingEvents) {
            if (nextEvent < sequence.getEvents().size()) {
                nextTime = Math.min(nextTime, sequence.getEvents().get(nextEvent).timeSeconds());
            }
        }
        if (replayingEvents || previewingIncompleteMotion) {
            List<Keyframe> frames = sequence.getKeyframes();
            if (nextReplayFrame < frames.size() && nextTime >= frames.get(nextReplayFrame).getTimeSeconds()) {
                nextTime = frames.get(nextReplayFrame).getTimeSeconds();
                do {
                    nextReplayFrame++;
                } while (nextReplayFrame < frames.size() && frames.get(nextReplayFrame).getTimeSeconds() == nextTime);
            }
        }
        currentTime = nextTime;

        float duration = sequence.getDurationSeconds();
        if (currentTime >= duration) {
            if (!previewingIncompleteMotion && (isLooping || sequence.isLoop())) {
                currentTime = currentTime % duration;
            } else {
                currentTime = duration;
                isPlaying = false;
                previewingIncompleteMotion = false;
            }
        }

        applyCurrentFrame(joints);
    }

    public boolean isReplayingEvents() { return replayingEvents; }
    public boolean isPreviewingIncompleteMotion() { return previewingIncompleteMotion; }

    public boolean startsNewPlayback() {
        return !replayingEvents && (currentTime == 0.0f || currentTime >= getDuration());
    }

    public void cancelReplay() {
        if (replayingEvents || previewingIncompleteMotion) stop();
    }


    public void applyReplayEvents(SceneNode root, List<Joint> joints, ObjectInteractionController interactions) {
        if (!replayingEvents) return;
        List<MotionEvent> events = sequence.getEvents();
        while (nextEvent < events.size() && events.get(nextEvent).timeSeconds() <= currentTime) {
            events.get(nextEvent).apply(root, joints, interactions);
            nextEvent++;
        }
        if (currentTime >= sequence.getDurationSeconds()) replayingEvents = false;
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
        if (sequence != null) {
            sequence.setLoop(looping);
            this.isLooping = sequence.isLoop();
        } else {
            this.isLooping = looping;
        }
    }
}
