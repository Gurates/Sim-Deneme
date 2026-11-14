package rsim2.io;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import rsim2.motion.Keyframe;
import rsim2.motion.MotionSequence;

import java.io.IOException;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;

public final class MotionSequenceIO {
    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().create();

    private MotionSequenceIO() {}

    public static void save(MotionSequence sequence, Path path) throws IOException {
        if (sequence.isIncomplete()) throw new IllegalArgumentException("The interaction has not completed.");
        Path target = path.toAbsolutePath();
        Path draft = Files.createTempFile(target.getParent(), ".motion-", ".tmp");
        try {
            Files.writeString(draft, GSON.toJson(sequence));
            try {
                Files.move(draft, target, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
            } catch (AtomicMoveNotSupportedException error) {
                Files.move(draft, target, StandardCopyOption.REPLACE_EXISTING);
            }
        } finally {
            Files.deleteIfExists(draft);
        }
    }

    public static MotionSequence load(Path path) throws IOException {
        MotionSequence sequence;
        try (var reader = Files.newBufferedReader(path)) {
            sequence = GSON.fromJson(reader, MotionSequence.class);
        }
        if (sequence == null || !sequence.hasKeyframes() || !Float.isFinite(sequence.getDurationSeconds())
                || sequence.getDurationSeconds() <= 0) {
            throw new IllegalArgumentException("Expected a motion file with keyframes and a positive duration.");
        }
        for (Keyframe frame : sequence.getKeyframes()) {
            if (frame == null || !validTime(frame.getTimeSeconds(), sequence) || frame.getJointAnglesDegrees() == null
                    || frame.getJointAnglesDegrees().values().stream().anyMatch(angle -> angle == null || !Float.isFinite(angle))) {
                throw new IllegalArgumentException("Invalid joint keyframe in motion file.");
            }
        }
        float previousTime = 0;
        for (var event : sequence.getEvents()) {
            if (event == null || !validTime(event.timeSeconds(), sequence) || event.timeSeconds() < previousTime
                    || event.objectId() == null || event.objectId().isBlank()
                    || !Float.isFinite(event.offsetX()) || !Float.isFinite(event.offsetY()) || !Float.isFinite(event.offsetZ())) {
                throw new IllegalArgumentException("Invalid interaction event in motion file.");
            }
            previousTime = event.timeSeconds();
        }
        sequence.sortKeyframes();
        sequence.setLoop(sequence.isLoop());
        return sequence;
    }

    private static boolean validTime(float time, MotionSequence sequence) {
        return Float.isFinite(time) && time >= 0 && time <= sequence.getDurationSeconds();
    }
}
