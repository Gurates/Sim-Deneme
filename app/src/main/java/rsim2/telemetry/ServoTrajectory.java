package rsim2.telemetry;

import rsim2.motion.Keyframe;
import rsim2.motion.MotionSequence;
import rsim2.scene.Joint;
import rsim2.scene.JointType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

record ServoTrajectory(String name, boolean loop, boolean hasSceneEvents, List<Channel> channels,
                       List<Sample> samples) {
    static final int MIN_PULSE_US = 544;
    static final int MAX_PULSE_US = 2400;
    static final int MAX_LATENESS_US = 50_000;
    private static final double ANGLE_TOLERANCE = 0.001;
    private static final int MAX_SAMPLES = 100_000;
    private static final int MAX_TARGETS = 1_000_000;

    record Channel(String id, int pin) {}
    record Sample(int timeUs, int[] centidegrees) {}

    static ServoTrajectory compile(MotionSequence sequence, List<Joint> joints, int rateHz, float speed,
                                   String selectedJoint, int selectedPin, boolean centered) {
        if (sequence == null || !sequence.hasKeyframes() || sequence.isIncomplete()) {
            throw new IllegalArgumentException("Load or finish recording a motion before exporting.");
        }
        float duration = sequence.getDurationSeconds();
        double usPerMotionSecond = 1_000_000.0 / speed;
        long durationUs = Math.round(duration * usPerMotionSecond);
        if (!Float.isFinite(duration) || !Float.isFinite(speed) || speed <= 0 || rateHz < 1 || rateHz > 200
                || durationUs < 1 || durationUs > Integer.MAX_VALUE) {
            throw new IllegalArgumentException("Export requires a valid playback speed, duration and rate of 1–200 Hz.");
        }
        if (joints == null) throw new IllegalArgumentException("Load the robot used to record this motion.");
        Map<String, Joint> model = new HashMap<>();
        for (Joint joint : joints) {
            if (model.putIfAbsent(joint.getId(), joint) != null) {
                throw new IllegalArgumentException("Duplicate joint ID in the robot: " + joint.getId());
            }
        }

        TreeMap<Integer, Float> clock = new TreeMap<>();
        int periodUs = (int) Math.round(1_000_000.0 / rateHz);
        if (Math.ceilDiv(durationUs, periodUs) >= MAX_SAMPLES || sequence.getKeyframes().size() > MAX_SAMPLES) {
            throw new IllegalArgumentException("This motion is too long to export. Split it into shorter motions.");
        }
        Map<String, Float> firstValues = new TreeMap<>();
        Set<String> moving = new HashSet<>();
        float previousTime = -1;
        for (Keyframe frame : sequence.getKeyframes()) {
            float time = frame.getTimeSeconds();
            if (!Float.isFinite(time) || time < 0 || time <= previousTime || time > duration) {
                throw new IllegalArgumentException("Motion keyframes must have distinct, increasing times within the motion duration.");
            }
            previousTime = time;
            int timeUs = (int) Math.round(time * usPerMotionSecond);
            if (clock.put(timeUs, time) != null || (timeUs == 0 && time != 0)
                    || (timeUs == durationUs && time != duration)) {
                throw new IllegalArgumentException("Motion times are too close together at this playback speed.");
            }
            for (var entry : frame.getJointAnglesDegrees().entrySet()) {
                String id = entry.getKey();
                if (selectedJoint != null && !selectedJoint.equals(id)) continue;
                Joint joint = model.get(id);
                if (joint == null) throw new IllegalArgumentException("Motion joint is missing from the robot: " + id);
                if (joint.getType() == JointType.FIXED) continue;
                float angle = modelAngle(joint, entry.getValue());
                Float first = firstValues.putIfAbsent(id, angle);
                if (first != null && angle != first) moving.add(id);
            }
        }

        List<Channel> channels = new ArrayList<>();
        Set<Integer> pins = new HashSet<>();
        for (String id : firstValues.keySet()) {
            Joint joint = model.get(id);
            int pin = selectedJoint != null ? selectedPin : joint.getPin();
            if (selectedJoint == null && pin == -1 && !moving.contains(id)) continue;
            if (pin < 0 || pin > 255) {
                throw new IllegalArgumentException("Set a valid pin/channel for " + id + " in Inspector > Hardware / Pin Mapping.");
            }
            if (!pins.add(pin)) throw new IllegalArgumentException("Pin/channel " + pin + " is assigned more than once.");
            if (!"SERVO_PWM".equalsIgnoreCase(joint.getMotorType())) {
                throw new IllegalArgumentException(id + " uses " + joint.getMotorType() + "; this export requires positional PWM servos.");
            }
            channels.add(new Channel(id, pin));
        }
        if (channels.isEmpty()) throw new IllegalArgumentException("The motion has no mapped PWM servo joints.");

        Map<String, Float> sampled = new HashMap<>();
        sequence.sampleClamped(0, sampled);
        for (Channel channel : channels) {
            if (!sampled.containsKey(channel.id())) {
                throw new IllegalArgumentException("The starting pose is missing " + channel.id() + ". Record its position in the first keyframe.");
            }
        }
        for (long timeUs = 0; timeUs < durationUs; timeUs += periodUs) {
            clock.putIfAbsent((int) timeUs, (float) (timeUs / usPerMotionSecond));
        }
        clock.put((int) durationUs, duration);
        if (clock.size() > MAX_SAMPLES || (long) clock.size() * channels.size() > MAX_TARGETS) {
            throw new IllegalArgumentException("This motion has too many servo commands. Split it into shorter motions.");
        }

        List<Sample> samples = new ArrayList<>(clock.size());
        List<Keyframe> frames = sequence.getKeyframes();
        MotionSequence segment = new MotionSequence("Export segment", duration, false);
        Map<String, Float> interpolated = new HashMap<>();
        int left = 0;
        int[] targets = new int[channels.size()];
        for (var tick : clock.entrySet()) {
            float time = tick.getValue();
            if (time <= frames.getFirst().getTimeSeconds()) {
                sampled = frames.getFirst().getJointAnglesDegrees();
            } else if (time >= frames.getLast().getTimeSeconds()) {
                sampled = frames.getLast().getJointAnglesDegrees();
            } else {
                while (time > frames.get(left + 1).getTimeSeconds()) left++;
                Keyframe from = frames.get(left);
                Keyframe to = frames.get(left + 1);
                if (time == to.getTimeSeconds()) {
                    interpolated.clear();
                    interpolated.putAll(from.getJointAnglesDegrees());
                    interpolated.putAll(to.getJointAnglesDegrees());
                } else {
                    // Reuse the simulator's interpolation without scanning the full keyframe list for every tick.
                    segment.getKeyframes().clear();
                    segment.getKeyframes().add(from);
                    segment.getKeyframes().add(to);
                    segment.sampleClamped(time, interpolated);
                }
                sampled = interpolated;
            }
            for (int i = 0; i < channels.size(); i++) {
                String id = channels.get(i).id();
                if (sampled.containsKey(id)) {
                    targets[i] = servoTarget(model.get(id), sampled.get(id), centered, time);
                }
            }
            samples.add(new Sample(tick.getKey(), targets.clone()));
        }
        if (sequence.isLoop() && !Arrays.equals(samples.getFirst().centidegrees(), samples.getLast().centidegrees())) {
            throw new IllegalArgumentException("Loop ends at a different pose. Turn off Loop or add a return to the starting pose.");
        }
        return new ServoTrajectory(sequence.getName(), sequence.isLoop(), !sequence.getEvents().isEmpty(),
                List.copyOf(channels), List.copyOf(samples));
    }

    private static float modelAngle(Joint joint, Float degrees) {
        if (degrees == null || !Float.isFinite(degrees) || !Float.isFinite(joint.getMinLimit())
                || !Float.isFinite(joint.getMaxLimit()) || joint.getMinLimit() > joint.getMaxLimit()) {
            throw new IllegalArgumentException("Invalid angle or joint limits for " + joint.getId() + ".");
        }
        float radians = (float) Math.toRadians(degrees);
        return (float) Math.toDegrees(Math.clamp(radians, joint.getMinLimit(), joint.getMaxLimit()));
    }

    private static int servoTarget(Joint joint, Float degrees, boolean centered, float time) {
        double angle = modelAngle(joint, degrees);
        double zero = centered ? 90 : (joint.isInverted() ? 180 : 0);
        double target = zero + joint.getZeroOffsetDeg() + (joint.isInverted() ? -angle : angle);
        if (!Double.isFinite(target) || target < -ANGLE_TOLERANCE || target > 180 + ANGLE_TOLERANCE) {
            throw new IllegalArgumentException(String.format(Locale.ROOT,
                    "%s maps to %.2f servo degrees at motion time %.3f s. Adjust its offset or motion to fit 0–180 degrees.",
                    joint.getId(), target, time));
        }
        return (int) Math.round(Math.clamp(target, 0, 180) * 100);
    }
}
