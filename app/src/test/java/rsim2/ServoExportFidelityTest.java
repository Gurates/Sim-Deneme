package rsim2;

import org.junit.jupiter.api.Test;
import rsim2.motion.Keyframe;
import rsim2.motion.MotionEvent;
import rsim2.motion.MotionSequence;
import rsim2.scene.Joint;
import rsim2.scene.JointType;
import rsim2.telemetry.TrajectoryExporter;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Pattern;

import static org.junit.jupiter.api.Assertions.*;

class ServoExportFidelityTest {
    @Test
    void allExportsPreserveTheSameCalibratedMotionAtPlaybackSpeed() {
        Joint shoulder = servo("shoulder_lift", 9);
        shoulder.setZeroOffsetDeg(0.3f);
        Joint wrist = servo("wrist", 10);
        wrist.setInverted(true);
        Joint unconnected = servo("unused", -1);
        var joints = List.of(shoulder, wrist, unconnected);
        var motion = new MotionSequence("Servo alignment", 0.35f, false);
        motion.addKeyframe(new Keyframe(0, Map.of("shoulder_lift", -90f, "wrist", 10f, "unused", 0f)));
        motion.addKeyframe(new Keyframe(0.13f, Map.of("shoulder_lift", 0f, "wrist", 20f, "unused", 0f)));
        motion.addKeyframe(new Keyframe(0.35f, Map.of("shoulder_lift", -60f, "wrist", -15f, "unused", 0f)));

        String sketch = TrajectoryExporter.toArduinoSketch(motion, joints, 10, 2);
        String header = TrajectoryExporter.toArduinoHeader(motion, joints, 10, 2);
        String python = TrajectoryExporter.toPythonScript(motion, joints, 10, 2);

        List<Command> actual = arduinoCommands(sketch);
        assertEquals(arduinoCommands(header), actual);
        assertEquals(pythonCommands(python), actual);
        assertEquals(List.of(0, 65000, 100000, 175000), actual.stream().map(Command::timeUs).toList());
        assertEquals(new Command(0, List.of(30, 8000)), actual.getFirst());
        assertEquals(new Command(65000, List.of(9030, 7000)), actual.get(1));
        assertEquals(new Command(175000, List.of(3030, 10500)), actual.getLast());
        Map<String, Float> atSample = new HashMap<>();
        motion.sampleClamped(0.2f, atSample);
        assertEquals(Math.round((90.3f + atSample.get("shoulder_lift")) * 100), actual.get(2).targets().get(0));
        assertEquals(Math.round((90f - atSample.get("wrist")) * 100), actual.get(2).targets().get(1));
        assertFalse(sketch.contains("unused"));
    }

    @Test
    void sparseMotionHoldsTheLastPositionRatherThanZeroingTheAxis() {
        Joint shoulder = servo("shoulder", 9);
        var motion = new MotionSequence("Sparse channel", 1, false);
        motion.addKeyframe(new Keyframe(0, Map.of("shoulder", -30f)));
        motion.addKeyframe(new Keyframe(0.2f, Map.of("shoulder", -15f)));
        motion.addKeyframe(new Keyframe(0.4f));
        motion.addKeyframe(new Keyframe(1));

        var commands = arduinoCommands(TrajectoryExporter.toArduinoSketch(motion, List.of(shoulder), 10));
        assertEquals(new Command(1000000, List.of(7500)), commands.getLast());
        motion.getKeyframes().getFirst().setJointAnglesDegrees(Map.of());
        var error = assertThrows(IllegalArgumentException.class,
                () -> TrajectoryExporter.toArduinoSketch(motion, List.of(shoulder), 10));
        assertTrue(error.getMessage().contains("starting pose is missing shoulder"));
    }

    @Test
    void rejectsUnsafeLoopResetAndLabelsSimulationOnlyEvents() {
        Joint shoulder = servo("shoulder", 9);
        var motion = new MotionSequence("Scene event", 1, true);
        motion.addKeyframe(new Keyframe(0, Map.of("shoulder", -20f)));
        motion.addKeyframe(new Keyframe(1, Map.of("shoulder", 15f)));
        var loopError = assertThrows(IllegalArgumentException.class,
                () -> TrajectoryExporter.toPythonScript(motion, List.of(shoulder), 20));
        assertTrue(loopError.getMessage().contains("Loop ends at a different pose"));

        motion.addEvent(new MotionEvent(0.6f, "box", "gripper", 0, 0, 0));
        String python = TrajectoryExporter.toPythonScript(motion, List.of(shoulder), 20);
        assertTrue(python.contains("capture/release events are not hardware commands"));
        assertEquals(new Command(1000000, List.of(10500)), pythonCommands(python).getLast());
    }

    private static Joint servo(String name, int pin) {
        Joint joint = new Joint(name, null, null, JointType.REVOLUTE, null);
        joint.setPin(pin);
        return joint;
    }

    private static List<Command> arduinoCommands(String source) {
        List<Integer> times = numbers(block(source, "TRAJECTORY_TIME_US[^=]+="));
        var rows = Pattern.compile("\\{([^}]+)}").matcher(block(source, "TRAJECTORY_DATA[^=]+="));
        List<Command> commands = new ArrayList<>();
        while (rows.find()) commands.add(new Command(times.get(commands.size()), numbers(rows.group(1))));
        assertEquals(times.size(), commands.size());
        return commands;
    }

    private static String block(String source, String declaration) {
        var block = Pattern.compile(declaration + "\\s*\\{(.*?)\\n};", Pattern.DOTALL).matcher(source);
        assertTrue(block.find());
        return block.group(1);
    }

    private static List<Command> pythonCommands(String source) {
        var rows = Pattern.compile("(?m)^    \\((\\d+), \\(([0-9, ]+)\\)\\),$").matcher(source);
        List<Command> commands = new ArrayList<>();
        while (rows.find()) commands.add(new Command(Integer.parseInt(rows.group(1)), numbers(rows.group(2))));
        return commands;
    }

    private static List<Integer> numbers(String text) {
        return Pattern.compile("\\d+").matcher(text).results().map(match -> Integer.valueOf(match.group())).toList();
    }

    private record Command(int timeUs, List<Integer> targets) {}
}
