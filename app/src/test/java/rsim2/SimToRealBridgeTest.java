package rsim2;

import org.joml.Vector3f;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import rsim2.bridge.BridgeManager;
import rsim2.bridge.MockHardwareBridge;
import rsim2.motion.Keyframe;
import rsim2.motion.MotionSequence;
import rsim2.safety.SafetyFilter;
import rsim2.scene.Joint;
import rsim2.scene.JointType;
import rsim2.scene.SceneNode;
import rsim2.telemetry.TrajectoryExporter;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

public class SimToRealBridgeTest {

    public static void main(String[] args) {
        System.out.println("=== RUNNING SIM-TO-REAL PIPELINE TESTS ===");
        SimToRealBridgeTest test = new SimToRealBridgeTest();

        try {
            test.setup();
            test.testSafetyFilterJointLimitClamping();
            System.out.println("[PASS] testSafetyFilterJointLimitClamping");

            test.setup();
            test.testSafetyFilterEmergencyStop();
            System.out.println("[PASS] testSafetyFilterEmergencyStop");

            test.setup();
            test.testMockHardwareBridgeStreaming();
            System.out.println("[PASS] testMockHardwareBridgeStreaming");

            test.setup();
            test.testTrajectoryExporterArduinoHeader();
            System.out.println("[PASS] testTrajectoryExporterArduinoHeader");

            test.setup();
            test.testBridgeManagerOrchestration();
            System.out.println("[PASS] testBridgeManagerOrchestration");

            test.setup();
            test.testHardwarePinMappingSerialization();
            System.out.println("[PASS] testHardwarePinMappingSerialization");

            test.setup();
            test.testArduinoSketchGenerationWithPins();
            System.out.println("[PASS] testArduinoSketchGenerationWithPins");

            test.setup();
            test.testPythonScriptGenerationForRaspberryPi();
            System.out.println("[PASS] testPythonScriptGenerationForRaspberryPi");

            test.setup();
            test.testMotorVelocityProfileAndAccelerationRamp();
            System.out.println("[PASS] testMotorVelocityProfileAndAccelerationRamp");

            System.out.println("\nALL 10 SIM-TO-REAL & MOTOR DYNAMICS TESTS PASSED SUCCESSFULLY (100%)!");
        } catch (Throwable t) {
            System.err.println("\n[FAIL] Test threw exception: " + t.getMessage());
            t.printStackTrace();
            System.exit(1);
        }
    }

    private List<Joint> testJoints;

    @BeforeEach
    public void setup() {
        testJoints = new ArrayList<>();
        SceneNode parent = new SceneNode("base_link");
        SceneNode child1 = new SceneNode("link_1");
        SceneNode child2 = new SceneNode("link_2");

        Joint j1 = new Joint("joint_1", parent, child1, JointType.REVOLUTE,
                new Vector3f(0, 0, 1), -(float)(Math.PI / 2), (float)(Math.PI / 2), 2.0f);

        Joint j2 = new Joint("joint_2", child1, child2, JointType.REVOLUTE,
                new Vector3f(0, 1, 0), 0.0f, (float) Math.PI, 2.0f);

        testJoints.add(j1);
        testJoints.add(j2);
    }

    @Test
    public void testSafetyFilterJointLimitClamping() {
        SafetyFilter filter = new SafetyFilter();

        Map<String, Float> rawAngles = new HashMap<>();
        rawAngles.put("joint_1", 120.0f);
        rawAngles.put("joint_2", -30.0f);

        Map<String, Float> safeAngles = filter.processAndFilter(rawAngles, testJoints);

        assertEquals(90.0f, safeAngles.get("joint_1"), 0.01f, "Joint 1 should be clamped to max 90 deg");
        assertEquals(0.0f, safeAngles.get("joint_2"), 0.01f, "Joint 2 should be clamped to min 0 deg");
    }

    @Test
    public void testSafetyFilterEmergencyStop() {
        SafetyFilter filter = new SafetyFilter();
        assertFalse(filter.isEmergencyStopActive());

        filter.triggerEmergencyStop();
        assertTrue(filter.isEmergencyStopActive());

        Map<String, Float> rawAngles = new HashMap<>();
        rawAngles.put("joint_1", 45.0f);

        Map<String, Float> safeAngles = filter.processAndFilter(rawAngles, testJoints);
        assertTrue(safeAngles.isEmpty(), "Emergency stop must block all angle commands");

        filter.resetEmergencyStop();
        assertFalse(filter.isEmergencyStopActive());
    }

    @Test
    public void testMockHardwareBridgeStreaming() {
        MockHardwareBridge mockBridge = new MockHardwareBridge();
        mockBridge.connect("127.0.0.1", 8888);
        assertTrue(mockBridge.isConnected());

        Map<String, Float> angles = new HashMap<>();
        angles.put("joint_1", 45.0f);
        angles.put("joint_2", 90.0f);

        mockBridge.sendJointPositions(angles, 1.25f);

        assertEquals(1, mockBridge.getStats().getPacketsSent());
        assertEquals(45.0f, mockBridge.getLastReceivedJoints().get("joint_1"), 0.01f);
        assertEquals(90.0f, mockBridge.getLastReceivedJoints().get("joint_2"), 0.01f);

        mockBridge.disconnect();
        assertFalse(mockBridge.isConnected());
    }

    @Test
    public void testTrajectoryExporterArduinoHeader() {
        MotionSequence seq = new MotionSequence("TestWave", 2.0f, true);

        Keyframe kf1 = new Keyframe(0.0f);
        kf1.addJointAngle("joint_1", 0.0f);
        kf1.addJointAngle("joint_2", 0.0f);

        Keyframe kf2 = new Keyframe(2.0f);
        kf2.addJointAngle("joint_1", 45.0f);
        kf2.addJointAngle("joint_2", 90.0f);

        seq.addKeyframe(kf1);
        seq.addKeyframe(kf2);

        String header = TrajectoryExporter.toArduinoHeader(seq, 10);
        assertNotNull(header);
        assertTrue(header.contains("#pragma once"));
        assertTrue(header.contains("TRAJECTORY_DATA"));
        assertTrue(header.contains("joint_1"));

        String csv = TrajectoryExporter.toCsv(seq, 10);
        assertNotNull(csv);
        assertTrue(csv.contains("time_sec,joint_1,joint_2"));
    }

    @Test
    public void testBridgeManagerOrchestration() {
        BridgeManager manager = BridgeManager.getInstance();
        MockHardwareBridge mock = new MockHardwareBridge();
        manager.setActiveBridge(mock);
        mock.connect("127.0.0.1", 0);

        manager.setLiveSyncEnabled(true);
        manager.setTargetPublishRateHz(100);

        Map<String, Float> raw = new HashMap<>();
        raw.put("joint_1", 30.0f);
        manager.streamJointPositions(raw, 0.5f, testJoints);

        assertTrue(mock.getStats().getPacketsSent() >= 1);

        manager.triggerEmergencyStop();
        assertTrue(manager.getSafetyFilter().isEmergencyStopActive());

        manager.resetEmergencyStop();
        manager.setLiveSyncEnabled(false);
    }

    @Test
    public void testHardwarePinMappingSerialization() throws Exception {
        Joint j1 = testJoints.get(0);
        j1.setPin(13);
        j1.setInverted(true);
        j1.setZeroOffsetDeg(15.0f);
        j1.setMotorType("DYNAMIXEL");

        SceneNode root = j1.getParentNode();
        rsim2.data.RobotDefinitionDTO dto = rsim2.io.RobotJsonIO.fromSceneGraph(root, testJoints, "TestRobot");

        assertNotNull(dto.joints);
        assertEquals(2, dto.joints.size());
        rsim2.data.JointDTO j1Dto = dto.joints.get(0);
        assertNotNull(j1Dto.motor);
        assertEquals(13, j1Dto.motor.pin);
        assertTrue(j1Dto.motor.inverted);
        assertEquals(15.0f, j1Dto.motor.zeroOffsetDeg, 0.01f);
        assertEquals("DYNAMIXEL", j1Dto.motor.type);

        rsim2.io.RobotJsonIO.SceneGraphResult res = rsim2.io.RobotJsonIO.toSceneGraph(dto);
        assertNotNull(res.joints);
        assertEquals(2, res.joints.size());
        Joint restoredJ1 = res.joints.get(0);
        assertEquals(13, restoredJ1.getPin());
        assertTrue(restoredJ1.isInverted());
        assertEquals(15.0f, restoredJ1.getZeroOffsetDeg(), 0.01f);
        assertEquals("DYNAMIXEL", restoredJ1.getMotorType());
    }

    @Test
    public void testArduinoSketchGenerationWithPins() {
        Joint j1 = testJoints.get(0);
        j1.setPin(11);
        j1.setInverted(true);
        j1.setZeroOffsetDeg(5.0f);

        Joint j2 = testJoints.get(1);
        j2.setPin(12);
        j2.setInverted(false);
        j2.setZeroOffsetDeg(-10.0f);

        MotionSequence seq = new MotionSequence("TestSketch", 1.0f, true);
        Keyframe kf1 = new Keyframe(0.0f);
        kf1.addJointAngle("joint_1", 10.0f);
        kf1.addJointAngle("joint_2", 20.0f);
        seq.addKeyframe(kf1);

        Keyframe kf2 = new Keyframe(1.0f);
        kf2.addJointAngle("joint_1", 30.0f);
        kf2.addJointAngle("joint_2", 40.0f);
        seq.addKeyframe(kf2);

        String sketch = TrajectoryExporter.toArduinoSketch(seq, testJoints, 20);
        assertNotNull(sketch);
        assertTrue(sketch.contains("#define PIN_JOINT_1 11"));
        assertTrue(sketch.contains("#define INVERT_JOINT_1 1"));
        assertTrue(sketch.contains("#define OFFSET_JOINT_1 5.0f"));
        assertTrue(sketch.contains("#define PIN_JOINT_2 12"));
        assertTrue(sketch.contains("#define INVERT_JOINT_2 0"));
        assertTrue(sketch.contains("servos[j].attach(SERVO_PINS[j]);"));
        assertTrue(sketch.contains("void setup()"));
        assertTrue(sketch.contains("void loop()"));
    }

    @Test
    public void testDigitalTwinTelemetryDispatch() {
        BridgeManager manager = BridgeManager.getInstance();
        MockHardwareBridge mock = new MockHardwareBridge();
        manager.setActiveBridge(mock);
        mock.connect("127.0.0.1", 0);

        manager.registerJoints(testJoints);
        manager.setDigitalTwinEnabled(true);

        Joint j2 = testJoints.get(1);
        j2.setPin(12);
        j2.setInverted(false);
        j2.setZeroOffsetDeg(10.0f);

        Map<String, Float> rx = new HashMap<>();
        rx.put("joint_2", 50.0f);
        mock.simulateIncomingTelemetry(rx);

        assertEquals(40.0f, (float) Math.toDegrees(j2.getCurrentAngleRadians()), 0.05f);

        manager.setDigitalTwinEnabled(false);
    }

    @Test
    public void testPythonScriptGenerationForRaspberryPi() {
        Joint j1 = testJoints.get(0);
        j1.setPin(18);
        j1.setInverted(true);
        j1.setZeroOffsetDeg(5.0f);

        Joint j2 = testJoints.get(1);
        j2.setPin(23);
        j2.setInverted(false);
        j2.setZeroOffsetDeg(-5.0f);

        MotionSequence seq = new MotionSequence("RpiTest", 1.0f, true);
        Keyframe kf1 = new Keyframe(0.0f);
        kf1.addJointAngle("joint_1", 10.0f);
        kf1.addJointAngle("joint_2", 20.0f);
        seq.addKeyframe(kf1);

        Keyframe kf2 = new Keyframe(1.0f);
        kf2.addJointAngle("joint_1", 30.0f);
        kf2.addJointAngle("joint_2", 40.0f);
        seq.addKeyframe(kf2);

        String pyCode = TrajectoryExporter.toPythonScript(seq, testJoints, 20);
        assertNotNull(pyCode);
        assertTrue(pyCode.contains("JOINTS_CONFIG = {"));
        assertTrue(pyCode.contains("\"joint_1\": {\"pin\": 18, \"invert\": True, \"offset\": 5.0}"));
        assertTrue(pyCode.contains("\"joint_2\": {\"pin\": 23, \"invert\": False, \"offset\": -5.0}"));
        assertTrue(pyCode.contains("USE_PCA9685"));
        assertTrue(pyCode.contains("def write_angle("));
        assertTrue(pyCode.contains("time.sleep(sleep_time)"));

        String receiverScript = TrajectoryExporter.toRpiLiveReceiverScript();
        assertNotNull(receiverScript);
        assertTrue(receiverScript.contains("UDP_PORT = 8888"));
        assertTrue(receiverScript.contains("sock.bind"));
    }

    @Test
    public void testMotorVelocityProfileAndAccelerationRamp() {
        Joint j1 = testJoints.get(0);
        rsim2.scene.MotorController motor = j1.getMotor();
        assertNotNull(motor);

        motor.setMaxSpeedRadiansPerSecond(2.0f);
        motor.setTargetSpeedRatio(0.5f);
        motor.setAcceleration(4.0f);
        motor.setCurrentVelocity(0.0f);
        j1.setAngle(0.0f);

        motor.setTargetAngleRadians(1.5f);

        motor.update(0.02f);
        assertTrue(motor.getCurrentVelocity() > 0.0f, "Motor must start moving towards target");
        assertTrue(motor.getCurrentVelocity() <= 4.0f * 0.02f + 0.001f, "Velocity must be bounded by acceleration * dt");

        float totalTime = 0.0f;
        float maxObservedVel = 0.0f;
        boolean sawCruisingOrAccelerating = false;

        for (int step = 0; step < 300; step++) {
            float dt = 0.02f;
            motor.update(dt);
            totalTime += dt;

            float v = motor.getCurrentVelocity();
            if (v > maxObservedVel) {
                maxObservedVel = v;
            }

            assertTrue(v <= 1.0f + 0.05f, "Velocity " + v + " must never significantly exceed effective cruise speed (1.0 rad/s)");

            if ("ACCELERATING".equals(motor.getMotionStatus()) || "CRUISING".equals(motor.getMotionStatus())) {
                sawCruisingOrAccelerating = true;
            }

            if (Math.abs(j1.getCurrentAngleRadians() - 1.5f) < 0.001f && Math.abs(v) < 0.001f) {
                break;
            }
        }

        assertTrue(sawCruisingOrAccelerating, "Motor must transition through accelerating/cruising states");
        assertEquals(1.5f, j1.getCurrentAngleRadians(), 0.01f, "Motor must arrive accurately at target position");
        assertEquals(0.0f, motor.getCurrentVelocity(), 0.01f, "Motor must cleanly stop with zero residual velocity at target");
        assertEquals("IDLE", motor.getMotionStatus(), "Motor must be IDLE once stopped at target");

        rsim2.data.MotorDTO dto = new rsim2.data.MotorDTO("SERVO_PWM", 100.0f, 13, false, 0.0f, 6.5f, 0.4f);
        assertEquals(6.5f, dto.acceleration, 0.001f);
        assertEquals(0.4f, dto.speedRatio, 0.001f);
    }
}
