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

            System.out.println("\nALL 5 SIM-TO-REAL TESTS PASSED SUCCESSFULLY (100%)!");
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
}
