package rsim2;

import org.joml.Vector3f;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import rsim2.ai.AiActionExecutor;
import rsim2.ai.RobotContextBuilder;
import rsim2.bridge.BridgeManager;
import rsim2.collision.CollisionWorld;
import rsim2.motion.MotionPlayer;
import rsim2.scene.Joint;
import rsim2.scene.JointType;
import rsim2.scene.SceneNode;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class AiInteractionTest {

    public static void main(String[] args) {
        System.out.println("=== RUNNING AI INTERACTION & CONTEXT TESTS ===");
        AiInteractionTest test = new AiInteractionTest();

        try {
            test.setup();
            test.testRobotContextBuilderDetailedOutput();
            System.out.println("[PASS] testRobotContextBuilderDetailedOutput");

            test.setup();
            test.testEmptySceneContext();
            System.out.println("[PASS] testEmptySceneContext");

            test.setup();
            test.testAiActionExecutorPoseHomeAndZero();
            System.out.println("[PASS] testAiActionExecutorPoseHomeAndZero");

            test.setup();
            test.testAiActionExecutorEmergencyStop();
            System.out.println("[PASS] testAiActionExecutorEmergencyStop");

            test.setup();
            test.testAiActionExecutorMultiJointCoordinated();
            System.out.println("[PASS] testAiActionExecutorMultiJointCoordinated");

            test.setup();
            test.testAiActionExecutorAnimationSequence();
            System.out.println("[PASS] testAiActionExecutorAnimationSequence");

            System.out.println("\nALL AI INTERACTION & KINEMATICS CONTEXT TESTS PASSED (100%)!");
        } catch (Throwable t) {
            System.err.println("\n[FAIL] Test failed: " + t.getMessage());
            t.printStackTrace();
            System.exit(1);
        }
    }

    private SceneNode rootNode;
    private List<Joint> testJoints;
    private MotionPlayer motionPlayer;
    private CollisionWorld collisionWorld;

    @BeforeEach
    public void setup() {
        rootNode = new SceneNode("world");
        SceneNode base = new SceneNode("base_link");
        SceneNode arm1 = new SceneNode("arm_link1");
        SceneNode arm2 = new SceneNode("arm_link2");

        base.getLocalPosition().set(0, 0, 0);
        arm1.getLocalPosition().set(0, 1.0f, 0);
        arm2.getLocalPosition().set(0, 1.0f, 0);

        rootNode.addChild(base);
        base.addChild(arm1);
        arm1.addChild(arm2);

        testJoints = new ArrayList<>();
        Joint j1 = new Joint("shoulder", base, arm1, JointType.REVOLUTE,
                new Vector3f(0, 0, 1), -(float)(Math.PI / 2), (float)(Math.PI / 2), 2.0f);
        j1.setPin(9);

        Joint j2 = new Joint("elbow", arm1, arm2, JointType.REVOLUTE,
                new Vector3f(1, 0, 0), -(float) Math.PI, (float) Math.PI, 2.0f);
        j2.setPin(10);

        testJoints.add(j1);
        testJoints.add(j2);

        motionPlayer = new MotionPlayer();
        collisionWorld = new CollisionWorld();
    }

    @Test
    public void testRobotContextBuilderDetailedOutput() {
        String context = RobotContextBuilder.buildContext(rootNode, testJoints, collisionWorld, motionPlayer);

        assertNotNull(context);
        assertTrue(context.contains("Degrees of Freedom (Active Joints): 2"), "Should include DOF count");
        assertTrue(context.contains("shoulder"), "Should mention shoulder joint");
        assertTrue(context.contains("elbow"), "Should mention elbow joint");
        assertTrue(context.contains("World Pos"), "Should include 3D Forward Kinematics world coordinates");
        assertTrue(context.contains("End-Effector"), "Should detect end-effector");
        assertTrue(context.contains("Collision Status"), "Should report collision status");
        assertTrue(context.contains("Hardware Pin=9"), "Should include hardware pin mappings");
        assertTrue(context.contains("PRESET POSES"), "Should specify pose action schema");
    }

    @Test
    public void testEmptySceneContext() {
        String context = RobotContextBuilder.buildContext(null, new ArrayList<>(), null, null);
        assertNotNull(context);
        assertTrue(context.contains("No robot or active joints currently loaded"));
    }

    @Test
    public void testAiActionExecutorPoseHomeAndZero() {
        // First set non-zero angles
        Joint j1 = testJoints.get(0);
        Joint j2 = testJoints.get(1);
        j1.setAngle((float) Math.toRadians(45.0));
        j2.setAngle((float) Math.toRadians(-30.0));

        String aiResponse = "Sure, I am returning the robot to its home pose.\n```json\n{\n  \"pose\": \"home\"\n}\n```";
        AiActionExecutor.ExecutionResult res = AiActionExecutor.processAndExecute(aiResponse, testJoints, motionPlayer);

        assertTrue(res.hasActions);
        assertEquals(0.0f, (float) Math.toDegrees(j1.getMotor().getTargetAngleRadians()), 0.01f);
        assertEquals(0.0f, (float) Math.toDegrees(j2.getMotor().getTargetAngleRadians()), 0.01f);
        assertTrue(res.cleanedMessage.contains("HOME"));
    }

    @Test
    public void testAiActionExecutorEmergencyStop() {
        BridgeManager.getInstance().resetEmergencyStop();
        assertFalse(BridgeManager.getInstance().getSafetyFilter().isEmergencyStopActive());

        String aiResponse = "Emergency stop requested! Halting immediately.\n```json\n{\n  \"system\": {\n    \"estop\": true\n  }\n}\n```";
        AiActionExecutor.ExecutionResult res = AiActionExecutor.processAndExecute(aiResponse, testJoints, motionPlayer);

        assertTrue(res.hasActions);
        assertTrue(BridgeManager.getInstance().getSafetyFilter().isEmergencyStopActive());

        // Reset
        BridgeManager.getInstance().resetEmergencyStop();
    }

    @Test
    public void testAiActionExecutorMultiJointCoordinated() {
        Joint j1 = testJoints.get(0);
        Joint j2 = testJoints.get(1);

        String aiResponse = "Moving shoulder to 30 deg and elbow to 60 deg.\n```json\n{\n  \"actions\": [\n    {\"joint\": \"shoulder\", \"target_deg\": 30.0, \"speed_pct\": 80},\n    {\"joint\": \"elbow\", \"target_deg\": 60.0}\n  ]\n}\n```";
        AiActionExecutor.ExecutionResult res = AiActionExecutor.processAndExecute(aiResponse, testJoints, motionPlayer);

        assertTrue(res.hasActions);
        assertEquals(30.0f, (float) Math.toDegrees(j1.getMotor().getTargetAngleRadians()), 0.01f);
        assertEquals(0.80f, j1.getMotor().getTargetSpeedRatio(), 0.01f);
        assertEquals(60.0f, (float) Math.toDegrees(j2.getMotor().getTargetAngleRadians()), 0.01f);
    }

    @Test
    public void testAiActionExecutorAnimationSequence() {
        String aiResponse = "Here is a waving animation.\n```json\n{\n  \"animation\": {\n    \"name\": \"Wave\",\n    \"duration\": 2.0,\n    \"loop\": true,\n    \"keyframes\": [\n      {\"time\": 0.0, \"joints\": {\"shoulder\": 0.0}},\n      {\"time\": 1.0, \"joints\": {\"shoulder\": 40.0}},\n      {\"time\": 2.0, \"joints\": {\"shoulder\": 0.0}}\n    ]\n  }\n}\n```";

        AiActionExecutor.ExecutionResult res = AiActionExecutor.processAndExecute(aiResponse, testJoints, motionPlayer);

        assertTrue(res.hasAnimation);
        assertTrue(res.hasActions);
        assertNotNull(res.generatedSequence);
        assertEquals("Wave", res.generatedSequence.getName());
        assertEquals(2.0f, res.generatedSequence.getDurationSeconds(), 0.01f);
        assertTrue(res.generatedSequence.isLoop());
        assertEquals(3, res.generatedSequence.getKeyframes().size());
        assertTrue(motionPlayer.isPlaying());
    }
}
