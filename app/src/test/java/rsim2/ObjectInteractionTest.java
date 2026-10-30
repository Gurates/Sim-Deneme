package rsim2;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import rsim2.ai.AiActionExecutor;
import rsim2.ai.RobotContextBuilder;
import rsim2.bridge.BridgeManager;
import rsim2.graphics.Mesh;
import rsim2.motion.MotionPlayer;
import rsim2.scene.*;

import java.util.List;
import java.util.Locale;

import static org.junit.jupiter.api.Assertions.*;

class ObjectInteractionTest {
    SceneNode root, base, arm, tip, box;
    Joint joint;
    ObjectInteractionController controller;

    @BeforeEach void setup() {
        BridgeManager.getInstance().resetEmergencyStop();
        root = new SceneNode("world");
        base = new SceneNode("base");
        arm = new SceneNode("arm");
        tip = new SceneNode("tool");
        tip.getLocalPosition().set(1, 0, 0);
        root.addChild(base);
        base.addChild(arm);
        arm.addChild(tip);
        joint = new Joint("shoulder", base, arm, new Vector3f(0, 0, 1));
        box = new SceneNode("box");
        box.setRobotPart(false);
        box.getLocalPosition().set(0, 1, 0);
        root.addChild(box);
        controller = new ObjectInteractionController();
    }

    String grasp() {
        return controller.grasp(root, List.of(joint), "box", "tool", new Vector3f());
    }

    void finishGrasp() {
        for (int i = 0; i < 500 && box.getParent() != tip; i++) {
            joint.getMotor().update(0.01f);
            controller.update(0.01f, root, false);
        }
        assertSame(tip, box.getParent());
    }

    @Test void approachAttachCarryReleasePreservesPose() {
        box.getLocalRotation().rotateZ(0.2f);
        box.getLocalScale().set(0.3f);
        Matrix4f original = box.getWorldTransform();
        assertTrue(grasp().contains("Approaching"));
        assertSame(root, box.getParent(), "Must not attach a distant object");
        assertEquals(0, joint.getCurrentAngleRadians(), 1e-6f, "IK must restore temporary poses");
        assertEquals(Math.PI / 2, joint.getMotor().getTargetAngleRadians(), 0.01);
        finishGrasp();
        assertTrue(original.equals(box.getWorldTransform(), 0.001f), "Attachment must preserve pose and scale");
        joint.setAngle(0);
        assertTrue(box.getWorldTransform().getTranslation(new Vector3f()).x > 0.95f);
        Matrix4f beforeRelease = box.getWorldTransform();
        controller.release(root, "box");
        assertSame(root, box.getParent());
        assertTrue(beforeRelease.equals(box.getWorldTransform(), 0.001f));
        joint.setAngle(0.7f);
        assertTrue(beforeRelease.equals(box.getWorldTransform(), 0.001f));
        assertEquals("free", controller.state(box));
    }

    @Test void unreachableAndLimitedTargetsDoNotMutateMotorsOrPose() {
        box.getLocalPosition().set(5, 5, 0);
        joint.getMotor().setTargetAngleRadians(0.3f);
        assertThrows(IllegalArgumentException.class, this::grasp);
        assertEquals(0, joint.getCurrentAngleRadians());
        assertEquals(0.3f, joint.getMotor().getTargetAngleRadians());
        box.getLocalPosition().set(0, 1, 0);
        joint.setLimits(-0.1f, 0.1f, 2);
        assertThrows(IllegalArgumentException.class, this::grasp);
        assertSame(root, box.getParent());
    }

    @Test void stopRemovalTimeoutAndConflictingMotionCancelPendingGrasp() {
        grasp();
        assertTrue(controller.update(0.01f, root, true).contains("cancelled"));
        assertEquals(0, joint.getMotor().getTargetAngleRadians());
        grasp();
        assertTrue(controller.update(100, root, false).contains("failed"));
        grasp();
        AiActionExecutor.processAndExecute("```json\n{\"pose\":\"home\"}\n```", List.of(joint), new MotionPlayer(), root, controller);
        assertNull(controller.update(0.1f, root, false));
        grasp();
        root.removeChild(box);
        assertTrue(controller.update(0.1f, root, false).contains("removed"));
    }

    @Test void localToolOffsetSupportsToolAtJointPivot() {
        tip.getLocalPosition().zero();
        controller.grasp(root, List.of(joint), "box", "tool", new Vector3f(1, 0, 0));
        finishGrasp();
        assertEquals(1, box.getWorldTransform().getTranslation(new Vector3f()).y, 0.001f);
    }

    @Test void twoJointIkWorksInTranslatedRotatedScaledRobotFrame() {
        base.getLocalPosition().set(2, 3, -1);
        base.getLocalRotation().rotateY(0.4f);
        base.getLocalScale().set(2);
        SceneNode tcp = new SceneNode("tcp");
        tcp.getLocalPosition().set(1, 0, 0);
        tip.addChild(tcp);
        Joint elbow = new Joint("elbow", arm, tip, new Vector3f(0, 0, 1));
        joint.setAngle(0.6f);
        elbow.setAngle(-0.8f);
        box.setWorldPosition(tcp.getWorldTransform().getTranslation(new Vector3f()));
        joint.setAngle(0);
        elbow.setAngle(0);
        List<Joint> joints = List.of(joint, elbow);
        controller.grasp(root, joints, "box", "tcp", new Vector3f());
        assertEquals(0, joint.getCurrentAngleRadians());
        assertEquals(0, elbow.getCurrentAngleRadians());
        for (int i = 0; i < 1000 && box.getParent() != tcp; i++) {
            for (Joint j : joints) j.getMotor().update(0.01f);
            controller.update(0.01f, root, false);
        }
        assertSame(tcp, box.getParent());
        Matrix4f heldPose = box.getWorldTransform();
        controller.release(root, "box");
        assertTrue(heldPose.equals(box.getWorldTransform(), 0.001f));
    }

    @Test void contextIncludesObjectsWithoutRobotAndUsesWorldCoordinates() {
        SceneNode group = new SceneNode("environment");
        group.getLocalPosition().set(2, 0, 0);
        root.addChild(group);
        box.reparentPreservingWorldTransform(group);
        box.getLocalPosition().set(1, 2, 3);
        Locale previous = Locale.getDefault();
        try {
            Locale.setDefault(Locale.forLanguageTag("tr-TR"));
            String context = RobotContextBuilder.buildContext(root, List.of(), null, null, controller);
            assertTrue(context.contains("\"box\""));
            assertTrue(context.contains("World Pos=(3.0000, 2.0000, 3.0000)"));
            assertTrue(context.contains("World rotation"));
            assertTrue(context.contains("No robot or active joints"));
        } finally { Locale.setDefault(previous); }
    }

    @Test void transformedMeshBoundsDefineGraspCenter() {
        Mesh mesh = new Mesh(new float[]{0, 0, 0, 2, 2, 2}, null, new int[0]);
        box.addMesh(mesh);
        box.getLocalScale().set(2);
        box.getLocalRotation().rotateZ((float) Math.PI / 2);
        Vector3f center = ObjectInteractionController.objectCenter(box);
        assertEquals(-2, center.x, 0.001f);
        assertEquals(3, center.y, 0.001f);
        assertEquals(2, center.z, 0.001f);
    }

    @Test void interactionJsonAndErrorsAreVisibleAndMixedCommandsCannotMoveRobot() {
        String json = "```json\n{\"interaction\":{\"type\":\"grasp\",\"object\":\"box\",\"tool\":\"tool\"}}\n```";
        var result = AiActionExecutor.processAndExecute(json, List.of(joint), new MotionPlayer(), root, controller);
        assertTrue(result.cleanedMessage.contains("Approaching the surface of box"));
        finishGrasp();
        assertTrue(RobotContextBuilder.buildContext(root, List.of(joint), null, null, controller).contains("held by tool"));
        result = AiActionExecutor.processAndExecute("{\"interaction\":{\"type\":\"release\",\"object\":\"box\"}}",
                List.of(joint), null, root, controller);
        assertTrue(result.cleanedMessage.contains("Released box"));
        String mixed = "```json\n{\"interaction\":{},\"pose\":\"home\"}\n```";
        float target = joint.getMotor().getTargetAngleRadians();
        result = AiActionExecutor.processAndExecute(mixed, List.of(joint), null, root, controller);
        assertTrue(result.cleanedMessage.contains("standalone"));
        assertEquals(target, joint.getMotor().getTargetAngleRadians());
        result = AiActionExecutor.processAndExecute(json.replace("\"box\"", "\"missing\""), List.of(joint), null, root, controller);
        assertTrue(result.cleanedMessage.contains("Interaction failed"));
    }

    @Test void rejectsRobotAsObjectAndDuplicateIds() {
        assertThrows(IllegalArgumentException.class, () -> controller.grasp(root, List.of(joint), "arm", "tool", new Vector3f()));
        SceneNode duplicate = new SceneNode("box");
        root.addChild(duplicate);
        assertThrows(IllegalArgumentException.class, this::grasp);
    }

    @Test void emergencyStopRejectsNewGrasp() {
        BridgeManager.getInstance().triggerEmergencyStop();
        try {
            var result = AiActionExecutor.processAndExecute("{\"interaction\":{\"type\":\"grasp\",\"object\":\"box\",\"tool\":\"tool\"}}",
                    List.of(joint), null, root, controller);
            assertTrue(result.cleanedMessage.contains("Emergency stop"));
            assertEquals(0, joint.getMotor().getTargetAngleRadians());
        } finally { BridgeManager.getInstance().resetEmergencyStop(); }
    }

    private void addSurfaceGeometry() {
        root.getLocalPosition().y = 1;
        tip.setCollisionShape(new rsim2.collision.BoxShape(-0.03f, -0.03f, -0.03f, 0.03f, 0.03f, 0.03f));
        box.setCollisionShape(new rsim2.collision.BoxShape(-0.1f, -0.1f, -0.1f, 0.1f, 0.1f, 0.1f));
        box.getLocalPosition().set(0, 1 - 0.1f - (float) Math.sqrt(3 * 0.03f * 0.03f) - 0.01f, 0);
    }

    @Test void surfaceGraspKeepsToolOutsideBoxForEntireApproach() {
        addSurfaceGeometry();
        var guard = new rsim2.collision.EnvironmentMotionGuard();
        grasp();
        for (int i = 0; i < 500 && box.getParent() != tip; i++) {
            float before = joint.getCurrentAngleRadians();
            joint.getMotor().update(0.01f);
            assertNull(guard.validateStep(root, List.of(joint), new float[]{before}, new float[]{joint.getCurrentAngleRadians()}));
            assertNull(guard.collisionAtPose(root, List.of(joint)));
            controller.update(0.01f, root, false);
        }
        assertSame(tip, box.getParent());
        assertNull(guard.collisionAtPose(root, List.of(joint)));
        assertTrue(tip.getWorldTransform().getTranslation(new Vector3f()).y > ObjectInteractionController.objectBounds(box).getMax().y);
    }

    @Test void obstacleOnOnlyReachableApproachRejectsWithoutMoving() {
        addSurfaceGeometry();
        SceneNode obstacle = new SceneNode("obstacle");
        obstacle.setRobotPart(false);
        obstacle.getLocalPosition().set(0.7071f, 0.7071f, 0);
        obstacle.setCollisionShape(new rsim2.collision.BoxShape(-0.05f, -0.05f, -0.05f, 0.05f, 0.05f, 0.05f));
        root.addChild(obstacle);
        assertThrows(IllegalArgumentException.class, this::grasp);
        assertEquals(0, joint.getCurrentAngleRadians());
        assertEquals(0, joint.getMotor().getTargetAngleRadians());
        assertSame(root, box.getParent());
    }

    @Test void movedTargetCancelsInsteadOfAttachingUsingStalePlan() {
        grasp();
        box.getLocalPosition().x += 0.1f;
        assertTrue(controller.update(0.01f, root, false).contains("object moved"));
        assertSame(root, box.getParent());
        assertEquals(0, joint.getMotor().getTargetAngleRadians());
    }

    @Test void attachmentRejectsShearWithoutChangingObjectPose() {
        base.getLocalScale().set(2, 1, 1);
        tip.getLocalRotation().rotateZ(0.5f);
        box.getLocalRotation().rotateZ(0.4f);
        box.setWorldPosition(tip.getWorldTransform().getTranslation(new Vector3f()));
        Matrix4f original = box.getWorldTransform();
        grasp();
        assertTrue(controller.update(0.01f, root, false).contains("distort geometry"));
        assertSame(root, box.getParent());
        assertTrue(original.equals(box.getWorldTransform(), 0.0001f));
        assertEquals("free", controller.state(box));
    }
}
