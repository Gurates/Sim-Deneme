package rsim2;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import rsim2.collision.*;
import rsim2.graphics.Mesh;
import rsim2.motion.Keyframe;
import rsim2.motion.MotionSequence;
import rsim2.safety.SafetyFilter;
import rsim2.scene.Joint;
import rsim2.scene.JointType;
import rsim2.scene.SceneNode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

public class CollisionDetectionTest {

    public static void main(String[] args) {
        System.out.println("=== RUNNING COLLISION DETECTION PIPELINE TESTS ===");
        CollisionDetectionTest test = new CollisionDetectionTest();

        try {
            test.testAABBIntersection();
            System.out.println("[PASS] testAABBIntersection");

            test.testOBBIntersectionSAT();
            System.out.println("[PASS] testOBBIntersectionSAT");

            test.testKinematicJointAdjacencyFilter();
            System.out.println("[PASS] testKinematicJointAdjacencyFilter");

            test.testCollisionWorldSelfAndGroundCollision();
            System.out.println("[PASS] testCollisionWorldSelfAndGroundCollision");

            test.testOfflineTrajectoryValidation();
            System.out.println("[PASS] testOfflineTrajectoryValidation");

            test.testSafetyFilterCollisionStopIntegration();
            System.out.println("[PASS] testSafetyFilterCollisionStopIntegration");

            System.out.println("\nALL 6 COLLISION DETECTION TESTS PASSED SUCCESSFULLY (100%)!");
        } catch (Throwable t) {
            System.err.println("\n[FAIL] Test threw exception: " + t.getMessage());
            t.printStackTrace();
            System.exit(1);
        }
    }

    private Mesh createTestCubeMesh(float halfSize) {
        float[] vertices = {
                -halfSize, -halfSize, -halfSize,
                 halfSize, -halfSize, -halfSize,
                 halfSize,  halfSize, -halfSize,
                -halfSize,  halfSize, -halfSize,
                -halfSize, -halfSize,  halfSize,
                 halfSize, -halfSize,  halfSize,
                 halfSize,  halfSize,  halfSize,
                -halfSize,  halfSize,  halfSize
        };
        int[] indices = {
                0, 1, 2, 2, 3, 0,
                4, 5, 6, 6, 7, 4,
                0, 1, 5, 5, 4, 0,
                2, 3, 7, 7, 6, 2,
                0, 3, 7, 7, 4, 0,
                1, 2, 6, 6, 5, 1
        };
        float[] normals = new float[vertices.length];
        return new Mesh(vertices, normals, indices);
    }

    @Test
    public void testAABBIntersection() {
        AABB box1 = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));
        AABB box2 = new AABB(new Vector3f(0.5f, 0.5f, 0.5f), new Vector3f(2, 2, 2));
        AABB box3 = new AABB(new Vector3f(5, 5, 5), new Vector3f(6, 6, 6));

        assertTrue(box1.intersects(box2), "Overlapping boxes must intersect");
        assertFalse(box1.intersects(box3), "Disjoint boxes must not intersect");

        Matrix4f translation = new Matrix4f().translate(5, 0, 0);
        AABB transformed = new AABB();
        box1.transform(translation, transformed);

        assertEquals(4.0f, transformed.getMin().x, 0.001f);
        assertEquals(6.0f, transformed.getMax().x, 0.001f);
    }

    @Test
    public void testOBBIntersectionSAT() {
        OBB obb1 = new OBB();
        AABB aabb1 = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));
        obb1.fromAABBAndTransform(aabb1, new Matrix4f());

        OBB obb2 = new OBB();
        obb2.fromAABBAndTransform(aabb1, new Matrix4f().translate(1.5f, 0, 0));

        OBB obb3 = new OBB();
        obb3.fromAABBAndTransform(aabb1, new Matrix4f().translate(5.0f, 0, 0));

        assertTrue(obb1.intersects(obb2), "OBBs with 0.5m overlap must intersect");
        assertFalse(obb1.intersects(obb3), "Separated OBBs must not intersect");

        OBB obb4 = new OBB();
        Matrix4f rot45 = new Matrix4f().translate(1.8f, 0, 0).rotateZ((float) Math.toRadians(45));
        obb4.fromAABBAndTransform(aabb1, rot45);

        assertTrue(obb1.intersects(obb4), "Rotated OBB penetrating corner must intersect");
    }

    @Test
    public void testKinematicJointAdjacencyFilter() {
        SceneNode base = new SceneNode("base_link");
        SceneNode link1 = new SceneNode("link_1");
        SceneNode link2 = new SceneNode("link_2");
        SceneNode link3 = new SceneNode("link_3");

        base.addChild(link1);
        link1.addChild(link2);
        link2.addChild(link3);

        base.addMesh(createTestCubeMesh(0.5f));
        link1.addMesh(createTestCubeMesh(0.5f));
        link2.addMesh(createTestCubeMesh(0.5f));
        link3.addMesh(createTestCubeMesh(0.5f));

        List<Joint> joints = new ArrayList<>();
        joints.add(new Joint("j1", base, link1, JointType.REVOLUTE, new Vector3f(0, 1, 0)));
        joints.add(new Joint("j2", link1, link2, JointType.REVOLUTE, new Vector3f(0, 1, 0)));
        joints.add(new Joint("j3", link2, link3, JointType.REVOLUTE, new Vector3f(0, 1, 0)));

        CollisionFilter filter = new CollisionFilter();
        filter.setIgnoreAdjacentJoints(true);

        filter.setJointAdjacencyDepth(1);
        assertFalse(filter.shouldCheckPair(base, link1, joints), "1-hop (base, link1) must be ignored");
        assertFalse(filter.shouldCheckPair(link1, link2, joints), "1-hop (link1, link2) must be ignored");
        assertTrue(filter.shouldCheckPair(base, link2, joints), "2-hop (base, link2) must be checked under Depth 1");

        filter.setJointAdjacencyDepth(2);
        assertFalse(filter.shouldCheckPair(base, link2, joints), "2-hop (base, link2) must be ignored under Depth 2");
        assertTrue(filter.shouldCheckPair(base, link3, joints), "3-hop (base, link3) must be checked under Depth 2");

        filter.ignorePair("base_link", "link_3");
        assertFalse(filter.shouldCheckPair(base, link3, joints), "Custom ignored pair must be ignored");
    }

    @Test
    public void testCollisionWorldSelfAndGroundCollision() {
        CollisionWorld world = new CollisionWorld();
        world.getFilter().setJointAdjacencyDepth(1);
        SceneNode root = new SceneNode("root");

        SceneNode base = new SceneNode("base_link");
        base.addMesh(createTestCubeMesh(0.5f));
        base.getLocalPosition().set(0, 2.0f, 0);

        SceneNode link1 = new SceneNode("link_1");
        link1.addMesh(createTestCubeMesh(0.5f));
        link1.getLocalPosition().set(0, 0, 0);

        SceneNode link2 = new SceneNode("link_2");
        link2.addMesh(createTestCubeMesh(0.5f));
        link2.getLocalPosition().set(0, 2.0f, 0);

        root.addChild(base);
        base.addChild(link1);
        link1.addChild(link2);

        List<Joint> joints = new ArrayList<>();
        Joint j1 = new Joint("j1", base, link1, JointType.REVOLUTE, new Vector3f(0, 1, 0));
        Joint j2 = new Joint("j2", link1, link2, JointType.REVOLUTE, new Vector3f(1, 0, 0));
        joints.add(j1);
        joints.add(j2);

        CollisionResult res1 = world.update(root, joints);
        assertFalse(res1.hasCollision(), "Initial elevated robot pose should have 0 collisions");

        link2.getLocalPosition().set(0, 0, 0);
        CollisionResult res2 = world.update(root, joints);
        assertTrue(res2.hasCollision(), "Penetrating non-adjacent link2 into base_link must trigger collision");
        assertTrue(res2.isNodeColliding(base));
        assertTrue(res2.isNodeColliding(link2));

        base.getLocalPosition().set(0, -0.2f, 0);
        link2.getLocalPosition().set(0, 5.0f, 0);
        CollisionResult res3 = world.update(root, joints);
        assertTrue(res3.hasCollision(), "Lowering link below ground plane must trigger ground collision");
        boolean foundGround = false;
        for (ContactPair cp : res3.getContacts()) {
            if (cp.getType() == ContactPair.CollisionType.GROUND_COLLISION) {
                foundGround = true;
                break;
            }
        }
        assertTrue(foundGround, "Ground collision contact pair must be generated");
    }

    @Test
    public void testOfflineTrajectoryValidation() {
        CollisionWorld world = new CollisionWorld();
        SceneNode root = new SceneNode("root");

        SceneNode base = new SceneNode("base_link");
        base.addMesh(createTestCubeMesh(0.5f));
        base.getLocalPosition().set(0, 5.0f, 0);

        SceneNode arm = new SceneNode("arm_link");
        arm.addMesh(createTestCubeMesh(0.5f));
        arm.getLocalPosition().set(3.0f, 0, 0);

        root.addChild(base);
        base.addChild(arm);

        List<Joint> joints = new ArrayList<>();
        Joint j = new Joint("j_arm", base, arm, JointType.REVOLUTE, new Vector3f(0, 0, 1));
        joints.add(j);

        MotionSequence safeSeq = new MotionSequence("SafeMotion", 1.0f, false);
        Keyframe kf1 = new Keyframe(0.0f);
        kf1.addJointAngle("j_arm", 0.0f);
        Keyframe kf2 = new Keyframe(1.0f);
        kf2.addJointAngle("j_arm", 10.0f);
        safeSeq.addKeyframe(kf1);
        safeSeq.addKeyframe(kf2);

        List<String> logs = new ArrayList<>();
        boolean safePass = world.validateTrajectory(safeSeq, joints, root, 0.1f, logs);
        assertTrue(safePass, "Safe trajectory above ground must pass validation");

        SceneNode swingBase = new SceneNode("swing_base");
        swingBase.addMesh(createTestCubeMesh(0.5f));
        swingBase.getLocalPosition().set(0, 1.0f, 0);

        SceneNode swingArm = new SceneNode("swing_arm");
        swingArm.addMesh(createTestCubeMesh(0.5f));
        swingArm.getLocalPosition().set(0, 2.0f, 0);

        SceneNode root2 = new SceneNode("root2");
        root2.addChild(swingBase);
        swingBase.addChild(swingArm);

        List<Joint> joints2 = new ArrayList<>();
        Joint j2 = new Joint("j_swing", swingBase, swingArm, JointType.REVOLUTE, new Vector3f(0, 0, 1));
        joints2.add(j2);

        MotionSequence safeSwing = new MotionSequence("SafeSwing", 1.0f, false);
        Keyframe skf1 = new Keyframe(0.0f);
        skf1.addJointAngle("j_swing", 0.0f);
        Keyframe skf2 = new Keyframe(1.0f);
        skf2.addJointAngle("j_swing", 20.0f);
        safeSwing.addKeyframe(skf1);
        safeSwing.addKeyframe(skf2);

        List<String> safeLogs = new ArrayList<>();
        boolean passSafe = world.validateTrajectory(safeSwing, joints2, root2, 0.1f, safeLogs);
        assertTrue(passSafe, "Upright trajectory must pass validation");

        swingBase.getLocalPosition().set(0, -0.2f, 0);

        MotionSequence badSwing = new MotionSequence("BadGroundMotion", 1.0f, false);
        Keyframe bkf1 = new Keyframe(0.0f);
        bkf1.addJointAngle("j_swing", 0.0f);
        Keyframe bkf2 = new Keyframe(1.0f);
        bkf2.addJointAngle("j_swing", 180.0f);
        badSwing.addKeyframe(bkf1);
        badSwing.addKeyframe(bkf2);

        List<String> badLogs = new ArrayList<>();
        boolean badPass = world.validateTrajectory(badSwing, joints2, root2, 0.1f, badLogs);
        assertFalse(badPass, "Trajectory swinging below ground plane must fail validation");
        assertFalse(badLogs.isEmpty(), "Trajectory validator must generate collision logs");
    }

    @Test
    public void testSafetyFilterCollisionStopIntegration() {
        SafetyFilter filter = new SafetyFilter();
        filter.setPreventCollisionStreaming(true);

        assertFalse(filter.isCollisionStopActive());

        Map<String, Float> rawAngles = new HashMap<>();
        rawAngles.put("joint_1", 45.0f);

        Map<String, Float> safe = filter.processAndFilter(rawAngles, null);
        assertFalse(safe.isEmpty(), "Commands should pass when no collision stop is active");

        filter.triggerCollisionStop();
        assertTrue(filter.isCollisionStopActive());

        Map<String, Float> blocked = filter.processAndFilter(rawAngles, null);
        assertTrue(blocked.isEmpty(), "Collision stop must block angle command dispatch to real hardware");

        filter.resetCollisionStop();
        assertFalse(filter.isCollisionStopActive());
    }
}
