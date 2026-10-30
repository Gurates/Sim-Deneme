package rsim2;

import org.joml.Vector3f;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import rsim2.collision.BoxShape;
import rsim2.collision.EnvironmentMotionGuard;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class EnvironmentMotionGuardTest {
    SceneNode root, pivot, tool;
    Joint joint;
    EnvironmentMotionGuard guard = new EnvironmentMotionGuard();

    @BeforeEach void setup() {
        root = new SceneNode("world");
        pivot = new SceneNode("pivot");
        pivot.getLocalPosition().set(0, 1, 0);
        root.addChild(pivot);
        tool = new SceneNode("tool");
        tool.getLocalPosition().set(1, 0, 0);
        tool.setCollisionShape(new BoxShape(-0.02f, -0.02f, -0.02f, 0.02f, 0.02f, 0.02f));
        pivot.addChild(tool);
        joint = new Joint("joint", root, pivot, new Vector3f(0, 0, 1));
    }

    SceneNode obstacle(String name, float x, float y, float extent) {
        SceneNode box = new SceneNode(name);
        box.setRobotPart(false);
        box.getLocalPosition().set(x, y, 0);
        box.setCollisionShape(new BoxShape(-extent, -extent, -extent, extent, extent, extent));
        root.addChild(box);
        return box;
    }

    @Test void sweptMotionBlocksIntermediateObstacleAndRestoresPose() {
        obstacle("thin obstacle", 0.7071f, 1.7071f, 0.01f);
        joint.setAngle((float) Math.PI / 2);
        assertNull(guard.collisionAtPose(root, List.of(joint)), "Endpoint itself is clear");
        String blocked = guard.validateStep(root, List.of(joint), new float[]{0}, new float[]{(float) Math.PI / 2});
        assertNotNull(blocked);
        assertTrue(blocked.contains("thin obstacle"));
        assertEquals(Math.PI / 2, joint.getCurrentAngleRadians(), 1e-6);
        assertEquals(0, joint.getMotor().getTargetAngleRadians(), "Validation must not alter motor targets");
    }

    @Test void clearMotionAndTouchingSurfacesAreAccepted() {
        SceneNode box = obstacle("box", 1.12f, 1, 0.1f);
        assertNull(guard.collisionAtPose(root, List.of(joint)), "Surface contact is allowed");
        box.getLocalPosition().x += 0.05f; // Room for the tool's corners as it rotates.
        assertNull(guard.validateStep(root, List.of(joint), new float[]{0}, new float[]{0.5f}));
    }

    @Test void finalOverlapIsBlockedEvenWithoutMeshOrDebugCollisionWorld() {
        obstacle("box", 1, 1, 0.1f);
        assertNotNull(guard.collisionAtPose(root, List.of(joint)));
    }

    @Test void carriedObjectCannotPassThroughAnotherObject() {
        tool.setCollisionShape(null);
        SceneNode carried = new SceneNode("carried box");
        carried.setRobotPart(false);
        carried.setCollisionShape(new BoxShape(-0.1f, -0.1f, -0.1f, 0.1f, 0.1f, 0.1f));
        tool.addChild(carried);
        obstacle("wall", 0.7071f, 1.7071f, 0.03f);
        String collision = guard.validateStep(root, List.of(joint), new float[]{0}, new float[]{(float) Math.PI / 2});
        assertNotNull(collision);
        assertTrue(collision.contains("carried box"));
    }

    @Test void environmentChildrenInheritClassification() {
        SceneNode group = new SceneNode("object group");
        group.setRobotPart(false);
        root.addChild(group);
        SceneNode meshPart = new SceneNode("object part");
        meshPart.getLocalPosition().set(1, 1, 0);
        meshPart.setCollisionShape(new BoxShape(-0.1f, -0.1f, -0.1f, 0.1f, 0.1f, 0.1f));
        group.addChild(meshPart);
        assertTrue(guard.collisionAtPose(root, List.of(joint)).contains("object part"));
    }

    @Test void movingBelowGroundIsBlockedButUnchangedGroundedBaseIsIgnored() {
        root.setCollisionShape(new BoxShape(-1, -0.1f, -1, 1, 0, 1));
        assertNull(guard.validateStep(root, List.of(joint), new float[]{0}, new float[]{0.3f}));
        tool.getLocalPosition().set(2, 0, 0);
        assertTrue(guard.validateStep(root, List.of(joint), new float[]{0}, new float[]{-(float) Math.PI / 2}).contains("ground"));
    }

    @Test void giantMotionFailsClosedAndRestoresPose() {
        assertNotNull(guard.validateStep(root, List.of(joint), new float[]{0}, new float[]{1000}));
        assertEquals(0, joint.getCurrentAngleRadians());
    }
}
