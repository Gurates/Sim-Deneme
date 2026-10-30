package rsim2;

import org.joml.Vector3f;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import rsim2.collision.BoxShape;
import rsim2.collision.MotionCollisionState;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class MotionCollisionStateTest {
    SceneNode root, pivot, tool, obstacle;
    Joint joint;
    MotionCollisionState state;

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
        obstacle = new SceneNode("obstacle");
        obstacle.setRobotPart(false);
        obstacle.getLocalPosition().set(0.7071f, 1.7071f, 0);
        obstacle.setCollisionShape(new BoxShape(-0.03f, -0.03f, -0.03f, 0.03f, 0.03f, 0.03f));
        root.addChild(obstacle);
        state = new MotionCollisionState();
    }

    @Test void seekBetweenFramesIsCheckedAgainstLastAcceptedPose() {
        state.beginStep(root, List.of(joint));
        assertNull(state.validateAndCommit(root, List.of(joint)));
        joint.setAngle((float) Math.PI / 2); // UI seek after the frame guard.
        joint.getMotor().setTargetAngleRadians(joint.getCurrentAngleRadians());
        joint.getMotor().setCurrentVelocity(1);
        state.beginStep(root, List.of(joint));
        assertNotNull(state.validateAndCommit(root, List.of(joint)));
        assertEquals(0, joint.getCurrentAngleRadians());
        assertEquals(0, joint.getMotor().getTargetAngleRadians());
        assertEquals(0, joint.getMotor().getCurrentVelocity());
    }

    @Test void validMotionCommitsNewRollbackPose() {
        state.beginStep(root, List.of(joint));
        joint.setAngle(0.1f);
        assertNull(state.validateAndCommit(root, List.of(joint)));
        joint.setAngle((float) Math.PI / 2);
        assertNotNull(state.validateAndCommit(root, List.of(joint)));
        assertEquals(0.1f, joint.getCurrentAngleRadians());
    }

    @Test void newSceneOrJointIdentityStartsNewBaseline() {
        state.beginStep(root, List.of(joint));
        joint.setAngle((float) Math.PI / 2);
        SceneNode newRoot = new SceneNode("new world");
        root.removeChild(pivot);
        root.removeChild(obstacle);
        newRoot.addChild(pivot);
        newRoot.addChild(obstacle);
        state.beginStep(newRoot, List.of(joint));
        assertNull(state.validateAndCommit(newRoot, List.of(joint)));
        joint.setAngle(0);
        Joint replacement = new Joint("replacement", newRoot, pivot, new Vector3f(0, 0, 1));
        state.beginStep(newRoot, List.of(replacement));
        assertNull(state.validateAndCommit(newRoot, List.of(replacement)));
        state.beginStep(newRoot, List.of());
        assertNull(state.validateAndCommit(newRoot, List.of()));
    }
}
