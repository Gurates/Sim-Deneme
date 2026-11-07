package rsim2;

import org.joml.Vector3f;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import rsim2.ai.*;
import rsim2.collision.*;
import rsim2.graphics.Mesh;
import rsim2.motion.MotionPlayer;
import rsim2.scene.*;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

class ApproachInteractionTest {
    SceneNode root, pivot, tool, box;
    Joint joint;
    MotionPlayer player;
    ObjectInteractionController interactions;

    @BeforeEach void setup() {
        root = new SceneNode("world"); pivot = new SceneNode("pivot"); tool = new SceneNode("tool"); box = new SceneNode("box");
        root.addChild(pivot); pivot.addChild(tool); root.addChild(box);
        pivot.getLocalPosition().y = 1;
        tool.getLocalPosition().x = 1;
        tool.setCollisionShape(new BoxShape(-.02f,-.02f,-.02f,.02f,.02f,.02f));
        box.setRobotPart(false);
        box.setCollisionShape(new BoxShape(-.1f,-.1f,-.1f,.1f,.1f,.1f));
        box.getLocalPosition().set(0, 3.5f, 0);
        joint = new Joint("joint", root, pivot, new Vector3f(0,0,1));
        player = new MotionPlayer(); interactions = new ObjectInteractionController();
        rsim2.bridge.BridgeManager.getInstance().resetEmergencyStop();
    }

    AiActionExecutor.ExecutionResult approach() {
        return AiActionExecutor.processAndExecute("{\"interaction\":{\"type\":\"approach\",\"object\":\"box\",\"tool\":\"tool\"}}", List.of(joint), player, root, interactions);
    }

    void executeAndCheck() {
        MotionCollisionState state = new MotionCollisionState();
        for (int i=0; i<2000 && player.isPlaying(); i++) {
            state.beginStep(root,List.of(joint));
            player.update(.01f,List.of(joint));
            assertNull(state.validateAndCommit(root,List.of(joint)));
            assertNull(interactions.update(.01f,root,false));
        }
        assertSame(root,box.getParent(), "Approach must never attach the object");
        assertEquals("free",interactions.state(box));
    }

    @Test void unreachableSurfaceStillAllowsCloserReachablePose() {
        float before = tool.getWorldTransform().getTranslation(new Vector3f()).distance(ObjectInteractionController.objectCenter(box));
        var result = approach();
        assertTrue(result.hasActions,result.cleanedMessage);
        assertTrue(result.hasAnimation);
        assertTrue(result.cleanedMessage.contains("Approach only"));
        assertEquals(0,joint.getCurrentAngleRadians(),1e-6);
        executeAndCheck();
        float after = tool.getWorldTransform().getTranslation(new Vector3f()).distance(ObjectInteractionController.objectCenter(box));
        assertTrue(after < before - .2f);
        assertTrue(after > 1, "Should not teleport to an unreachable surface");
    }

    @Test void blockedFullPathStillAllowsSafePartialAdvance() {
        box.getLocalPosition().set(0,2,0);
        SceneNode obstacle = new SceneNode("wall"); obstacle.setRobotPart(false);
        obstacle.getLocalPosition().set(.7071f,1.7071f,0);
        obstacle.setCollisionShape(new BoxShape(-.04f,-.04f,-.04f,.04f,.04f,.04f));
        root.addChild(obstacle);
        var result = approach();
        assertTrue(result.hasActions,result.cleanedMessage);
        executeAndCheck();
        assertTrue(joint.getCurrentAngleRadians() > .05f);
        assertTrue(joint.getCurrentAngleRadians() < Math.PI / 4);
    }

    @Test void immobileRobotRejectsWithoutFakeProgress() {
        joint.setLimits(0,0,2);
        var result = approach();
        assertFalse(result.hasActions);
        assertEquals(0,joint.getMotor().getTargetAngleRadians());
        assertFalse(player.hasSequence());
    }

    @Test void separateFingerMeshesKeepTheirGapButStillBlockFingerPenetration() {
        tool.setCollisionShape(null); tool.getLocalPosition().zero();
        tool.addMesh(new Mesh(new float[]{-.3f,-.1f,-.1f, -.2f,.1f,.1f},null,new int[0]));
        tool.addMesh(new Mesh(new float[]{.2f,-.1f,-.1f, .3f,.1f,.1f},null,new int[0]));
        box.getLocalPosition().set(0,1,0);
        EnvironmentMotionGuard guard = new EnvironmentMotionGuard();
        assertNull(guard.collisionAtPose(root,List.of(joint)), "Empty space between fingers must stay empty");
        box.getLocalPosition().x = .25f;
        assertNotNull(guard.collisionAtPose(root,List.of(joint)), "A real finger intersection must still be blocked");
    }

    @Test void aiContextDistinguishesApproachFromGrasp() {
        String context = RobotContextBuilder.buildContext(root,List.of(joint),null,player);
        assertTrue(context.contains("type=approach"));
        assertTrue(context.contains("Simply bring the robotic arm closer"));
    }

    @Test void samplingResolutionFollowsSceneScale() {
        EnvironmentMotionGuard guard = new EnvironmentMotionGuard();
        assertNull(guard.validateStep(root,List.of(joint),new float[]{0},new float[]{(float)Math.PI/2}));
        root.getLocalScale().set(1000);
        assertNull(guard.validateStep(root,List.of(joint),new float[]{0},new float[]{(float)Math.PI/2}),
                "Uniform scaling must not turn a clear path into a sample-budget rejection");
    }
}
