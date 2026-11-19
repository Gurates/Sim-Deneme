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

    @Test void interruptedPreviewRestoresEditedTransformAndMotor() {
        pivot.getLocalRotation().rotateX(.17f);
        var rotation = new org.joml.Quaternionf(pivot.getLocalRotation());
        joint.getMotor().setTargetAngleRadians(.45f);
        int[] checks = {0};
        assertThrows(IllegalArgumentException.class, () -> interactions.previewApproach(root, List.of(joint),
                "box", "tool", new Vector3f(), () -> {
                    if (++checks[0] == 3) throw new IllegalArgumentException("expired");
                }));
        assertEquals(0, joint.getCurrentAngleRadians());
        assertEquals(rotation, pivot.getLocalRotation());
        assertEquals(.45f, joint.getMotor().getTargetAngleRadians());
        assertFalse(interactions.hasPendingGrasp());
    }

    @Test void comparesCandidateApproachesWithoutChangingPlaybackOrTargets() {
        pivot.getLocalPosition().y = 3;
        tool.setTcpOffset(new Vector3f());
        tool.setCollisionShape(new BoxShape(-.003f,-.003f,-.003f,.003f,.003f,.003f));
        box.getLocalPosition().set(0,4,0);
        SceneNode far = new SceneNode("far");
        far.setRobotPart(false);
        far.getLocalPosition().set(-.643f,2.234f,0);
        far.setCollisionShape(new BoxShape(-.1f,-.1f,-.1f,.1f,.1f,.1f));
        root.addChild(far);
        SceneNode wall = new SceneNode("wall");
        wall.setRobotPart(false);
        wall.getLocalPosition().set(.7071f,3.7071f,0);
        wall.setCollisionShape(new BoxShape(-.04f,-.04f,-.04f,.04f,.04f,.04f));
        root.addChild(wall);
        var joints = List.of(joint);
        var mcp = new rsim2.mcp.McpRobotController();
        mcp.useInteractions(root, interactions);
        var command = com.google.gson.JsonParser.parseString("{\"targets_degrees\":{\"joint\":15}}").getAsJsonObject();
        var started = mcp.execute("move_joints", command, root, joints, player, false, () -> {});
        var sequence = player.getSequence();
        float target = joint.getMotor().getTargetAngleRadians();
        var query = com.google.gson.JsonParser.parseString("{\"object\":\"box\",\"tool\":\"tool\"}").getAsJsonObject();
        var near = mcp.execute("preview_approach", query, root, joints, player, true, () -> fail("Read-only query took control"));
        query.addProperty("object", "far");
        var other = mcp.execute("preview_approach", query, root, joints, player, true, () -> fail("Read-only query took control"));
        assertTrue(near.get("initial_distance").getAsFloat() < other.get("initial_distance").getAsFloat());
        assertEquals("partial", near.get("status").getAsString(), near.toString());
        assertEquals("reachable", other.get("status").getAsString(), other.toString());
        var response = AiActionExecutor.processAndExecute("""
                {"preview_approach":{"tool":"tool","objects":["box","far"]}}
                """, joints, player, root, interactions);
        assertFalse(response.hasActions);
        assertEquals(near, response.previewResults.get(0));
        assertEquals(other, response.previewResults.get(1));
        assertEquals(0, joint.getCurrentAngleRadians());
        assertEquals(target, joint.getMotor().getTargetAngleRadians());
        assertTrue(player.isPlaying());
        assertSame(sequence, player.getSequence());
        assertEquals(started, mcp.execute("get_task_status", new com.google.gson.JsonObject(),
                root, joints, player, false, () -> fail("Status took control")));
    }

    @Test void samplingResolutionFollowsSceneScale() {
        EnvironmentMotionGuard guard = new EnvironmentMotionGuard();
        assertNull(guard.validateStep(root,List.of(joint),new float[]{0},new float[]{(float)Math.PI/2}));
        root.getLocalScale().set(1000);
        assertNull(guard.validateStep(root,List.of(joint),new float[]{0},new float[]{(float)Math.PI/2}),
                "Uniform scaling must not turn a clear path into a sample-budget rejection");
    }
}
