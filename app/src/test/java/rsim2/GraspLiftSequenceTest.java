package rsim2;

import org.joml.Vector3f;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import rsim2.ai.AiActionExecutor;
import rsim2.ai.GraspLiftCoordinator;
import rsim2.collision.BoxShape;
import rsim2.collision.MotionCollisionState;
import rsim2.io.MotionSequenceIO;
import rsim2.motion.MotionPlayer;
import rsim2.scene.Joint;
import rsim2.scene.ObjectInteractionController;
import rsim2.scene.SceneNode;

import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class GraspLiftSequenceTest {
    private final SceneNode root = new SceneNode("world");
    private final SceneNode tool = new SceneNode("tool");
    private final SceneNode box = new SceneNode("box");
    private final MotionPlayer player = new MotionPlayer();
    private final ObjectInteractionController interactions = new ObjectInteractionController();
    private final GraspLiftCoordinator task = new GraspLiftCoordinator();
    private final MotionCollisionState collisions = new MotionCollisionState();
    private List<Joint> joints;

    @BeforeEach void setUp() {
        var shoulder = new SceneNode("shoulder");
        var elbow = new SceneNode("elbow");
        root.addChild(shoulder);
        shoulder.addChild(elbow);
        elbow.addChild(tool);
        root.addChild(box);
        shoulder.getLocalPosition().y = 1;
        elbow.getLocalPosition().x = 1;
        tool.getLocalPosition().x = 1;
        box.setRobotPart(false);
        box.setCollisionShape(new BoxShape(-.04f, -.04f, -.04f, .04f, .04f, .04f));
        box.getLocalPosition().set(1.2f, 1.2f, 0);
        joints = List.of(new Joint("shoulder", root, shoulder, new Vector3f(0, 0, 1)),
                new Joint("elbow", shoulder, elbow, new Vector3f(0, 0, 1)));
        rsim2.bridge.BridgeManager.getInstance().resetEmergencyStop();
    }

    @Test void graspAndLiftKeepsApproachAndReplaysFromSavedFile(@TempDir Path directory) throws Exception {
        var startPosition = new Vector3f(box.getLocalPosition());
        var result = AiActionExecutor.processAndExecute("""
                {"interaction":{"type":"grasp_and_lift","object":"box","tool":"tool","lift_distance":0.1}}
                """, joints, player, root, interactions, task);
        assertTrue(result.hasActions, result.cleanedMessage);
        var wholeMotion = result.generatedSequence;
        var approachFrames = List.copyOf(wholeMotion.getKeyframes());
        float approachDuration = wholeMotion.getDurationSeconds();
        Path file = directory.resolve("pick.motion.json");
        assertThrows(IllegalArgumentException.class, () -> MotionSequenceIO.save(wholeMotion, file));

        String message = null;
        for (int frame = 0; frame < 1500 && task.isActive(); frame++) {
            step(.02f);
            String event = interactions.update(.02f, root, false);
            message = task.update(root, false, event);
            assertSame(wholeMotion, player.getSequence());
        }
        assertNotNull(message);
        assertTrue(message.contains("completed"), message);
        assertFalse(wholeMotion.isIncomplete());
        assertEquals(approachFrames, wholeMotion.getKeyframes().subList(0, approachFrames.size()));
        assertEquals(approachDuration, wholeMotion.getEvents().getFirst().timeSeconds());
        assertTrue(wholeMotion.getDurationSeconds() > approachDuration);
        assertSame(tool, box.getParent());
        assertTrue(box.getWorldTransform().getTranslation(new Vector3f()).y - startPosition.y >= .094f);

        MotionSequenceIO.save(wholeMotion, file);
        assertThrows(IllegalArgumentException.class, () -> wholeMotion.validateReplay(root, interactions));
        interactions.release(root, "box");
        box.getLocalPosition().set(startPosition);
        AiActionExecutor.processAndExecute("{\"pose\":\"home\"}", joints, player, root, interactions, task);
        assertNotSame(wholeMotion, player.getSequence());
        while (player.isPlaying()) step(.02f);

        var loaded = MotionSequenceIO.load(file);
        assertEquals(wholeMotion.getEvents(), loaded.getEvents());
        assertEquals(wholeMotion.getKeyframes().size(), loaded.getKeyframes().size());
        loaded.validateReplay(root, interactions);
        player.loadSequence(loaded);
        player.play();
        for (int frame = 0; frame < 1500 && player.isPlaying(); frame++) {
            step(60);
            player.applyReplayEvents(root, joints, interactions);
        }
        assertFalse(player.isReplayingEvents());
        assertSame(tool, box.getParent());
        assertTrue(box.getWorldTransform().getTranslation(new Vector3f()).y - startPosition.y >= .094f);
    }

    @Test void splitAiReplyDoesNotExecuteItsFirstPart() {
        var result = AiActionExecutor.processAndExecute("""
                ```json
                {"actions":[{"joint":"shoulder","target_deg":20}]}
                ```
                ```json
                {"actions":[{"joint":"shoulder","target_deg":30}]}
                ```
                """, joints, player, root, interactions, task);
        assertFalse(result.hasActions);
        assertFalse(player.hasSequence());
        assertEquals(0, joints.getFirst().getMotor().getTargetAngleRadians());
    }

    private void step(float seconds) {
        collisions.beginStep(root, joints);
        player.update(seconds, joints);
        assertNull(collisions.validateAndCommit(root, joints));
    }
}
