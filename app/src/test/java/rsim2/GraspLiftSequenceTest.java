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
        for (int run = 0; run < 2; run++) {
            if (run == 1) player.stop();
            player.playInteraction(root, interactions);
            assertSame(root, box.getParent());
            assertFalse(interactions.isHeld(box));
            assertEquals(startPosition, box.getLocalPosition());
            assertEquals(0f, player.getCurrentTime());
            while (!interactions.isHeld(box) && player.isPlaying()) {
                step(60);
                player.applyReplayEvents(root, joints, interactions);
            }
            assertTrue(interactions.isHeld(box));
            player.pause();
            float pausedAt = player.getCurrentTime();
            player.playInteraction(root, interactions);
            assertEquals(pausedAt, player.getCurrentTime());
            assertSame(tool, box.getParent());
            while (player.isPlaying()) {
                step(60);
                player.applyReplayEvents(root, joints, interactions);
            }
            assertTrue(ObjectInteractionController.objectCenter(box).y - startPosition.y >= .094f);
        }
        interactions.release(root, "box");
        box.getLocalPosition().set(startPosition);
        AiActionExecutor.processAndExecute("{\"pose\":\"home\"}", joints, player, root, interactions, task);
        assertNotSame(wholeMotion, player.getSequence());
        while (player.isPlaying()) step(.02f);

        var loaded = MotionSequenceIO.load(file);
        assertEquals(wholeMotion.getEvents(), loaded.getEvents());
        assertEquals(wholeMotion.getKeyframes().size(), loaded.getKeyframes().size());
        player.loadSequence(loaded);
        player.playInteraction(root, interactions);
        for (int frame = 0; frame < 1500 && player.isPlaying(); frame++) {
            step(60);
            player.applyReplayEvents(root, joints, interactions);
        }
        assertFalse(player.isReplayingEvents());
        assertSame(tool, box.getParent());
        assertTrue(box.getWorldTransform().getTranslation(new Vector3f()).y - startPosition.y >= .094f);
    }

    @Test void slowFrameAfterCaptureStillShowsIntermediateLiftPoses() {
        float startY = ObjectInteractionController.objectCenter(box).y;
        float height = .1f;
        task.start(root, joints, interactions, player, "box", "tool", new Vector3f(), height, true);
        float normalFrame = 1f / 60;
        int risingFrames = 0;
        for (int frame = 0; frame < 1500 && task.isActive(); frame++) {
            collisions.beginStep(root, joints);
            String event = interactions.update(normalFrame, root, false);
            task.update(root, false, event);
            boolean capturedNow = event != null && interactions.isHeld(box);
            float before = ObjectInteractionController.objectCenter(box).y;
            if (capturedNow) assertEquals(startY, before, .00001f);
            player.updateFromRenderFrame(capturedNow ? .6f : normalFrame, joints);
            assertNull(collisions.validateAndCommit(root, joints));
            float rise = ObjectInteractionController.objectCenter(box).y - before;
            if (rise > .000001f) risingFrames++;
            assertTrue(rise < height * .15f, "Single-frame rise: " + rise);
        }
        assertFalse(task.isActive());
        assertTrue(interactions.isHeld(box));
        assertTrue(risingFrames > 10, "Expected a gradual lift, got " + risingFrames + " moving frames");
        assertEquals(startY + height, ObjectInteractionController.objectCenter(box).y, .006f);
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
