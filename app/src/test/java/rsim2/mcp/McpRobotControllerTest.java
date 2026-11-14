package rsim2.mcp;

import com.google.gson.*;
import org.joml.Vector3f;
import org.junit.jupiter.api.*;
import rsim2.bridge.BridgeManager;
import rsim2.motion.MotionPlayer;
import rsim2.scene.*;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

class McpRobotControllerTest {
    SceneNode root;
    Joint joint;
    List<Joint> joints;
    MotionPlayer player;
    McpRobotController controller;

    @BeforeEach void setup() {
        root = new SceneNode("world");
        SceneNode arm = new SceneNode("arm");
        root.addChild(arm);
        joint = new Joint("shoulder", root, arm, new Vector3f(0, 1, 0));
        joints = List.of(joint);
        player = new MotionPlayer();
        controller = new McpRobotController();
        resetHardware();
    }

    @AfterEach void resetHardware() {
        var hardware = BridgeManager.getInstance();
        hardware.setLiveSyncEnabled(false);
        hardware.setDigitalTwinEnabled(false);
        hardware.resetEmergencyStop();
    }

    JsonObject call(String tool, String args) {
        return controller.execute(tool, JsonParser.parseString(args).getAsJsonObject(), root, joints, player, false, () -> {});
    }

    @Test void movesToRequestedAngleAndReportsCompletionOnlyAfterPlayback() {
        var accepted = call("move_joints", "{\"targets_degrees\":{\"shoulder\":30}}");
        assertEquals("running", accepted.get("state").getAsString());
        assertEquals(0, joint.getCurrentAngleRadians());
        assertNotNull(accepted.get("task_id"));
        assertThrows(IllegalArgumentException.class, () -> call("move_joints", "{\"targets_degrees\":{\"shoulder\":10}}"));
        player.update(10, joints);
        controller.afterStep(root, joints, player, null);
        assertEquals(30, Math.toDegrees(joint.getCurrentAngleRadians()), 0.001);
        assertEquals("completed", call("get_task_status", "{}").get("state").getAsString());
        assertEquals(1, call("get_robot_state", "{}").getAsJsonArray("joints").size());
        assertEquals(2, call("get_scene", "{}").getAsJsonArray("nodes").size());
    }

    @Test void validatesAllTargetsBeforeMutation() {
        for (String args : List.of("{\"targets_degrees\":{\"shoulder\":20,\"missing\":10}}",
                "{\"targets_degrees\":{\"shoulder\":999}}", "{\"targets_degrees\":{\"shoulder\":\"20\"}}",
                "{\"targets_degrees\":{}}", "{\"targets_degrees\":{\"shoulder\":1e999}}")) {
            assertThrows(IllegalArgumentException.class, () -> call("move_joints", args));
            assertEquals(0, joint.getMotor().getTargetAngleRadians());
            assertFalse(player.isPlaying());
        }
    }

    @Test void oneTaskVisitsOrderedWaypointsAndCanPauseAndResume() {
        var started = call("move_joints", """
                {"waypoints":[{"targets_degrees":{"shoulder":30}},
                              {"targets_degrees":{"shoulder":-20}},
                              {"targets_degrees":{"shoulder":10}}]}
                """);
        player.update(controller.limitDelta(60, player), joints);
        controller.afterStep(root, joints, player, null);
        assertEquals(30, Math.toDegrees(joint.getCurrentAngleRadians()), .001);
        assertEquals("paused", call("pause_motion", "{}").get("state").getAsString());
        player.update(60, joints);
        assertEquals(30, Math.toDegrees(joint.getCurrentAngleRadians()), .001);
        assertEquals("running", call("resume_motion", "{}").get("state").getAsString());
        player.update(controller.limitDelta(60, player), joints);
        controller.afterStep(root, joints, player, null);
        assertEquals(-20, Math.toDegrees(joint.getCurrentAngleRadians()), .001);
        player.update(controller.limitDelta(60, player), joints);
        controller.afterStep(root, joints, player, null);
        assertEquals(10, Math.toDegrees(joint.getCurrentAngleRadians()), .001);
        var finished = call("get_task_status", "{}");
        assertEquals(started.get("task_id"), finished.get("task_id"));
        assertEquals("completed", finished.get("state").getAsString());
        assertEquals(4, call("get_motion", "{}").getAsJsonObject("sequence").getAsJsonArray("keyframes").size());
    }

    @Test void aBadLaterWaypointDoesNotStartPartialMotion() {
        assertThrows(IllegalArgumentException.class, () -> call("execute_trajectory", """
                {"waypoints":[{"targets_degrees":{"shoulder":20}},{"targets_degrees":{"shoulder":300}}]}
                """));
        assertFalse(player.isPlaying());
        assertEquals(0, joint.getCurrentAngleRadians());
    }

    @Test void stopAndBlockedMotionAreTerminal() {
        call("move_joints", "{\"targets_degrees\":{\"shoulder\":20}}");
        player.update(0.1f, joints);
        float current = joint.getCurrentAngleRadians();
        assertEquals("stopped", call("stop_motion", "{}").get("state").getAsString());
        player.update(1, joints);
        assertEquals(current, joint.getCurrentAngleRadians());
        call("move_joints", "{\"targets_degrees\":{\"shoulder\":10}}");
        player.pause(); // Engine pauses playback when collision validation rejects a step.
        controller.afterStep(root, joints, player, "obstacle");
        assertEquals("blocked", call("get_task_status", "{}").get("state").getAsString());
    }

    @Test void safetyAndHardwareSyncRejectCommandsAndStopActiveTasks() {
        var hardware = BridgeManager.getInstance();
        hardware.setLiveSyncEnabled(true);
        assertThrows(IllegalArgumentException.class, () -> call("move_joints", "{\"targets_degrees\":{\"shoulder\":20}}"));
        hardware.setLiveSyncEnabled(false);
        hardware.setDigitalTwinEnabled(true);
        assertThrows(IllegalArgumentException.class, () -> call("move_joints", "{\"targets_degrees\":{\"shoulder\":20}}"));
        hardware.setDigitalTwinEnabled(false);
        call("move_joints", "{\"targets_degrees\":{\"shoulder\":20}}");
        hardware.triggerEmergencyStop();
        controller.afterStep(root, joints, player, null);
        assertFalse(player.isPlaying());
        assertEquals("stopped", call("get_task_status", "{}").get("state").getAsString());
    }

    @Test void sceneReplacementAndDisableStopTasks() {
        call("move_joints", "{\"targets_degrees\":{\"shoulder\":20}}");
        controller.afterStep(new SceneNode("replacement"), joints, player, null);
        assertFalse(player.isPlaying());
        assertEquals("interrupted", call("get_task_status", "{}").get("state").getAsString());
        call("move_joints", "{\"targets_degrees\":{\"shoulder\":20}}");
        controller.disable(player, joints);
        assertFalse(player.isPlaying());
        assertEquals("stopped", call("get_task_status", "{}").get("state").getAsString());
    }
}
