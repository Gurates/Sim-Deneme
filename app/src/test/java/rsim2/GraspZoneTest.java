package rsim2;

import org.joml.Vector3f;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import rsim2.collision.BoxShape;
import rsim2.collision.CollisionWorld;
import rsim2.collision.EnvironmentMotionGuard;
import rsim2.collision.GraspZone;
import rsim2.scene.Joint;
import rsim2.scene.JointType;
import rsim2.scene.ObjectInteractionController;
import rsim2.scene.SceneNode;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Grasp zone: objects between the jaws do not collide with the gripper; closing the jaw grips, opening releases. */
class GraspZoneTest {
    SceneNode root, arm, palm, fixedJaw, movingJaw, box;
    Joint shoulder, jaw;
    List<Joint> joints;
    final EnvironmentMotionGuard guard = new EnvironmentMotionGuard();
    final ObjectInteractionController interactions = new ObjectInteractionController();

    static SceneNode link(String id, SceneNode parent, float x, float y, BoxShape shape) {
        SceneNode node = new SceneNode(id);
        node.getLocalPosition().set(x, y, 0);
        node.setCollisionShape(shape);
        parent.addChild(node);
        return node;
    }

    /**
     * Arm turning about the vertical axis at (0,1,0), palm at x=1, fixed jaw below (y 0.89..0.93) and a moving
     * jaw above, hinged at the palm front and swinging down toward the fixed jaw. A box sits between the jaws.
     */
    @BeforeEach void setup() {
        root = new SceneNode("world");
        arm = link("arm", root, 0, 1, new BoxShape(0, -.02f, -.02f, .9f, .02f, .02f));
        palm = link("palm", arm, 1, 0, new BoxShape(-.05f, -.15f, -.05f, .05f, .15f, .05f));
        fixedJaw = link("fixed_jaw", palm, .2f, -.09f, new BoxShape(-.15f, -.02f, -.05f, .15f, .02f, .05f));
        movingJaw = link("moving_jaw", palm, .05f, .12f, new BoxShape(0, -.02f, -.05f, .3f, .02f, .05f));
        shoulder = new Joint("shoulder", root, arm, new Vector3f(0, 1, 0));
        jaw = new Joint("jaw", palm, movingJaw, JointType.REVOLUTE, new Vector3f(0, 0, 1), -.6f, .3f, 2);
        joints = List.of(shoulder, jaw);
        box = link("box", root, 1.2f, 1, new BoxShape(-.04f, -.08f, -.04f, .04f, .08f, .04f));
        box.setRobotPart(false);
        rsim2.bridge.BridgeManager.getInstance().resetEmergencyStop();
    }

    @Test void detectsTheGripperAndItsClosedDirection() {
        List<GraspZone> zones = GraspZone.find(root, joints);
        assertEquals(1, zones.size());
        GraspZone zone = zones.get(0);
        assertSame(palm, zone.getTool());
        assertEquals(List.of(jaw), zone.getJaws());
        assertEquals(-.6f, zone.closedAngle(jaw), 1e-6, "Closed is the limit nearest the palm");
        assertTrue(zone.contains(box));
        Vector3f grasp = palm.getWorldTransform().transformPosition(zone.graspPointToolLocal());
        assertTrue(grasp.x > 1.05f && grasp.y > .9f && grasp.y < 1.15f, "Grasp point lies between the jaws: " + grasp);
    }

    @Test void objectBetweenTheJawsDoesNotCollideWithTheGripper() {
        // The box bottom (0.92) overlaps the fixed jaw top (0.93): a squeeze, not a crash.
        assertNull(guard.collisionAtPose(root, joints));
        CollisionWorld world = new CollisionWorld();
        assertFalse(world.update(root, joints).hasCollision(), "No contact reported between the jaws");
    }

    @Test void objectOutsideTheZoneOrHittingTheArmStillCollides() {
        box.getLocalPosition().set(.93f, 1, 0); // behind the jaws, overlapping the back of the palm
        assertNotNull(guard.collisionAtPose(root, joints));
        box.getLocalPosition().set(.5f, 1, 0); // on the arm
        String hit = guard.collisionAtPose(root, joints);
        assertNotNull(hit);
        assertTrue(hit.contains("arm"), hit);
    }

    @Test void swingingTheOpenGripperAroundTheObjectIsAllowed() {
        assertNull(guard.validateStep(root, joints, new float[]{.5f, .3f}, new float[]{0, .3f}),
                "Bringing the open jaws around the box must not be blocked");
    }

    @Test void closingTheJawGripsTheObjectAndOpeningReleasesIt() {
        jaw.setAngle(.3f);
        assertTrue(interactions.updateGripping(root, joints).isEmpty());
        String gripped = null;
        for (int frame = 1; frame <= 90 && gripped == null; frame++) {
            jaw.setAngle(.3f - .01f * frame);
            var events = interactions.updateGripping(root, joints);
            if (!events.isEmpty()) gripped = events.get(0);
        }
        assertNotNull(gripped, "Closing onto the box grips it");
        assertTrue(gripped.contains("box"), gripped);
        assertSame(palm, box.getParent());
        assertSame(box, interactions.grippedBy(jaw));
        float gripAngle = jaw.getCurrentAngleRadians();
        assertTrue(gripAngle > -.6f && gripAngle < .3f);

        // Driving the jaw further closed keeps it on the box surface.
        jaw.setAngle(gripAngle - .2f);
        interactions.updateGripping(root, joints);
        assertEquals(gripAngle, jaw.getCurrentAngleRadians(), 1e-6);
        assertEquals(gripAngle, jaw.getMotor().getTargetAngleRadians(), 1e-6);

        // The held box follows the arm, and touching its own jaws is not a collision.
        float before = ObjectInteractionController.objectCenter(box).z;
        assertNull(guard.validateStep(root, joints, new float[]{0, gripAngle}, new float[]{.4f, gripAngle}));
        shoulder.setAngle(.4f);
        assertNotEquals(before, ObjectInteractionController.objectCenter(box).z, 1e-3f);

        // Opening releases it where it is.
        Vector3f heldAt = ObjectInteractionController.objectCenter(box);
        jaw.setAngle(gripAngle + .1f);
        var released = interactions.updateGripping(root, joints);
        assertEquals(1, released.size(), released.toString());
        assertSame(root, box.getParent());
        assertNull(interactions.grippedBy(jaw));
        assertTrue(heldAt.distance(ObjectInteractionController.objectCenter(box)) < 1e-4f);
    }

    @Test void oneMoveTcpCallOpensApproachesGripsAndCarries() {
        // Planar two-joint arm, so straight Cartesian segments are reachable.
        SceneNode forearm = link("forearm", arm, .5f, 0, new BoxShape(0, -.02f, -.02f, .4f, .02f, .02f));
        arm.removeChild(palm);
        forearm.addChild(palm);
        palm.getLocalPosition().set(.5f, 0, 0);
        Joint elbow = new Joint("elbow", arm, forearm, new Vector3f(0, 1, 0));
        joints = List.of(shoulder, elbow, jaw);
        shoulder.setAngle(.5f);
        jaw.setAngle(-.6f);
        Vector3f graspLocal = GraspZone.find(root, joints).get(0).graspPointToolLocal();
        float[] stops = {0, -.4f};
        Vector3f[] positions = new Vector3f[stops.length];
        for (int i = 0; i < stops.length; i++) {
            shoulder.setAngle(stops[i]);
            positions[i] = palm.getWorldTransform().transformPosition(new Vector3f(graspLocal));
        }
        shoulder.setAngle(.5f);
        var mcp = new rsim2.mcp.McpRobotController();
        var player = new rsim2.motion.MotionPlayer();
        mcp.useInteractions(root, interactions);
        String args = String.format(java.util.Locale.ROOT, """
                {"tool":"palm","tool_offset":[%f,%f,%f],"waypoints":[
                  {"joints":{"jaw":17}},
                  {"position":[%f,%f,%f]},
                  {"joints":{"jaw":-34}},
                  {"position":[%f,%f,%f]}]}""",
                graspLocal.x, graspLocal.y, graspLocal.z,
                positions[0].x, positions[0].y, positions[0].z, positions[1].x, positions[1].y, positions[1].z);
        var started = mcp.execute("move_tcp", com.google.gson.JsonParser.parseString(args).getAsJsonObject(),
                root, joints, player, false, () -> {});
        assertEquals("running", started.get("state").getAsString(), started.toString());

        var state = new rsim2.collision.MotionCollisionState();
        java.util.List<String> events = new java.util.ArrayList<>();
        Vector3f boxStart = ObjectInteractionController.objectCenter(box);
        for (int frame = 0; frame < 3000 && player.isPlaying(); frame++) {
            state.beginStep(root, joints);
            player.update(mcp.limitDelta(1 / 60f, player), joints);
            String blocked = state.validateAndCommit(root, joints);
            assertNull(blocked, "One continuous pick must not be blocked");
            events.addAll(interactions.updateGripping(root, joints));
            mcp.afterStep(root, joints, player, blocked);
        }
        var status = mcp.execute("get_task_status", new com.google.gson.JsonObject(), root, joints, player, false, () -> {});
        assertEquals("completed", status.get("state").getAsString(), status.toString());
        assertEquals(1, events.size(), events.toString());
        assertTrue(events.get(0).startsWith("Gripped box"), events.get(0));
        assertTrue(status.get("last_grip_event") == null || status.get("last_grip_event").getAsString().contains("box"));
        assertSame(palm, box.getParent(), "Still held after the carry");
        assertTrue(ObjectInteractionController.objectCenter(box).distance(boxStart) > .3f, "The box was carried");

        // Put it back with one call: return, open the jaw, move away. It must release once and stay released.
        String back = String.format(java.util.Locale.ROOT, """
                {"tool":"palm","tool_offset":[%f,%f,%f],"waypoints":[
                  {"position":[%f,%f,%f]},
                  {"joints":{"jaw":17}},
                  {"position":[%f,%f,%f]}]}""",
                graspLocal.x, graspLocal.y, graspLocal.z,
                positions[0].x, positions[0].y, positions[0].z, positions[1].x, positions[1].y, positions[1].z);
        mcp.execute("move_tcp", com.google.gson.JsonParser.parseString(back).getAsJsonObject(), root, joints, player, false, () -> {});
        events.clear();
        for (int frame = 0; frame < 3000 && player.isPlaying(); frame++) {
            state.beginStep(root, joints);
            player.update(mcp.limitDelta(1 / 60f, player), joints);
            String blocked = state.validateAndCommit(root, joints);
            assertNull(blocked, "Putting the box back must not be blocked");
            events.addAll(interactions.updateGripping(root, joints));
            mcp.afterStep(root, joints, player, blocked);
        }
        assertEquals(1, events.size(), events.toString());
        assertTrue(events.get(0).startsWith("Released box"), events.get(0));
        assertSame(root, box.getParent(), "Released and left behind");
        assertEquals(17, Math.toDegrees(jaw.getCurrentAngleRadians()), .5, "The jaw ends open");
        assertTrue(ObjectInteractionController.objectCenter(box).distance(boxStart) < .05f, "Put back where it was");
    }

    @Test void jawMovingWithNothingBetweenItGripsNothing() {
        root.removeChild(box);
        jaw.setAngle(.3f);
        interactions.updateGripping(root, joints);
        for (int frame = 1; frame <= 90; frame++) {
            jaw.setAngle(.3f - .01f * frame);
            assertTrue(interactions.updateGripping(root, joints).isEmpty());
        }
    }

    @Test void singleJointArmWithoutGripperHasNoZone() {
        SceneNode world = new SceneNode("w");
        SceneNode pivot = link("pivot", world, 0, 1, new BoxShape(-.1f, -.1f, -.1f, .1f, .1f, .1f));
        assertTrue(GraspZone.find(world, List.of(new Joint("j", world, pivot, new Vector3f(0, 0, 1)))).isEmpty());
    }
}
