package rsim2.mcp;

import com.google.gson.*;
import org.joml.Vector3f;
import org.junit.jupiter.api.*;
import rsim2.collision.*;
import rsim2.motion.*;
import rsim2.scene.*;
import java.util.*;
import static org.junit.jupiter.api.Assertions.*;

class TopDownPickTest {
    SceneNode root, tool, box;
    List<Joint> joints;
    MotionPlayer player;
    McpRobotController robot;
    String arguments = "{\"object\":\"box\",\"tool\":\"tool\",\"tool_offset\":[0,0,0],\"tool_axis\":[1,0,0],\"clearance\":0.15,\"lift_distance\":0.1}";

    @BeforeEach void setup() {
        root=new SceneNode("world");
        SceneNode a=new SceneNode("a"), b=new SceneNode("b"), c=new SceneNode("c");
        tool=new SceneNode("tool"); box=new SceneNode("box");
        root.addChild(a); a.addChild(b); b.addChild(c); c.addChild(tool); root.addChild(box);
        a.getLocalPosition().set(0,1,0); b.getLocalPosition().set(1,0,0); c.getLocalPosition().set(1,0,0); tool.getLocalPosition().set(.3f,0,0);
        box.setRobotPart(false); box.getLocalPosition().set(1.2f,1.2f,0);
        box.setCollisionShape(new BoxShape(-.04f,-.04f,-.04f,.04f,.04f,.04f));
        joints=List.of(new Joint("a",root,a,new Vector3f(0,0,1)), new Joint("b",a,b,new Vector3f(0,0,1)), new Joint("c",b,c,new Vector3f(0,0,1)));
        player=new MotionPlayer(); robot=new McpRobotController();
        var hardware=rsim2.bridge.BridgeManager.getInstance();
        hardware.resetEmergencyStop(); hardware.setLiveSyncEnabled(false); hardware.setDigitalTwinEnabled(false);
    }

    JsonObject call(String tool, String args) {
        return robot.execute(tool,JsonParser.parseString(args).getAsJsonObject(),root,joints,player,false,()->{});
    }

    @Test void singleSequenceApproachesOrientsCapturesAndLiftsWithoutSkippingContact() {
        float initialY=ObjectInteractionController.objectCenter(box).y;
        assertEquals("running",call("pick_object_top_down",arguments).get("state").getAsString());
        MotionSequence sequence=player.getSequence();
        assertEquals(0,joints.get(0).getCurrentAngleRadians(),1e-6,"Planning must restore initial pose");
        assertSame(root,box.getParent(),"Planning must not attach the object");
        MotionCollisionState collision=new MotionCollisionState();
        for (int i=0;i<sequence.getKeyframes().size()+1 && player.isPlaying();i++) {
            collision.beginStep(root,joints);
            player.update(robot.limitDelta(100,player),joints);
            String blocked=collision.validateAndCommit(root,joints);
            assertNull(blocked);
            robot.afterStep(root,joints,player,blocked);
            assertSame(sequence,player.getSequence(),"All phases share one motion sequence");
        }
        assertEquals("completed",call("get_task_status","{}").get("state").getAsString());
        assertSame(tool,box.getParent());
        assertTrue(ObjectInteractionController.objectCenter(box).y-initialY>=.095f);
        assertTrue(TopDownPickPlanner.direction(tool,new Vector3f(1,0,0)).dot(new Vector3f(0,-1,0))>.999f);
        call("release_object","{\"object\":\"box\"}");
        assertSame(root,box.getParent());
    }

    @Test void unreachablePlanningRestoresEverything() {
        box.getLocalPosition().set(100,100,0);
        var original=box.getWorldTransform();
        assertThrows(IllegalArgumentException.class,()->call("pick_object_top_down",arguments));
        assertEquals(original,box.getWorldTransform());
        assertSame(root,box.getParent());
        for (Joint joint:joints) { assertEquals(0,joint.getCurrentAngleRadians()); assertEquals(0,joint.getMotor().getTargetAngleRadians()); }
        assertFalse(player.isPlaying());
    }

    @Test void actualToolBodyRemainsClearDuringDensePlayback() {
        tool.setCollisionShape(new BoxShape(-.01f,-.01f,-.01f,.01f,.01f,.01f));
        arguments=arguments.replace("[0,0,0]", "[0.011,0,0]");
        call("pick_object_top_down",arguments);
        MotionCollisionState collision=new MotionCollisionState();
        for (int i=0;i<20000 && player.isPlaying();i++) {
            collision.beginStep(root,joints);
            player.update(robot.limitDelta(.01f,player),joints);
            String blocked=collision.validateAndCommit(root,joints);
            assertNull(blocked);
            robot.afterStep(root,joints,player,blocked);
        }
        assertEquals("completed",call("get_task_status","{}").get("state").getAsString());
        assertSame(tool,box.getParent());
    }

    @Test void cartesianWaypointsReachWorldPositionAndDirection() {
        call("move_tcp", """
                {"tool":"tool","tool_offset":[0,0,0],"tool_axis":[1,0,0],
                 "waypoints":[{"position":[1.2,1.39,0]},{"position":[1.2,1.39,0],"direction":[0,-1,0]}]}
                """);
        int steps = player.getSequence().getKeyframes().size() + 1;
        for (int i = 0; i < steps && player.isPlaying(); i++) {
            player.update(robot.limitDelta(60, player), joints);
            robot.afterStep(root, joints, player, null);
        }
        assertEquals("completed", call("get_task_status", "{}").get("state").getAsString());
        assertTrue(TopDownPickPlanner.point(tool, new Vector3f()).distance(new Vector3f(1.2f,1.39f,0)) < .002f);
        assertTrue(TopDownPickPlanner.direction(tool,new Vector3f(1,0,0)).dot(new Vector3f(0,-1,0)) > .999f);
    }

    @Test void jointTrajectoryExecutesContactAndReleaseAtTheirWaypoints() {
        box.getLocalPosition().set(2.3f,1,0);
        box.setCollisionShape(null);
        call("execute_trajectory", """
                {"waypoints":[{"event":{"type":"capture","object":"box","tool":"tool","tool_offset":[0,0,0]}},
                              {"targets_degrees":{"a":10}},
                              {"event":{"type":"release","object":"box"}}]}
                """);
        player.update(robot.limitDelta(60, player), joints);
        robot.afterStep(root, joints, player, null);
        assertSame(tool, box.getParent());
        player.update(robot.limitDelta(60, player), joints);
        robot.afterStep(root, joints, player, null);
        assertSame(tool, box.getParent());
        player.update(robot.limitDelta(60, player), joints);
        robot.afterStep(root, joints, player, null);
        assertSame(root, box.getParent());
        var status = call("get_task_status", "{}");
        assertEquals(2, status.get("events_completed").getAsInt());
        assertEquals("completed", status.get("state").getAsString());
    }

    @Test void blockedPathDoesNotLeaveObjectAttachedOrChangePose() {
        SceneNode wall=new SceneNode("wall"); wall.setRobotPart(false);
        wall.getLocalPosition().set(1.2f,1.39f,0);
        wall.setCollisionShape(new BoxShape(-.2f,-.02f,-.2f,.2f,.02f,.2f));
        root.addChild(wall);
        tool.setCollisionShape(new BoxShape(-.01f,-.01f,-.01f,.01f,.01f,.01f));
        assertThrows(IllegalArgumentException.class,()->call("pick_object_top_down",arguments));
        assertSame(root,box.getParent());
        for (Joint joint:joints) assertEquals(0,joint.getCurrentAngleRadians());
        assertFalse(player.isPlaying());
    }

    @Test void movingObjectOrStopCannotCauseLateCapture() {
        call("pick_object_top_down",arguments);
        box.getLocalPosition().x+=.1f;
        robot.afterStep(root,joints,player,null);
        assertEquals("blocked",call("get_task_status","{}").get("state").getAsString());
        assertFalse(player.isPlaying()); assertSame(root,box.getParent());
        setup();
        call("pick_object_top_down",arguments);
        call("stop_motion","{}");
        player.update(100,joints);
        robot.afterStep(root,joints,player,null);
        assertEquals("stopped",call("get_task_status","{}").get("state").getAsString());
        assertSame(root,box.getParent());
    }

    @Test void pausedOrLoopingPickDoesNotAttachLater() {
        call("pick_object_top_down",arguments);
        player.pause();
        robot.afterStep(root,joints,player,null);
        assertEquals("interrupted",call("get_task_status","{}").get("state").getAsString());
        assertSame(root,box.getParent());
        setup();
        call("pick_object_top_down",arguments);
        player.setLooping(true);
        robot.afterStep(root,joints,player,null);
        assertEquals("blocked",call("get_task_status","{}").get("state").getAsString());
        assertFalse(player.isPlaying()); assertSame(root,box.getParent());
    }
}
