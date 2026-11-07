package rsim2;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;
import rsim2.ai.AiActionExecutor;
import rsim2.collision.*;
import rsim2.motion.MotionPlayer;
import rsim2.scene.*;
import rsim2.data.*;
import rsim2.io.RobotJsonIO;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

class PlaybackControlTest {
    @Test void directJointCommandSupportsPauseResumeStopAndReplay() {
        SceneNode base = new SceneNode("base"), link = new SceneNode("link");
        base.addChild(link);
        Joint joint = new Joint("joint", base, link, new Vector3f(0,0,1));
        List<Joint> joints = List.of(joint);
        MotionPlayer player = new MotionPlayer();
        var result = AiActionExecutor.processAndExecute("{\"actions\":[{\"joint\":\"joint\",\"target_deg\":60}]}", joints, player);
        assertTrue(result.hasAnimation);
        assertTrue(player.hasSequence());
        player.update(0.1f, joints);
        float pausedAngle = joint.getCurrentAngleRadians();
        assertTrue(pausedAngle > 0);
        player.pause();
        player.update(5, joints);
        joint.getMotor().update(0.05f);
        assertEquals(pausedAngle, joint.getCurrentAngleRadians(), 1e-6);
        player.play();
        player.update(player.getDuration(), joints);
        assertEquals(60, Math.toDegrees(joint.getCurrentAngleRadians()), .001);
        player.stop();
        player.seek(0, joints);
        assertEquals(0, joint.getCurrentAngleRadians(), .001);
        player.play();
        player.update(player.getDuration(), joints);
        assertEquals(60, Math.toDegrees(joint.getCurrentAngleRadians()), .001);
    }

    @Test void graspApproachSurvivesPauseAndStopAndCompletesOnReplay() {
        SceneNode root = new SceneNode("world"), arm = new SceneNode("arm"), tool = new SceneNode("tool"), box = new SceneNode("box");
        root.addChild(arm); arm.addChild(tool); root.addChild(box);
        tool.getLocalPosition().set(1,0,0);
        box.getLocalPosition().set(0,1,0); box.setRobotPart(false);
        Joint joint = new Joint("joint", root, arm, new Vector3f(0,0,1));
        List<Joint> joints = List.of(joint);
        MotionPlayer player = new MotionPlayer();
        ObjectInteractionController interactions = new ObjectInteractionController();
        var result = AiActionExecutor.processAndExecute("{\"interaction\":{\"type\":\"grasp\",\"object\":\"box\",\"tool\":\"tool\"}}", joints, player, root, interactions);
        assertTrue(result.hasAnimation);
        player.update(.1f, joints);
        player.pause();
        assertNull(interactions.update(100, root, false), "Paused approach must not time out");
        float paused = joint.getCurrentAngleRadians();
        joint.getMotor().update(.05f);
        assertEquals(paused, joint.getCurrentAngleRadians());
        player.stop(); player.seek(0, joints);
        assertNull(interactions.update(100, root, false));
        assertSame(root, box.getParent());
        player.play();
        for (int i = 0; i < 2000 && box.getParent() != tool; i++) {
            player.update(.01f, joints);
            interactions.update(.01f, root, false);
        }
        assertSame(tool, box.getParent());
    }

    @Test void rejectedGraspIsNotReportedAsExecution() {
        SceneNode root = new SceneNode("world");
        var result = AiActionExecutor.processAndExecute("Moving now.\n{\"interaction\":{\"type\":\"grasp\",\"object\":\"missing\",\"tool\":\"tool\"}}", List.of(), new MotionPlayer(), root, new ObjectInteractionController());
        assertFalse(result.hasActions);
        assertFalse(result.cleanedMessage.contains("Moving now"));
        assertTrue(result.cleanedMessage.contains("Command rejected"));
    }

    @Test void constantExistingGroundDepthAllowsYawButFurtherPenetrationIsRejected() {
        SceneNode root = new SceneNode("root"), moving = new SceneNode("moving");
        root.addChild(moving);
        moving.setCollisionShape(new BoxShape(-.1f,-.01f,-.1f,.1f,.1f,.1f));
        Joint joint = new Joint("yaw",root,moving,new Vector3f(0,1,0));
        assertNull(new EnvironmentMotionGuard().validateStep(root,List.of(joint),new float[]{0},new float[]{.3f}));
    }

    @Test void projectWithSeparateWorldRetainsArticulatedRobot() throws Exception {
        RobotDefinitionDTO def = new RobotDefinitionDTO();
        for (String id : new String[]{"world","base","tool"}) { LinkDTO link = new LinkDTO(); link.id=id; def.links.add(link); }
        JointDTO j = new JointDTO(); j.id="joint"; j.parent="base"; j.child="tool"; j.type="revolute"; def.joints.add(j);
        var loaded = RobotJsonIO.toSceneGraph(def);
        assertEquals("world", loaded.rootNode.getId());
        assertSame(loaded.nodesById.get("tool"), ObjectInteractionController.findNode(loaded.rootNode,"tool"));
        assertSame(loaded.rootNode, loaded.nodesById.get("base").getParent());
    }
}
