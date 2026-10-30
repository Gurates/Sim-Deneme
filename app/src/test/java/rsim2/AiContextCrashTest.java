package rsim2;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;
import rsim2.ai.RobotContextBuilder;
import rsim2.collision.CollisionWorld;
import rsim2.collision.ContactPair;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class AiContextCrashTest {
    @Test
    void groundContactCanBeReportedWhenSendingAnotherAiPrompt() {
        SceneNode root = new SceneNode("world");
        SceneNode base = new SceneNode("base");
        SceneNode tool = new SceneNode("tool");
        root.addChild(base);
        base.addChild(tool);
        Joint joint = new Joint("shoulder", base, tool, new Vector3f(0, 0, 1));
        CollisionWorld collisions = new CollisionWorld();
        collisions.getLastResult().addContact(ContactPair.createGroundCollision(tool, new Vector3f(), 0.01f));
        assertNull(collisions.getLastResult().getContacts().get(0).getNodeB());

        String context = assertDoesNotThrow(() -> RobotContextBuilder.buildContext(root, List.of(joint), collisions, null));
        assertTrue(context.contains("Ground"));
        assertTrue(context.contains("GROUND_COLLISION"));
        assertTrue(context.contains("tool"));
    }
}
