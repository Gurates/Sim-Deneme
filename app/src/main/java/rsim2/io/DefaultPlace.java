package rsim2.io;

import rsim2.collision.CollisionWorld;
import rsim2.collision.OBB;
import rsim2.scene.SceneNode;

public class DefaultPlace {

    private static boolean groundCheck = false;
    private static SceneNode rootNode;

    public static void setRootNode(SceneNode rootNode) {
        DefaultPlace.rootNode = rootNode;
    }

    public static boolean run(CollisionWorld collisionWorld) {
        return run(rootNode, collisionWorld);
    }

    public static boolean run(SceneNode node, CollisionWorld collisionWorld) {
        groundCheck = false;

        if (collisionWorld == null || collisionWorld.getWorldOBBs() == null || collisionWorld.getWorldOBBs().isEmpty()) {
            return false;
        }

        SceneNode target = node != null ? node : rootNode;
        while (target != null && target.getParent() != null && target.getParent().getParent() != null) {
            target = target.getParent();
        }
        if (target != null && target.getParent() == null && !target.getChildren().isEmpty()) {
            target = target.getChildren().get(0);
        }

        float lowestY = Float.MAX_VALUE;
        for (OBB obb : collisionWorld.getWorldOBBs().values()) {
            float minY = obb.getMinY();
            if (minY < lowestY) {
                lowestY = minY;
            }
        }

        if (lowestY != Float.MAX_VALUE && lowestY <= 0.0f) {
            groundCheck = true;
            if (target != null && lowestY < -0.0001f) {
                target.getLocalPosition().y -= lowestY;
            }
        }

        return groundCheck;
    }

    public static boolean isGroundCheck() {
        return groundCheck;
    }
}