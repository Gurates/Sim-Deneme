package rsim2.editor;

import rsim2.scene.SceneNode;

public class AlignmentHelper {

    public static float getTopY(SceneNode node) {
        if (node == null || node.getMeshes().isEmpty()) {
            return 0.0f;
        }
        return node.getCombinedBoundingBoxMax().y * node.getLocalScale().y;
    }

    public static float getBottomY(SceneNode node) {
        if (node == null || node.getMeshes().isEmpty()) {
            return 0.0f;
        }
        return node.getCombinedBoundingBoxMin().y * node.getLocalScale().y;
    }

    public static void snapToTop(SceneNode parent, SceneNode child) {
        if (parent == null || child == null) {
            return;
        }
        float targetY = getTopY(parent) - getBottomY(child);
        child.getLocalPosition().y = targetY;
    }

    public static void snapCenterXZ(SceneNode parent, SceneNode child) {
        if (child == null) {
            return;
        }
        if (parent == null || parent.getMeshes().isEmpty() || child.getMeshes().isEmpty()) {
            child.getLocalPosition().x = 0.0f;
            child.getLocalPosition().z = 0.0f;
            return;
        }

        float parentCenterX = (parent.getCombinedBoundingBoxMin().x + parent.getCombinedBoundingBoxMax().x) / 2.0f * parent.getLocalScale().x;
        float parentCenterZ = (parent.getCombinedBoundingBoxMin().z + parent.getCombinedBoundingBoxMax().z) / 2.0f * parent.getLocalScale().z;

        float childCenterX = (child.getCombinedBoundingBoxMin().x + child.getCombinedBoundingBoxMax().x) / 2.0f * child.getLocalScale().x;
        float childCenterZ = (child.getCombinedBoundingBoxMin().z + child.getCombinedBoundingBoxMax().z) / 2.0f * child.getLocalScale().z;

        child.getLocalPosition().x = parentCenterX - childCenterX;
        child.getLocalPosition().z = parentCenterZ - childCenterZ;
    }
}
