package rsim2.editor;

import rsim2.scene.SceneNode;

public class AlignmentHelper {

    public static float getTopY(SceneNode node) {
        if (node == null || node.getMesh() == null) {
            return 0.0f;
        }
        return node.getMesh().getBoundingBoxMax().y * node.getLocalScale().y;
    }

    public static float getBottomY(SceneNode node) {
        if (node == null || node.getMesh() == null) {
            return 0.0f;
        }
        return node.getMesh().getBoundingBoxMin().y * node.getLocalScale().y;
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
        if (parent == null || parent.getMesh() == null || child.getMesh() == null) {
            child.getLocalPosition().x = 0.0f;
            child.getLocalPosition().z = 0.0f;
            return;
        }

        float parentCenterX = (parent.getMesh().getBoundingBoxMin().x + parent.getMesh().getBoundingBoxMax().x) / 2.0f * parent.getLocalScale().x;
        float parentCenterZ = (parent.getMesh().getBoundingBoxMin().z + parent.getMesh().getBoundingBoxMax().z) / 2.0f * parent.getLocalScale().z;

        float childCenterX = (child.getMesh().getBoundingBoxMin().x + child.getMesh().getBoundingBoxMax().x) / 2.0f * child.getLocalScale().x;
        float childCenterZ = (child.getMesh().getBoundingBoxMin().z + child.getMesh().getBoundingBoxMax().z) / 2.0f * child.getLocalScale().z;

        child.getLocalPosition().x = parentCenterX - childCenterX;
        child.getLocalPosition().z = parentCenterZ - childCenterZ;
    }
}
