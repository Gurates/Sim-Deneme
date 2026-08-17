package rsim2.editor;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import rsim2.camera.Camera;
import rsim2.scene.SceneNode;

public class PointAlignTool {
    private SceneNode parentNode;
    private SceneNode childNode;

    private Vector3f pickedParentPointWorld;
    private Vector3f pickedChildPointWorld;

    private boolean pickingParentPoint = false;
    private boolean pickingChildPoint = false;

    public void setNodes(SceneNode parentNode, SceneNode childNode) {
        if (this.parentNode != parentNode || this.childNode != childNode) {
            this.parentNode = parentNode;
            this.childNode = childNode;
        }
    }

    public void startPickParentPoint() {
        pickingParentPoint = true;
        pickingChildPoint = false;
    }

    public void startPickChildPoint() {
        pickingChildPoint = true;
        pickingParentPoint = false;
    }

    public void cancelPicking() {
        pickingParentPoint = false;
        pickingChildPoint = false;
    }

    public boolean isPicking() {
        return pickingParentPoint || pickingChildPoint;
    }

    public boolean isPickingParent() {
        return pickingParentPoint;
    }

    public boolean isPickingChild() {
        return pickingChildPoint;
    }

    public void onSceneClick(float mouseX, float mouseY, Camera camera, int screenWidth, int screenHeight) {
        if (pickingParentPoint && parentNode != null) {
            Vector3f hit = Picker.raycastMeshSurface(mouseX, mouseY, screenWidth, screenHeight, camera, parentNode);
            if (hit != null) {
                pickedParentPointWorld = hit;
                pickingParentPoint = false;
            }
        } else if (pickingChildPoint && childNode != null) {
            Vector3f hit = Picker.raycastMeshSurface(mouseX, mouseY, screenWidth, screenHeight, camera, childNode);
            if (hit != null) {
                pickedChildPointWorld = hit;
                pickingChildPoint = false;
            }
        }
    }

    public boolean canAlign() {
        return pickedParentPointWorld != null && pickedChildPointWorld != null && parentNode != null && childNode != null;
    }

    public void align() {
        if (!canAlign()) {
            return;
        }

        Vector3f childWorldOrigin = new Vector3f();
        childNode.getWorldTransform().getTranslation(childWorldOrigin);

        Vector3f offsetVec = new Vector3f(pickedChildPointWorld).sub(childWorldOrigin);
        Vector3f desiredChildWorldOrigin = new Vector3f(pickedParentPointWorld).sub(offsetVec);

        if (childNode.getParent() == parentNode) {
            Matrix4f parentInv = new Matrix4f(parentNode.getWorldTransform()).invert();
            Vector3f newLocalPos = new Vector3f();
            parentInv.transformPosition(desiredChildWorldOrigin, newLocalPos);
            childNode.getLocalPosition().set(newLocalPos);
        } else if (childNode.getParent() != null) {
            Matrix4f curParentInv = new Matrix4f(childNode.getParent().getWorldTransform()).invert();
            Vector3f newLocalPos = new Vector3f();
            curParentInv.transformPosition(desiredChildWorldOrigin, newLocalPos);
            childNode.getLocalPosition().set(newLocalPos);
        } else {
            childNode.getLocalPosition().set(desiredChildWorldOrigin);
        }

        clearPoints();
    }

    public void clearPoints() {
        pickedParentPointWorld = null;
        pickedChildPointWorld = null;
        pickingParentPoint = false;
        pickingChildPoint = false;
    }

    public SceneNode getParentNode() {
        return parentNode;
    }

    public SceneNode getChildNode() {
        return childNode;
    }

    public Vector3f getPickedParentPointWorld() {
        return pickedParentPointWorld;
    }

    public Vector3f getPickedChildPointWorld() {
        return pickedChildPointWorld;
    }
}
