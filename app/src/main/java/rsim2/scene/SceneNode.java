package rsim2.scene;

import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;
import rsim2.graphics.Mesh;

import java.util.ArrayList;
import java.util.List;

public class SceneNode {
    private String id;
    private final List<Mesh> meshes = new ArrayList<>();
    private String sourceMeshPath;
    private final Vector3f localPosition;
    private final Quaternionf localRotation;
    private final Vector3f localScale;
    private SceneNode parent;
    private final List<SceneNode> children;

    public SceneNode(String id) {
        this(id, (Mesh) null);
    }

    public SceneNode(String id, Mesh mesh) {
        this.id = id;
        if (mesh != null) {
            this.meshes.add(mesh);
        }
        this.localPosition = new Vector3f();
        this.localRotation = new Quaternionf();
        this.localScale = new Vector3f(1.0f, 1.0f, 1.0f);
        this.children = new ArrayList<>();
    }

    public SceneNode(String id, List<Mesh> meshes) {
        this.id = id;
        if (meshes != null) {
            this.meshes.addAll(meshes);
        }
        this.localPosition = new Vector3f();
        this.localRotation = new Quaternionf();
        this.localScale = new Vector3f(1.0f, 1.0f, 1.0f);
        this.children = new ArrayList<>();
    }

    public Matrix4f getLocalTransform() {
        return new Matrix4f().translationRotateScale(localPosition, localRotation, localScale);
    }

    public Matrix4f getWorldTransform() {
        if (parent != null) {
            return new Matrix4f(parent.getWorldTransform()).mul(getLocalTransform());
        }
        return getLocalTransform();
    }

    public void reparentPreservingWorldTransform(SceneNode newParent) {
        if (this.parent == newParent) {
            return;
        }

        Matrix4f currentWorld = new Matrix4f(this.getWorldTransform());

        if (this.parent != null) {
            this.parent.removeChild(this);
        }

        if (newParent != null) {
            newParent.addChild(this);
            Matrix4f newParentWorldInverse = new Matrix4f(newParent.getWorldTransform()).invert();
            Matrix4f newLocalTransform = newParentWorldInverse.mul(currentWorld, new Matrix4f());

            newLocalTransform.getTranslation(this.localPosition);
            newLocalTransform.getNormalizedRotation(this.localRotation);
            newLocalTransform.getScale(this.localScale);
        } else {
            currentWorld.getTranslation(this.localPosition);
            currentWorld.getNormalizedRotation(this.localRotation);
            currentWorld.getScale(this.localScale);
        }
    }

    public void setWorldPosition(Vector3f targetWorldPos) {
        if (parent != null) {
            Matrix4f parentWorldInv = new Matrix4f(parent.getWorldTransform()).invert();
            parentWorldInv.transformPosition(targetWorldPos, this.localPosition);
        } else {
            this.localPosition.set(targetWorldPos);
        }
    }

    public void addChild(SceneNode child) {
        if (child == null) return;
        child.setParent(this);
        children.add(child);
    }

    public void addChild(int index, SceneNode child) {
        if (child == null) return;
        child.setParent(this);
        if (index >= 0 && index <= children.size()) {
            children.add(index, child);
        } else {
            children.add(child);
        }
    }

    public int indexOfChild(SceneNode child) {
        return children.indexOf(child);
    }

    public boolean removeChild(SceneNode child) {
        if (child != null && children.remove(child)) {
            child.setParent(null);
            return true;
        }
        return false;
    }

    public void cleanup() {
        for (Mesh mesh : meshes) {
            if (mesh != null) {
                mesh.cleanup();
            }
        }
        for (SceneNode child : children) {
            child.cleanup();
        }
    }

    public String getId() {
        return id;
    }

    public void setId(String id) {
        this.id = id;
    }

    public List<Mesh> getMeshes() {
        return meshes;
    }

    public void addMesh(Mesh m) {
        if (m != null) {
            meshes.add(m);
        }
    }

    public void clearMeshes() {
        meshes.clear();
    }

    public Mesh getMesh() {
        return meshes.isEmpty() ? null : meshes.get(0);
    }

    public Mesh getPrimaryMesh() {
        return meshes.isEmpty() ? null : meshes.get(0);
    }

    public void setMesh(Mesh mesh) {
        meshes.clear();
        if (mesh != null) {
            meshes.add(mesh);
        }
    }

    public Vector3f getCombinedBoundingBoxMin() {
        if (meshes.isEmpty()) {
            return new Vector3f(0.0f, 0.0f, 0.0f);
        }
        Vector3f min = new Vector3f(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
        for (Mesh m : meshes) {
            if (m != null && m.getMinBound() != null) {
                min.min(m.getMinBound());
            }
        }
        return min.x == Float.MAX_VALUE ? new Vector3f(0.0f, 0.0f, 0.0f) : min;
    }

    public Vector3f getCombinedBoundingBoxMax() {
        if (meshes.isEmpty()) {
            return new Vector3f(0.0f, 0.0f, 0.0f);
        }
        Vector3f max = new Vector3f(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE);
        for (Mesh m : meshes) {
            if (m != null && m.getMaxBound() != null) {
                max.max(m.getMaxBound());
            }
        }
        return max.x == -Float.MAX_VALUE ? new Vector3f(0.0f, 0.0f, 0.0f) : max;
    }

    public Vector3f getBoundingBoxMin() {
        return getCombinedBoundingBoxMin();
    }

    public Vector3f getBoundingBoxMax() {
        return getCombinedBoundingBoxMax();
    }

    public String getSourceMeshPath() {
        return sourceMeshPath;
    }

    public void setSourceMeshPath(String sourceMeshPath) {
        this.sourceMeshPath = sourceMeshPath;
    }

    public Vector3f getLocalPosition() {
        return localPosition;
    }

    public Quaternionf getLocalRotation() {
        return localRotation;
    }

    public Vector3f getLocalScale() {
        return localScale;
    }

    public SceneNode getParent() {
        return parent;
    }

    public void setParent(SceneNode parent) {
        this.parent = parent;
    }

    public List<SceneNode> getChildren() {
        return children;
    }
}
