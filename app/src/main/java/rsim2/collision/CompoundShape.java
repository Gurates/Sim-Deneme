package rsim2.collision;

import org.joml.Matrix4f;
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class CompoundShape extends CollisionShape {

    public static class Entry {
        private final CollisionShape shape;
        private final Matrix4f localTransform;
        private final Matrix4f tempWorldTransform = new Matrix4f();
        private final AABB tempWorldAABB = new AABB();
        private final OBB tempWorldOBB = new OBB();

        public Entry(CollisionShape shape, Matrix4f localTransform) {
            this.shape = shape;
            this.localTransform = localTransform != null ? new Matrix4f(localTransform) : new Matrix4f();
        }

        public CollisionShape getShape() {
            return shape;
        }

        public Matrix4f getLocalTransform() {
            return localTransform;
        }

        public Matrix4f computeWorldTransform(Matrix4f parentWorldTransform) {
            if (parentWorldTransform != null) {
                parentWorldTransform.mul(localTransform, tempWorldTransform);
            } else {
                tempWorldTransform.set(localTransform);
            }
            return tempWorldTransform;
        }

        public AABB computeWorldAABB(Matrix4f parentWorldTransform) {
            Matrix4f wTransform = computeWorldTransform(parentWorldTransform);
            shape.updateWorldBounds(wTransform, tempWorldAABB, tempWorldOBB);
            return tempWorldAABB;
        }
    }

    private final List<Entry> children = new ArrayList<>();
    private final AABB localAABB = new AABB();

    public CompoundShape() {
    }

    public void addShape(CollisionShape shape, Matrix4f localTransform) {
        if (shape != null) {
            children.add(new Entry(shape, localTransform));
            updateLocalAABB();
        }
    }

    public void addShape(CollisionShape shape) {
        addShape(shape, new Matrix4f());
    }

    public List<Entry> getChildren() {
        return children;
    }

    public boolean isEmpty() {
        return children.isEmpty();
    }

    public int size() {
        return children.size();
    }

    public void clear() {
        children.clear();
        localAABB.clear();
    }

    public void updateLocalAABB() {
        localAABB.clear();
        AABB childAABB = new AABB();
        OBB childOBB = new OBB();

        for (Entry entry : children) {
            entry.getShape().updateWorldBounds(entry.getLocalTransform(), childAABB, childOBB);
            localAABB.union(childAABB);
        }
    }

    @Override
    public AABB getLocalAABB() {
        return localAABB;
    }

    @Override
    public void updateWorldBounds(Matrix4f worldTransform, AABB outWorldAABB, OBB outWorldOBB) {
        if (outWorldAABB != null) {
            localAABB.transform(worldTransform, outWorldAABB);
        }
        if (outWorldOBB != null) {
            outWorldOBB.fromAABBAndTransform(localAABB, worldTransform != null ? worldTransform : new Matrix4f());
        }
    }

    @Override
    public void getSupportPoint(Vector3f direction, Matrix4f worldTransform, Vector3f out) {
        if (children.isEmpty()) {
            if (worldTransform != null) {
                worldTransform.transformPosition(0, 0, 0, out);
            } else {
                out.set(0, 0, 0);
            }
            return;
        }

        Vector3f tempPt = new Vector3f();
        Vector3f bestPt = new Vector3f();
        float maxDot = -Float.MAX_VALUE;

        for (Entry entry : children) {
            Matrix4f childWorld = entry.computeWorldTransform(worldTransform);
            entry.getShape().getSupportPoint(direction, childWorld, tempPt);
            float dot = tempPt.dot(direction);
            if (dot > maxDot) {
                maxDot = dot;
                bestPt.set(tempPt);
            }
        }

        out.set(bestPt);
    }
}
