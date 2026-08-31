package rsim2.collision;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.joml.Vector4f;

public class OBB {
    private final Vector3f center = new Vector3f();
    private final Vector3f[] axes = new Vector3f[]{
            new Vector3f(1.0f, 0.0f, 0.0f),
            new Vector3f(0.0f, 1.0f, 0.0f),
            new Vector3f(0.0f, 0.0f, 1.0f)
    };
    private final Vector3f halfExtents = new Vector3f(0.5f, 0.5f, 0.5f);

    public OBB() {
    }

    public OBB(Vector3f center, Vector3f[] axes, Vector3f halfExtents) {
        if (center != null) this.center.set(center);
        if (axes != null && axes.length >= 3) {
            this.axes[0].set(axes[0]);
            this.axes[1].set(axes[1]);
            this.axes[2].set(axes[2]);
        }
        if (halfExtents != null) this.halfExtents.set(halfExtents);
    }

    public Vector3f getCenter() {
        return center;
    }

    public Vector3f[] getAxes() {
        return axes;
    }

    public Vector3f getAxis(int index) {
        return axes[index];
    }

    public Vector3f getHalfExtents() {
        return halfExtents;
    }

    public void fromAABBAndTransform(AABB localAABB, Matrix4f worldTransform) {
        if (localAABB == null || worldTransform == null || !localAABB.isValid()) {
            return;
        }

        Vector3f localCenter = localAABB.getCenter();
        Vector3f localHalfExtents = localAABB.getHalfExtents();

        worldTransform.transformPosition(localCenter, this.center);

        Vector3f col0 = new Vector3f();
        Vector3f col1 = new Vector3f();
        Vector3f col2 = new Vector3f();

        worldTransform.getColumn(0, col0);
        worldTransform.getColumn(1, col1);
        worldTransform.getColumn(2, col2);

        float scaleX = col0.length();
        float scaleY = col1.length();
        float scaleZ = col2.length();

        if (scaleX > 1e-6f) axes[0].set(col0).mul(1.0f / scaleX);
        else axes[0].set(1, 0, 0);

        if (scaleY > 1e-6f) axes[1].set(col1).mul(1.0f / scaleY);
        else axes[1].set(0, 1, 0);

        if (scaleZ > 1e-6f) axes[2].set(col2).mul(1.0f / scaleZ);
        else axes[2].set(0, 0, 1);

        halfExtents.set(
                Math.abs(localHalfExtents.x * scaleX),
                Math.abs(localHalfExtents.y * scaleY),
                Math.abs(localHalfExtents.z * scaleZ)
        );
    }

    public Vector3f[] getCorners() {
        Vector3f[] corners = new Vector3f[8];
        Vector3f e0 = new Vector3f(axes[0]).mul(halfExtents.x);
        Vector3f e1 = new Vector3f(axes[1]).mul(halfExtents.y);
        Vector3f e2 = new Vector3f(axes[2]).mul(halfExtents.z);

        corners[0] = new Vector3f(center).sub(e0).sub(e1).sub(e2);
        corners[1] = new Vector3f(center).add(e0).sub(e1).sub(e2);
        corners[2] = new Vector3f(center).add(e0).add(e1).sub(e2);
        corners[3] = new Vector3f(center).sub(e0).add(e1).sub(e2);

        corners[4] = new Vector3f(center).sub(e0).sub(e1).add(e2);
        corners[5] = new Vector3f(center).add(e0).sub(e1).add(e2);
        corners[6] = new Vector3f(center).add(e0).add(e1).add(e2);
        corners[7] = new Vector3f(center).sub(e0).add(e1).add(e2);

        return corners;
    }

    public float getMinY() {
        float r = halfExtents.x * Math.abs(axes[0].y) +
                  halfExtents.y * Math.abs(axes[1].y) +
                  halfExtents.z * Math.abs(axes[2].y);
        return center.y - r;
    }

    public boolean intersectsGround(float groundY) {
        return getMinY() <= groundY;
    }

    public boolean intersects(OBB other) {
        return CollisionMath.intersectOBBOBB(this, other);
    }

    @Override
    public String toString() {
        return String.format("OBB[center=(%.2f, %.2f, %.2f), extents=(%.2f, %.2f, %.2f)]",
                center.x, center.y, center.z, halfExtents.x, halfExtents.y, halfExtents.z);
    }
}
