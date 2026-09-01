package rsim2.collision;

import org.joml.Matrix4f;
import org.joml.Vector3f;

public class CapsuleShape extends CollisionShape {
    private final Vector3f localP0 = new Vector3f(0, 0, -0.05f);
    private final Vector3f localP1 = new Vector3f(0, 0, 0.05f);
    private float radius = 0.02f;
    private final AABB localAABB = new AABB();

    public CapsuleShape() {
        updateLocalAABB();
    }

    public CapsuleShape(float radius, float length) {
        this.radius = Math.max(0.0001f, radius);
        float halfLen = Math.max(0.0f, length * 0.5f);
        this.localP0.set(0, 0, -halfLen);
        this.localP1.set(0, 0, halfLen);
        updateLocalAABB();
    }

    public CapsuleShape(Vector3f p0, Vector3f p1, float radius) {
        if (p0 != null) this.localP0.set(p0);
        if (p1 != null) this.localP1.set(p1);
        this.radius = Math.max(0.0001f, radius);
        updateLocalAABB();
    }

    private void updateLocalAABB() {
        float minX = Math.min(localP0.x, localP1.x) - radius;
        float minY = Math.min(localP0.y, localP1.y) - radius;
        float minZ = Math.min(localP0.z, localP1.z) - radius;

        float maxX = Math.max(localP0.x, localP1.x) + radius;
        float maxY = Math.max(localP0.y, localP1.y) + radius;
        float maxZ = Math.max(localP0.z, localP1.z) + radius;

        localAABB.set(new Vector3f(minX, minY, minZ), new Vector3f(maxX, maxY, maxZ));
    }

    public float getRadius() {
        return radius;
    }

    public void setRadius(float radius) {
        this.radius = Math.max(0.0001f, radius);
        updateLocalAABB();
    }

    public Vector3f getLocalP0() {
        return localP0;
    }

    public Vector3f getLocalP1() {
        return localP1;
    }

    public void setEndpoints(Vector3f p0, Vector3f p1) {
        if (p0 != null) this.localP0.set(p0);
        if (p1 != null) this.localP1.set(p1);
        updateLocalAABB();
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
        Vector3f wP0 = new Vector3f();
        Vector3f wP1 = new Vector3f();

        if (worldTransform != null) {
            worldTransform.transformPosition(localP0, wP0);
            worldTransform.transformPosition(localP1, wP1);
        } else {
            wP0.set(localP0);
            wP1.set(localP1);
        }

        float dot0 = wP0.dot(direction);
        float dot1 = wP1.dot(direction);
        Vector3f bestEnd = (dot0 > dot1) ? wP0 : wP1;

        float lenSq = direction.lengthSquared();
        if (lenSq > 0.000001f) {
            float invLen = (float) (1.0 / Math.sqrt(lenSq));
            out.set(
                    bestEnd.x + direction.x * invLen * radius,
                    bestEnd.y + direction.y * invLen * radius,
                    bestEnd.z + direction.z * invLen * radius
            );
        } else {
            out.set(bestEnd);
        }
    }
}
