package rsim2.collision;

import org.joml.Matrix4f;
import org.joml.Vector3f;

public class SphereShape extends CollisionShape {
    private final Vector3f localCenter = new Vector3f(0, 0, 0);
    private float radius = 0.05f;
    private final AABB localAABB = new AABB();

    public SphereShape() {
        updateLocalAABB();
    }

    public SphereShape(float radius) {
        this.radius = Math.max(0.0001f, radius);
        updateLocalAABB();
    }

    public SphereShape(Vector3f localCenter, float radius) {
        if (localCenter != null) this.localCenter.set(localCenter);
        this.radius = Math.max(0.0001f, radius);
        updateLocalAABB();
    }

    private void updateLocalAABB() {
        localAABB.set(
                new Vector3f(localCenter.x - radius, localCenter.y - radius, localCenter.z - radius),
                new Vector3f(localCenter.x + radius, localCenter.y + radius, localCenter.z + radius)
        );
    }

    public float getRadius() {
        return radius;
    }

    public void setRadius(float radius) {
        this.radius = Math.max(0.0001f, radius);
        updateLocalAABB();
    }

    public Vector3f getLocalCenter() {
        return localCenter;
    }

    public void setLocalCenter(Vector3f center) {
        if (center != null) {
            this.localCenter.set(center);
            updateLocalAABB();
        }
    }

    @Override
    public AABB getLocalAABB() {
        return localAABB;
    }

    @Override
    public void updateWorldBounds(Matrix4f worldTransform, AABB outWorldAABB, OBB outWorldOBB) {
        Vector3f worldCenter = new Vector3f();
        if (worldTransform != null) {
            worldTransform.transformPosition(localCenter, worldCenter);
        } else {
            worldCenter.set(localCenter);
        }

        if (outWorldAABB != null) {
            outWorldAABB.set(
                    new Vector3f(worldCenter.x - radius, worldCenter.y - radius, worldCenter.z - radius),
                    new Vector3f(worldCenter.x + radius, worldCenter.y + radius, worldCenter.z + radius)
            );
        }

        if (outWorldOBB != null) {
            outWorldOBB.fromAABBAndTransform(localAABB, worldTransform != null ? worldTransform : new Matrix4f());
        }
    }

    @Override
    public void getSupportPoint(Vector3f direction, Matrix4f worldTransform, Vector3f out) {
        Vector3f worldCenter = new Vector3f();
        if (worldTransform != null) {
            worldTransform.transformPosition(localCenter, worldCenter);
        } else {
            worldCenter.set(localCenter);
        }

        float lenSq = direction.lengthSquared();
        if (lenSq > 0.000001f) {
            float invLen = (float) (1.0 / Math.sqrt(lenSq));
            out.set(
                    worldCenter.x + direction.x * invLen * radius,
                    worldCenter.y + direction.y * invLen * radius,
                    worldCenter.z + direction.z * invLen * radius
            );
        } else {
            out.set(worldCenter);
        }
    }
}
