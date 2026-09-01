package rsim2.collision;

import org.joml.Matrix4f;
import org.joml.Vector3f;

public class CylinderShape extends CollisionShape {
    private float radius = 0.05f;
    private float height = 0.1f;
    private final AABB localAABB = new AABB();

    public CylinderShape() {
        updateLocalAABB();
    }

    public CylinderShape(float radius, float height) {
        this.radius = Math.max(0.0001f, radius);
        this.height = Math.max(0.0001f, height);
        updateLocalAABB();
    }

    private void updateLocalAABB() {
        float halfH = height * 0.5f;
        localAABB.set(new Vector3f(-radius, -radius, -halfH), new Vector3f(radius, radius, halfH));
    }

    public float getRadius() {
        return radius;
    }

    public void setRadius(float radius) {
        this.radius = Math.max(0.0001f, radius);
        updateLocalAABB();
    }

    public float getHeight() {
        return height;
    }

    public void setHeight(float height) {
        this.height = Math.max(0.0001f, height);
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
        float dx = direction.x;
        float dy = direction.y;
        float dz = direction.z;

        float ldx = dx, ldy = dy, ldz = dz;
        if (worldTransform != null) {
            float m00 = worldTransform.m00(), m01 = worldTransform.m01(), m02 = worldTransform.m02();
            float m10 = worldTransform.m10(), m11 = worldTransform.m11(), m12 = worldTransform.m12();
            float m20 = worldTransform.m20(), m21 = worldTransform.m21(), m22 = worldTransform.m22();

            ldx = m00 * dx + m01 * dy + m02 * dz;
            ldy = m10 * dx + m11 * dy + m12 * dz;
            ldz = m20 * dx + m21 * dy + m22 * dz;
        }

        float halfH = height * 0.5f;
        float lz = ldz >= 0 ? halfH : -halfH;

        float radialLenSq = ldx * ldx + ldy * ldy;
        float lx = 0, ly = 0;
        if (radialLenSq > 0.000001f) {
            float invRad = (float) (radius / Math.sqrt(radialLenSq));
            lx = ldx * invRad;
            ly = ldy * invRad;
        }

        if (worldTransform != null) {
            worldTransform.transformPosition(lx, ly, lz, out);
        } else {
            out.set(lx, ly, lz);
        }
    }
}
