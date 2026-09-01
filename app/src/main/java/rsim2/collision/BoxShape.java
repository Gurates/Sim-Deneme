package rsim2.collision;

import org.joml.Matrix4f;
import org.joml.Vector3f;

public class BoxShape extends CollisionShape {
    private final AABB localAABB = new AABB();

    public BoxShape() {
    }

    public BoxShape(Vector3f min, Vector3f max) {
        this.localAABB.set(min, max);
    }

    public BoxShape(float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {
        this.localAABB.set(new Vector3f(minX, minY, minZ), new Vector3f(maxX, maxY, maxZ));
    }

    @Override
    public AABB getLocalAABB() {
        return localAABB;
    }

    public void setLocalBounds(Vector3f min, Vector3f max) {
        localAABB.set(min, max);
    }

    @Override
    public void updateWorldBounds(Matrix4f worldTransform, AABB outWorldAABB, OBB outWorldOBB) {
        if (outWorldAABB != null) {
            localAABB.transform(worldTransform, outWorldAABB);
        }
        if (outWorldOBB != null) {
            outWorldOBB.fromAABBAndTransform(localAABB, worldTransform);
        }
    }

    @Override
    public void getSupportPoint(Vector3f direction, Matrix4f worldTransform, Vector3f out) {
        float dx = direction.x;
        float dy = direction.y;
        float dz = direction.z;

        if (worldTransform != null) {
            float m00 = worldTransform.m00(), m01 = worldTransform.m01(), m02 = worldTransform.m02();
            float m10 = worldTransform.m10(), m11 = worldTransform.m11(), m12 = worldTransform.m12();
            float m20 = worldTransform.m20(), m21 = worldTransform.m21(), m22 = worldTransform.m22();

            float ldx = m00 * dx + m01 * dy + m02 * dz;
            float ldy = m10 * dx + m11 * dy + m12 * dz;
            float ldz = m20 * dx + m21 * dy + m22 * dz;

            Vector3f min = localAABB.getMin();
            Vector3f max = localAABB.getMax();

            float lx = ldx >= 0 ? max.x : min.x;
            float ly = ldy >= 0 ? max.y : min.y;
            float lz = ldz >= 0 ? max.z : min.z;

            worldTransform.transformPosition(lx, ly, lz, out);
        } else {
            Vector3f min = localAABB.getMin();
            Vector3f max = localAABB.getMax();
            out.set(dx >= 0 ? max.x : min.x,
                    dy >= 0 ? max.y : min.y,
                    dz >= 0 ? max.z : min.z);
        }
    }
}
