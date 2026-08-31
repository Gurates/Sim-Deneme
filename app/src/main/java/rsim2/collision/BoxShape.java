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
}
