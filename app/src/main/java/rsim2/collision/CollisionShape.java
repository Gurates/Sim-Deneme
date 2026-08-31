package rsim2.collision;

import org.joml.Matrix4f;

public abstract class CollisionShape {

    public abstract AABB getLocalAABB();

    public abstract void updateWorldBounds(Matrix4f worldTransform, AABB outWorldAABB, OBB outWorldOBB);
}
