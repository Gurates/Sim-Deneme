package rsim2.collision;

import org.joml.Matrix4f;
import org.joml.Vector3f;

public abstract class CollisionShape {

    public abstract AABB getLocalAABB();

    public abstract void updateWorldBounds(Matrix4f worldTransform, AABB outWorldAABB, OBB outWorldOBB);

    public abstract void getSupportPoint(Vector3f direction, Matrix4f worldTransform, Vector3f out);
}

