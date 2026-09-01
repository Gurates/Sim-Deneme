package rsim2.collision;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import rsim2.graphics.Mesh;

import java.util.ArrayList;
import java.util.List;

public class ConvexMeshShape extends CollisionShape {
    private final List<Vector3f> vertices = new ArrayList<>();
    private final AABB localAABB = new AABB();

    public ConvexMeshShape() {
    }

    public ConvexMeshShape(float[] vertexArray) {
        if (vertexArray != null) {
            Vector3f min = new Vector3f(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
            Vector3f max = new Vector3f(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE);

            for (int i = 0; i < vertexArray.length; i += 3) {
                float x = vertexArray[i];
                float y = vertexArray[i + 1];
                float z = vertexArray[i + 2];
                vertices.add(new Vector3f(x, y, z));
                min.min(new Vector3f(x, y, z));
                max.max(new Vector3f(x, y, z));
            }

            if (!vertices.isEmpty()) {
                localAABB.set(min, max);
            }
        }
    }

    public ConvexMeshShape(Mesh mesh) {
        if (mesh != null && mesh.getVertices() != null) {
            float[] verts = mesh.getVertices();
            Vector3f min = new Vector3f(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
            Vector3f max = new Vector3f(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE);

            int step = Math.max(1, (verts.length / 3) / 256);
            for (int i = 0; i < verts.length; i += 3 * step) {
                float x = verts[i];
                float y = verts[i + 1];
                float z = verts[i + 2];
                vertices.add(new Vector3f(x, y, z));
                min.min(new Vector3f(x, y, z));
                max.max(new Vector3f(x, y, z));
            }

            if (mesh.getBoundingBoxMin() != null && mesh.getBoundingBoxMax() != null) {
                localAABB.set(mesh.getBoundingBoxMin(), mesh.getBoundingBoxMax());
            } else if (!vertices.isEmpty()) {
                localAABB.set(min, max);
            }
        }
    }

    public List<Vector3f> getVertices() {
        return vertices;
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
        if (vertices.isEmpty()) {
            if (worldTransform != null) {
                worldTransform.transformPosition(0, 0, 0, out);
            } else {
                out.set(0, 0, 0);
            }
            return;
        }

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

        Vector3f bestVert = vertices.get(0);
        float maxDot = bestVert.x * ldx + bestVert.y * ldy + bestVert.z * ldz;

        for (int i = 1; i < vertices.size(); i++) {
            Vector3f v = vertices.get(i);
            float dot = v.x * ldx + v.y * ldy + v.z * ldz;
            if (dot > maxDot) {
                maxDot = dot;
                bestVert = v;
            }
        }

        if (worldTransform != null) {
            worldTransform.transformPosition(bestVert, out);
        } else {
            out.set(bestVert);
        }
    }
}
