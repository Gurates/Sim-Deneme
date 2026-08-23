package rsim2.editor;

import org.joml.Intersectionf;
import org.joml.Matrix4f;
import org.joml.Vector2f;
import org.joml.Vector3f;
import org.joml.Vector4f;
import rsim2.camera.Camera;
import rsim2.graphics.Mesh;
import rsim2.scene.SceneNode;

public class Picker {

    public static SceneNode pick(float mouseX, float mouseY, int screenWidth, int screenHeight, Camera camera, SceneNode root) {
        if (root == null || camera == null || screenWidth <= 0 || screenHeight <= 0) {
            return null;
        }

        float ndcX = (2.0f * mouseX) / screenWidth - 1.0f;
        float ndcY = 1.0f - (2.0f * mouseY) / screenHeight;

        Matrix4f invVP = new Matrix4f(camera.getProjectionMatrix()).mul(camera.getViewMatrix()).invert();

        Vector4f rayStartWorld = invVP.transform(new Vector4f(ndcX, ndcY, -1.0f, 1.0f));
        if (rayStartWorld.w != 0) {
            rayStartWorld.div(rayStartWorld.w);
        }

        Vector4f rayEndWorld = invVP.transform(new Vector4f(ndcX, ndcY, 1.0f, 1.0f));
        if (rayEndWorld.w != 0) {
            rayEndWorld.div(rayEndWorld.w);
        }

        Vector3f rayOrigin = new Vector3f(rayStartWorld.x, rayStartWorld.y, rayStartWorld.z);
        Vector3f rayDir = new Vector3f(rayEndWorld.x - rayStartWorld.x, rayEndWorld.y - rayStartWorld.y, rayEndWorld.z - rayStartWorld.z).normalize();

        float[] minDistance = new float[]{ Float.MAX_VALUE };
        SceneNode[] bestNode = new SceneNode[1];

        traverseAndPick(root, rayOrigin, rayDir, minDistance, bestNode);

        return bestNode[0];
    }

    public static Vector3f raycastMeshSurface(float mouseX, float mouseY, int screenWidth, int screenHeight, Camera camera, SceneNode targetNode) {
        if (targetNode == null || targetNode.getMeshes().isEmpty() || camera == null || screenWidth <= 0 || screenHeight <= 0) {
            return null;
        }

        float ndcX = (2.0f * mouseX) / screenWidth - 1.0f;
        float ndcY = 1.0f - (2.0f * mouseY) / screenHeight;

        Matrix4f invVP = new Matrix4f(camera.getProjectionMatrix()).mul(camera.getViewMatrix()).invert();

        Vector4f rayStartWorld = invVP.transform(new Vector4f(ndcX, ndcY, -1.0f, 1.0f));
        if (rayStartWorld.w != 0) {
            rayStartWorld.div(rayStartWorld.w);
        }

        Vector4f rayEndWorld = invVP.transform(new Vector4f(ndcX, ndcY, 1.0f, 1.0f));
        if (rayEndWorld.w != 0) {
            rayEndWorld.div(rayEndWorld.w);
        }

        Vector3f rayOrigin = new Vector3f(rayStartWorld.x, rayStartWorld.y, rayStartWorld.z);
        Vector3f rayDir = new Vector3f(rayEndWorld.x - rayStartWorld.x, rayEndWorld.y - rayStartWorld.y, rayEndWorld.z - rayStartWorld.z).normalize();

        Matrix4f invWorld = new Matrix4f(targetNode.getWorldTransform()).invert();

        Vector4f locOrig = invWorld.transform(new Vector4f(rayOrigin, 1.0f));
        Vector3f localRayOrigin = new Vector3f(locOrig.x, locOrig.y, locOrig.z);

        Vector4f locDir = invWorld.transform(new Vector4f(rayDir, 0.0f));
        Vector3f localRayDir = new Vector3f(locDir.x, locDir.y, locDir.z).normalize();

        float minT = Float.MAX_VALUE;
        Vector3f v0 = new Vector3f();
        Vector3f v1 = new Vector3f();
        Vector3f v2 = new Vector3f();

        for (Mesh mesh : targetNode.getMeshes()) {
            if (mesh == null) continue;

            float[] vertices = mesh.getVertices();
            int[] indices = mesh.getIndices();

            if (vertices == null || indices == null || indices.length < 3) {
                continue;
            }

            for (int i = 0; i < indices.length; i += 3) {
                int i0 = indices[i] * 3;
                int i1 = indices[i + 1] * 3;
                int i2 = indices[i + 2] * 3;

                if (i0 + 2 >= vertices.length || i1 + 2 >= vertices.length || i2 + 2 >= vertices.length) {
                    continue;
                }

                v0.set(vertices[i0], vertices[i0 + 1], vertices[i0 + 2]);
                v1.set(vertices[i1], vertices[i1 + 1], vertices[i1 + 2]);
                v2.set(vertices[i2], vertices[i2 + 1], vertices[i2 + 2]);

                float t = rayTriangleIntersect(localRayOrigin, localRayDir, v0, v1, v2);
                if (t > 0.0f && t < minT) {
                    minT = t;
                }
            }
        }

        if (minT < Float.MAX_VALUE) {
            Vector3f localHit = new Vector3f(localRayOrigin).add(new Vector3f(localRayDir).mul(minT));
            Vector3f worldHit = new Vector3f();
            targetNode.getWorldTransform().transformPosition(localHit, worldHit);
            return worldHit;
        }

        return null;
    }

    private static float rayTriangleIntersect(Vector3f orig, Vector3f dir, Vector3f v0, Vector3f v1, Vector3f v2) {
        float EPSILON = 1e-7f;
        Vector3f edge1 = new Vector3f(v1).sub(v0);
        Vector3f edge2 = new Vector3f(v2).sub(v0);
        Vector3f pvec = new Vector3f(dir).cross(edge2);
        float det = edge1.dot(pvec);

        if (det > -EPSILON && det < EPSILON) return -1.0f;
        float invDet = 1.0f / det;

        Vector3f tvec = new Vector3f(orig).sub(v0);
        float u = tvec.dot(pvec) * invDet;
        if (u < 0.0f || u > 1.0f) return -1.0f;

        Vector3f qvec = new Vector3f(tvec).cross(edge1);
        float v = dir.dot(qvec) * invDet;
        if (v < 0.0f || u + v > 1.0f) return -1.0f;

        float t = edge2.dot(qvec) * invDet;
        return (t > EPSILON) ? t : -1.0f;
    }

    private static void traverseAndPick(SceneNode node, Vector3f rayOrigin, Vector3f rayDir, float[] minDistance, SceneNode[] bestNode) {
        if (node == null) return;

        if (!node.getMeshes().isEmpty()) {
            Vector3f minBound = node.getCombinedBoundingBoxMin();
            Vector3f maxBound = node.getCombinedBoundingBoxMax();

            Matrix4f invWorld = new Matrix4f(node.getWorldTransform()).invert();

            Vector4f locOrig = invWorld.transform(new Vector4f(rayOrigin, 1.0f));
            Vector3f localOrigin = new Vector3f(locOrig.x, locOrig.y, locOrig.z);

            Vector4f locDir = invWorld.transform(new Vector4f(rayDir, 0.0f));
            Vector3f localDir = new Vector3f(locDir.x, locDir.y, locDir.z).normalize();

            Vector2f result = new Vector2f();
            if (Intersectionf.intersectRayAab(localOrigin, localDir, minBound, maxBound, result)) {
                float dist = result.x;
                if (dist < 0) dist = result.y;
                if (dist >= 0 && dist < minDistance[0]) {
                    minDistance[0] = dist;
                    bestNode[0] = node;
                }
            }
        }

        for (SceneNode child : node.getChildren()) {
            traverseAndPick(child, rayOrigin, rayDir, minDistance, bestNode);
        }
    }
}
