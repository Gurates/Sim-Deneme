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

    private static void traverseAndPick(SceneNode node, Vector3f rayOrigin, Vector3f rayDir, float[] minDistance, SceneNode[] bestNode) {
        if (node == null) return;

        Mesh mesh = node.getMesh();
        if (mesh != null && mesh.getMinBound() != null && mesh.getMaxBound() != null) {
            Matrix4f invWorld = new Matrix4f(node.getWorldTransform()).invert();

            Vector4f locOrig = invWorld.transform(new Vector4f(rayOrigin, 1.0f));
            Vector3f localOrigin = new Vector3f(locOrig.x, locOrig.y, locOrig.z);

            Vector4f locDir = invWorld.transform(new Vector4f(rayDir, 0.0f));
            Vector3f localDir = new Vector3f(locDir.x, locDir.y, locDir.z).normalize();

            Vector2f result = new Vector2f();
            if (Intersectionf.intersectRayAab(localOrigin, localDir, mesh.getMinBound(), mesh.getMaxBound(), result)) {
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
