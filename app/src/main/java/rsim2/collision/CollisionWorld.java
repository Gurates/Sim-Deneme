package rsim2.collision;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import rsim2.motion.MotionSequence;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.util.*;

public class CollisionWorld {

    private final CollisionFilter filter = new CollisionFilter();
    private final CollisionResult lastResult = new CollisionResult();

    private final Map<SceneNode, CollisionShape> nodeShapes = new HashMap<>();
    private final Map<SceneNode, AABB> worldAABBs = new HashMap<>();
    private final Map<SceneNode, OBB> worldOBBs = new HashMap<>();

    private boolean debugWireframesEnabled = false;

    public CollisionWorld() {
    }

    public CollisionFilter getFilter() {
        return filter;
    }

    public CollisionResult getLastResult() {
        return lastResult;
    }

    public boolean isDebugWireframesEnabled() {
        return debugWireframesEnabled;
    }

    public void setDebugWireframesEnabled(boolean enabled) {
        this.debugWireframesEnabled = enabled;
    }

    public Map<SceneNode, OBB> getWorldOBBs() {
        return worldOBBs;
    }

    public Map<SceneNode, CollisionShape> getNodeShapes() {
        return nodeShapes;
    }

    public CollisionResult update(SceneNode rootNode, List<Joint> joints) {
        long startTimeNanos = System.nanoTime();
        lastResult.clear();

        if (!filter.isEnabled() || rootNode == null) {
            lastResult.setComputationTimeNanos(System.nanoTime() - startTimeNanos);
            return lastResult;
        }

        List<SceneNode> meshNodes = new ArrayList<>();
        collectMeshNodes(rootNode, meshNodes);

        if (meshNodes.isEmpty()) {
            lastResult.setComputationTimeNanos(System.nanoTime() - startTimeNanos);
            return lastResult;
        }

        for (SceneNode node : meshNodes) {
            CollisionShape shape = node.getCollisionShape();
            if (shape == null) {
                shape = nodeShapes.computeIfAbsent(node, n -> {
                    if (!n.getMeshes().isEmpty()) {
                        return new ConvexMeshShape(n.getMeshes().get(0));
                    }
                    BoxShape bs = new BoxShape();
                    bs.setLocalBounds(n.getCombinedBoundingBoxMin(), n.getCombinedBoundingBoxMax());
                    return bs;
                });
            } else {
                nodeShapes.put(node, shape);
            }

            AABB worldAABB = worldAABBs.computeIfAbsent(node, n -> new AABB());
            OBB worldOBB = worldOBBs.computeIfAbsent(node, n -> new OBB());

            Matrix4f worldTransform = node.getWorldTransform();
            shape.updateWorldBounds(worldTransform, worldAABB, worldOBB);
        }

        if (filter.isGroundCollisionEnabled()) {
            float groundY = filter.getGroundHeight();
            for (SceneNode node : meshNodes) {
                if (!filter.shouldCheckGround(node)) continue;

                OBB obb = worldOBBs.get(node);
                if (obb != null && obb.intersectsGround(groundY)) {
                    float penetration = groundY - obb.getMinY();
                    Vector3f contactPt = new Vector3f(obb.getCenter().x, groundY, obb.getCenter().z);
                    lastResult.addContact(ContactPair.createGroundCollision(node, contactPt, penetration));
                }
            }
        }

        if (filter.isSelfCollisionEnabled()) {
            float margin = filter.getCollisionMargin();
            int n = meshNodes.size();
            for (int i = 0; i < n; i++) {
                SceneNode nodeA = meshNodes.get(i);
                AABB aabbA = worldAABBs.get(nodeA);
                OBB obbA = worldOBBs.get(nodeA);
                CollisionShape shapeA = nodeShapes.get(nodeA);

                for (int j = i + 1; j < n; j++) {
                    SceneNode nodeB = meshNodes.get(j);

                    if (!filter.shouldCheckPair(nodeA, nodeB, joints)) {
                        continue;
                    }

                    AABB aabbB = worldAABBs.get(nodeB);
                    OBB obbB = worldOBBs.get(nodeB);
                    CollisionShape shapeB = nodeShapes.get(nodeB);

                    if (aabbA == null || aabbB == null || obbA == null || obbB == null) {
                        continue;
                    }

                    if (!aabbA.intersects(aabbB)) {
                        continue;
                    }

                    if (shapeA != null && shapeB != null) {
                        if (CollisionMath.intersectShapes(shapeA, nodeA.getWorldTransform(),
                                shapeB, nodeB.getWorldTransform(), margin)) {
                            Vector3f contactPoint = new Vector3f(obbA.getCenter()).add(obbB.getCenter()).mul(0.5f);
                            lastResult.addContact(ContactPair.createSelfCollision(nodeA, nodeB, contactPoint));
                        }
                    } else if (CollisionMath.intersectOBBOBB(obbA, obbB)) {
                        Vector3f contactPoint = new Vector3f(obbA.getCenter()).add(obbB.getCenter()).mul(0.5f);
                        lastResult.addContact(ContactPair.createSelfCollision(nodeA, nodeB, contactPoint));
                    }
                }
            }
        }

        lastResult.setComputationTimeNanos(System.nanoTime() - startTimeNanos);
        return lastResult;
    }

    public boolean validateTrajectory(MotionSequence sequence, List<Joint> joints, SceneNode rootNode,
                                      float stepSec, List<String> outLogs) {
        if (sequence == null || joints == null || rootNode == null || !sequence.hasKeyframes()) {
            return true;
        }

        Map<String, Float> originalAngles = new HashMap<>();
        for (Joint j : joints) {
            if (j != null && j.getId() != null) {
                originalAngles.put(j.getId(), j.getCurrentAngleRadians());
            }
        }

        boolean collisionFree = true;
        float duration = Math.max(0.01f, sequence.getDurationSeconds());
        float dt = Math.max(0.01f, stepSec);

        Map<String, Float> sampledDeg = new HashMap<>();

        try {
            for (float t = 0.0f; t <= duration + 1e-4f; t += dt) {
                sequence.sample(t, sampledDeg);

                for (Map.Entry<String, Float> entry : sampledDeg.entrySet()) {
                    Joint j = findJoint(joints, entry.getKey());
                    if (j != null) {
                        float rad = (float) Math.toRadians(entry.getValue());
                        j.setAngle(rad);
                    }
                }

                CollisionResult res = update(rootNode, joints);
                if (res.hasCollision()) {
                    collisionFree = false;
                    if (outLogs != null) {
                        for (ContactPair contact : res.getContacts()) {
                            outLogs.add(String.format("Time %.2fs: Collision [%s]", t, contact.getDescription()));
                        }
                    }
                }
            }
        } finally {
            for (Joint j : joints) {
                if (j != null && j.getId() != null && originalAngles.containsKey(j.getId())) {
                    j.setAngle(originalAngles.get(j.getId()));
                }
            }
            update(rootNode, joints);
        }

        return collisionFree;
    }

    private Joint findJoint(List<Joint> joints, String jointId) {
        if (joints == null || jointId == null) return null;
        for (Joint j : joints) {
            if (j.getId().equalsIgnoreCase(jointId)) return j;
        }
        for (Joint j : joints) {
            if (j.getId().toLowerCase().contains(jointId.toLowerCase()) ||
                jointId.toLowerCase().contains(j.getId().toLowerCase())) {
                return j;
            }
        }
        return null;
    }

    private void collectMeshNodes(SceneNode node, List<SceneNode> collected) {
        if (node == null) return;
        if (!node.getMeshes().isEmpty()) {
            collected.add(node);
        }
        for (SceneNode child : node.getChildren()) {
            collectMeshNodes(child, collected);
        }
    }
}
