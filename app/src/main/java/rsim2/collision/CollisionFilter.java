package rsim2.collision;

import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class CollisionFilter {
    private boolean enabled = true;
    private boolean selfCollisionEnabled = true;
    private boolean groundCollisionEnabled = true;
    private boolean ignoreAdjacentJoints = true;
    private int jointAdjacencyDepth = 2;
    private float groundHeight = 0.0f;
    private float collisionMargin = 0.001f;
    private boolean autoAcmComputed = false;

    private final Set<String> customIgnoredPairs = new HashSet<>();

    public CollisionFilter() {
    }

    public static String getPairKey(String idA, String idB) {
        if (idA == null || idB == null) return "";
        return idA.compareTo(idB) < 0 ? idA + "::" + idB : idB + "::" + idA;
    }

    public void ignorePair(String idA, String idB) {
        if (idA != null && idB != null) {
            customIgnoredPairs.add(getPairKey(idA, idB));
        }
    }

    public void unignorePair(String idA, String idB) {
        if (idA != null && idB != null) {
            customIgnoredPairs.remove(getPairKey(idA, idB));
        }
    }

    public boolean isPairIgnored(String idA, String idB) {
        if (idA == null || idB == null) return true;
        return customIgnoredPairs.contains(getPairKey(idA, idB));
    }

    public void loadDisabledPairs(List<String[]> pairs) {
        if (pairs == null) return;
        for (String[] pair : pairs) {
            if (pair != null && pair.length >= 2) {
                ignorePair(pair[0], pair[1]);
            }
        }
    }

    public void computeAutoAcm(SceneNode root, List<Joint> joints, CollisionWorld world) {
        if (root == null || world == null) return;
        CollisionResult rawResult = world.update(root, joints);
        for (ContactPair contact : rawResult.getContacts()) {
            if (contact.getNodeA() != null && contact.getNodeB() != null) {
                ignorePair(contact.getNodeA().getId(), contact.getNodeB().getId());
            }
        }
        this.autoAcmComputed = true;
    }

    public void ignoreCurrentContacts(CollisionResult result) {
        if (result == null) return;
        for (ContactPair contact : result.getContacts()) {
            if (contact.getNodeA() != null && contact.getNodeB() != null) {
                ignorePair(contact.getNodeA().getId(), contact.getNodeB().getId());
            }
        }
    }

    public void clearIgnoredPairs() {
        customIgnoredPairs.clear();
        autoAcmComputed = false;
    }

    public Set<String> getCustomIgnoredPairs() {
        return customIgnoredPairs;
    }

    public float getCollisionMargin() {
        return collisionMargin;
    }

    public void setCollisionMargin(float collisionMargin) {
        this.collisionMargin = Math.max(0.0f, Math.min(0.05f, collisionMargin));
    }

    public boolean isAutoAcmComputed() {
        return autoAcmComputed;
    }

    public boolean shouldCheckPair(SceneNode a, SceneNode b, List<Joint> joints) {
        if (!enabled || !selfCollisionEnabled) return false;
        if (a == null || b == null || a == b) return false;
        if (a.getMeshes().isEmpty() || b.getMeshes().isEmpty()) return false;

        String idA = a.getId();
        String idB = b.getId();

        if (customIgnoredPairs.contains(getPairKey(idA, idB))) {
            return false;
        }

        if (ignoreAdjacentJoints && joints != null) {
            if (areJointConnected(a, b, joints, jointAdjacencyDepth)) {
                return false;
            }
        }

        return true;
    }

    public static boolean areJointConnected(SceneNode a, SceneNode b, List<Joint> joints, int maxDepth) {
        if (joints == null || maxDepth <= 0 || a == null || b == null) return false;

        for (Joint j1 : joints) {
            if (j1 == null) continue;
            SceneNode p1 = j1.getParentNode();
            SceneNode c1 = j1.getChildNode();
            if ((p1 == a && c1 == b) || (p1 == b && c1 == a)) {
                return true;
            }
        }

        if (maxDepth < 2) return false;

        for (Joint j1 : joints) {
            if (j1 == null) continue;
            SceneNode p1 = j1.getParentNode();
            SceneNode c1 = j1.getChildNode();

            for (Joint j2 : joints) {
                if (j2 == null || j1 == j2) continue;
                SceneNode p2 = j2.getParentNode();
                SceneNode c2 = j2.getChildNode();

                if (p1 != null && p1 == p2 && ((c1 == a && c2 == b) || (c1 == b && c2 == a))) {
                    return true;
                }

                SceneNode intermediate = (p1 == a) ? c1 : ((c1 == a) ? p1 : null);
                if (intermediate != null) {
                    if ((p2 == intermediate && c2 == b) || (c2 == intermediate && p2 == b)) {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    public boolean shouldCheckGround(SceneNode a) {
        if (!enabled || !groundCollisionEnabled || a == null) return false;
        if (a.getMeshes().isEmpty()) return false;

        return true;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isSelfCollisionEnabled() {
        return selfCollisionEnabled;
    }

    public void setSelfCollisionEnabled(boolean selfCollisionEnabled) {
        this.selfCollisionEnabled = selfCollisionEnabled;
    }

    public boolean isGroundCollisionEnabled() {
        return groundCollisionEnabled;
    }

    public void setGroundCollisionEnabled(boolean groundCollisionEnabled) {
        this.groundCollisionEnabled = groundCollisionEnabled;
    }

    public boolean isIgnoreAdjacentJoints() {
        return ignoreAdjacentJoints;
    }

    public void setIgnoreAdjacentJoints(boolean ignoreAdjacentJoints) {
        this.ignoreAdjacentJoints = ignoreAdjacentJoints;
    }

    public int getJointAdjacencyDepth() {
        return jointAdjacencyDepth;
    }

    public void setJointAdjacencyDepth(int jointAdjacencyDepth) {
        this.jointAdjacencyDepth = Math.max(0, Math.min(4, jointAdjacencyDepth));
    }

    public float getGroundHeight() {
        return groundHeight;
    }

    public void setGroundHeight(float groundHeight) {
        this.groundHeight = groundHeight;
    }
}
