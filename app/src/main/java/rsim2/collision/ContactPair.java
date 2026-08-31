package rsim2.collision;

import org.joml.Vector3f;
import rsim2.scene.SceneNode;

public class ContactPair {

    public enum CollisionType {
        SELF_COLLISION,
        GROUND_COLLISION,
        OBSTACLE_COLLISION
    }

    private final SceneNode nodeA;
    private final SceneNode nodeB;
    private final CollisionType type;
    private final Vector3f contactPoint;
    private final float penetrationDepth;
    private final String description;

    public ContactPair(SceneNode nodeA, SceneNode nodeB, CollisionType type, Vector3f contactPoint, float penetrationDepth) {
        this.nodeA = nodeA;
        this.nodeB = nodeB;
        this.type = type;
        this.contactPoint = contactPoint != null ? new Vector3f(contactPoint) : new Vector3f();
        this.penetrationDepth = penetrationDepth;

        String nameA = nodeA != null ? nodeA.getId() : "Unknown";
        if (type == CollisionType.GROUND_COLLISION) {
            this.description = nameA + " <-> Ground";
        } else {
            String nameB = nodeB != null ? nodeB.getId() : "Unknown";
            this.description = nameA + " <-> " + nameB;
        }
    }

    public static ContactPair createSelfCollision(SceneNode nodeA, SceneNode nodeB, Vector3f contactPoint) {
        return new ContactPair(nodeA, nodeB, CollisionType.SELF_COLLISION, contactPoint, 0.0f);
    }

    public static ContactPair createGroundCollision(SceneNode nodeA, Vector3f contactPoint, float penetrationDepth) {
        return new ContactPair(nodeA, null, CollisionType.GROUND_COLLISION, contactPoint, penetrationDepth);
    }

    public SceneNode getNodeA() {
        return nodeA;
    }

    public SceneNode getNodeB() {
        return nodeB;
    }

    public CollisionType getType() {
        return type;
    }

    public Vector3f getContactPoint() {
        return contactPoint;
    }

    public float getPenetrationDepth() {
        return penetrationDepth;
    }

    public String getDescription() {
        return description;
    }

    public boolean involves(SceneNode node) {
        return node != null && (node == nodeA || node == nodeB);
    }

    public String getPairKey() {
        if (nodeA == null) return "ground";
        if (nodeB == null) return nodeA.getId() + "_ground";
        String idA = nodeA.getId();
        String idB = nodeB.getId();
        return idA.compareTo(idB) < 0 ? idA + "::" + idB : idB + "::" + idA;
    }

    @Override
    public String toString() {
        return description;
    }
}
