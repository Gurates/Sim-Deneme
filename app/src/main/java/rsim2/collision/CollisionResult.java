package rsim2.collision;

import rsim2.scene.SceneNode;

import java.util.*;

public class CollisionResult {
    private final List<ContactPair> contacts = new ArrayList<>();
    private final Set<SceneNode> collidingNodes = new HashSet<>();
    private long computationTimeNanos = 0;

    public CollisionResult() {
    }

    public void clear() {
        contacts.clear();
        collidingNodes.clear();
        computationTimeNanos = 0;
    }

    public void addContact(ContactPair contact) {
        if (contact == null) return;
        contacts.add(contact);
        if (contact.getNodeA() != null) collidingNodes.add(contact.getNodeA());
        if (contact.getNodeB() != null) collidingNodes.add(contact.getNodeB());
    }

    public boolean hasCollision() {
        return !contacts.isEmpty();
    }

    public int getCollisionCount() {
        return contacts.size();
    }

    public List<ContactPair> getContacts() {
        return Collections.unmodifiableList(contacts);
    }

    public Set<SceneNode> getCollidingNodes() {
        return Collections.unmodifiableSet(collidingNodes);
    }

    public boolean isNodeColliding(SceneNode node) {
        return node != null && collidingNodes.contains(node);
    }

    public long getComputationTimeNanos() {
        return computationTimeNanos;
    }

    public void setComputationTimeNanos(long computationTimeNanos) {
        this.computationTimeNanos = computationTimeNanos;
    }

    public float getComputationTimeMs() {
        return computationTimeNanos / 1_000_000.0f;
    }
}
