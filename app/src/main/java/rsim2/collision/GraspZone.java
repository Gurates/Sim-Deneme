package rsim2.collision;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import rsim2.scene.Joint;
import rsim2.scene.JointType;
import rsim2.scene.SceneNode;

import java.util.*;

/**
 * The space between a gripper's jaws, found automatically: the volume the moving jaw sweeps between its joint
 * limits. A scene object inside this volume does not collide with the gripper, so the jaws can close around it.
 *
 * A gripper is a robot link (the palm, which may carry a fixed jaw) that is moved by another joint and carries one
 * or more terminal revolute joints (the moving jaws).
 */
public final class GraspZone {
    private static final int SWEEP_SAMPLES = 24;
    /** Each swept jaw box is enlarged by this fraction of its size, so objects count as inside slightly early. */
    private static final float MARGIN_FRACTION = 0.1f;

    /** One jaw position: the jaw link's local bounds and its pose in the tool frame. */
    private record Sample(AABB local, Matrix4f toTool, float angle, Joint jaw) {}

    private final SceneNode tool;
    private final List<Joint> jaws;
    private final List<Sample> samples = new ArrayList<>();
    private final Map<Joint, Float> closedAngles = new IdentityHashMap<>();

    private GraspZone(SceneNode tool, List<Joint> jaws) {
        this.tool = tool;
        this.jaws = List.copyOf(jaws);
    }

    public SceneNode getTool() { return tool; }

    public List<Joint> getJaws() { return jaws; }

    /** Estimated jaw angle at which the gripper is closed (the limit nearest to the palm), in radians. */
    public float closedAngle(Joint jaw) {
        Float angle = closedAngles.get(jaw);
        return angle == null ? jaw.getMinLimit() : angle;
    }

    public float openAngle(Joint jaw) {
        return closedAngle(jaw) == jaw.getMinLimit() ? jaw.getMaxLimit() : jaw.getMinLimit();
    }

    public static List<GraspZone> find(SceneNode root, List<Joint> joints) {
        if (root == null || joints == null) return List.of();
        Map<SceneNode, List<Joint>> jawsByTool = new LinkedHashMap<>();
        for (Joint joint : joints) {
            if (isJaw(joint, joints)) jawsByTool.computeIfAbsent(joint.getChildNode().getParent(), k -> new ArrayList<>()).add(joint);
        }
        List<GraspZone> zones = new ArrayList<>();
        jawsByTool.forEach((tool, jaws) -> {
            GraspZone zone = new GraspZone(tool, jaws);
            zone.sweep();
            if (!zone.samples.isEmpty()) zones.add(zone);
        });
        return zones;
    }

    private static boolean isJaw(Joint joint, List<Joint> joints) {
        if (joint == null || joint.getType() == JointType.FIXED) return false;
        SceneNode jaw = joint.getChildNode();
        if (jaw == null || !jaw.isRobotPart()) return false;
        SceneNode tool = jaw.getParent();
        if (tool == null || tool.getParent() == null || !tool.isRobotPart() || !hasGeometry(tool)) return false;
        if (joint.getMaxLimit() - joint.getMinLimit() < 1e-3f) return false;
        boolean toolMovedByArm = false;
        for (Joint other : joints) {
            if (other == joint || other.getType() == JointType.FIXED || other.getChildNode() == null) continue;
            if (isAncestor(jaw, other.getChildNode())) return false; // not terminal: a joint below the jaw
            if (isAncestor(other.getChildNode(), tool)) toolMovedByArm = true;
        }
        return toolMovedByArm;
    }

    private void sweep() {
        Matrix4f toolInverse = tool.getWorldTransform().invert();
        Vector3f toolCenter = localBounds(tool) == null ? new Vector3f() : localBounds(tool).getCenter();
        for (Joint jaw : jaws) {
            float saved = jaw.getCurrentAngleRadians();
            float nearestLimitDistance = Float.POSITIVE_INFINITY;
            try {
                for (int s = 0; s <= SWEEP_SAMPLES; s++) {
                    float angle = jaw.getMinLimit() + (jaw.getMaxLimit() - jaw.getMinLimit()) * s / SWEEP_SAMPLES;
                    jaw.setAngle(angle);
                    Vector3f jawCenter = new Vector3f();
                    int pieces = 0;
                    for (SceneNode node : subtree(jaw.getChildNode())) {
                        AABB local = node.isRobotPart() ? localBounds(node) : null;
                        if (local == null) continue;
                        Matrix4f toTool = new Matrix4f(toolInverse).mul(node.getWorldTransform());
                        samples.add(new Sample(enlarge(local), toTool, angle, jaw));
                        jawCenter.add(toTool.transformPosition(local.getCenter()));
                        pieces++;
                    }
                    if (pieces > 0 && (s == 0 || s == SWEEP_SAMPLES)) {
                        float distance = jawCenter.div(pieces).distance(toolCenter);
                        if (distance < nearestLimitDistance) {
                            nearestLimitDistance = distance;
                            closedAngles.put(jaw, angle);
                        }
                    }
                }
            } finally {
                jaw.setAngle(saved);
            }
        }
    }

    /** World boxes covering the zone at the tool's current pose. */
    public List<OBB> worldBoxes() {
        Matrix4f toolWorld = tool.getWorldTransform();
        List<OBB> boxes = new ArrayList<>(samples.size());
        for (Sample sample : samples) {
            OBB box = new OBB();
            box.fromAABBAndTransform(sample.local, new Matrix4f(toolWorld).mul(sample.toTool));
            boxes.add(box);
        }
        return boxes;
    }

    /** True when any part of {@code object} (including its children) is inside the zone. */
    public boolean contains(SceneNode object) {
        return contains(worldBoxes(), object);
    }

    static boolean contains(List<OBB> zoneBoxes, SceneNode object) {
        for (SceneNode node : subtree(object)) {
            AABB local = localBounds(node);
            if (local == null) continue;
            OBB box = new OBB();
            box.fromAABBAndTransform(local, node.getWorldTransform());
            for (OBB zone : zoneBoxes) if (zone.intersects(box)) return true;
        }
        return false;
    }

    /** True when a moving jaw's geometry, at its current angle, overlaps {@code object}. */
    public boolean jawTouches(Joint jaw, SceneNode object) {
        List<OBB> jawBoxes = new ArrayList<>();
        for (SceneNode node : subtree(jaw.getChildNode())) {
            AABB local = node.isRobotPart() ? localBounds(node) : null;
            if (local == null) continue;
            OBB box = new OBB();
            box.fromAABBAndTransform(local, node.getWorldTransform());
            jawBoxes.add(box);
        }
        return contains(jawBoxes, object);
    }

    /** True when {@code node} is the gripper or one of its jaws (any robot link below the tool). */
    public boolean isGripperPart(SceneNode node) {
        return node != null && node.isRobotPart() && isAncestor(tool, node);
    }

    /**
     * True when the pair is a gripper part and a scene object that is inside that gripper's zone or held by it.
     * Such a pair touches by design and is not a collision.
     */
    public static boolean gripIgnores(List<GraspZone> zones, SceneNode a, SceneNode b) {
        SceneNode objectA = a.getEnvironmentObject(), objectB = b.getEnvironmentObject();
        if ((objectA == null) == (objectB == null)) return false;
        SceneNode robot = objectA == null ? a : b;
        SceneNode object = objectA == null ? objectB : objectA;
        for (GraspZone zone : zones) {
            if (!zone.isGripperPart(robot)) continue;
            for (SceneNode n = object.getParent(); n != null; n = n.getParent()) {
                if (n.isRobotPart()) { if (zone.isGripperPart(n)) return true; break; }
            }
            if (zone.contains(object)) return true;
        }
        return false;
    }

    /** A grasp point in the tool frame: the middle of the swept volume on the closed half of the jaw range. */
    public Vector3f graspPointToolLocal() {
        AABB closedHalf = new AABB();
        closedHalf.reset();
        for (Sample sample : samples) {
            Joint jaw = sample.jaw;
            float closed = closedAngle(jaw), open = openAngle(jaw);
            if (Math.abs(sample.angle - closed) > Math.abs(open - closed) * 0.5f) continue;
            AABB inTool = new AABB();
            sample.local.transform(sample.toTool, inTool);
            closedHalf.union(inTool);
        }
        return closedHalf.isValid() ? closedHalf.getCenter() : new Vector3f();
    }

    private static AABB enlarge(AABB local) {
        Vector3f margin = local.getExtents().mul(MARGIN_FRACTION);
        return new AABB(new Vector3f(local.getMin()).sub(margin), new Vector3f(local.getMax()).add(margin));
    }

    static AABB localBounds(SceneNode node) {
        AABB local = node.getCollisionShape() != null ? node.getCollisionShape().getLocalAABB()
                : node.getMeshes().isEmpty() ? null : new AABB(node.getCombinedBoundingBoxMin(), node.getCombinedBoundingBoxMax());
        return local != null && local.isValid() ? local : null;
    }

    private static boolean hasGeometry(SceneNode node) {
        for (SceneNode n : subtree(node)) if (n.isRobotPart() && localBounds(n) != null) return true;
        return false;
    }

    private static List<SceneNode> subtree(SceneNode node) {
        List<SceneNode> result = new ArrayList<>();
        Deque<SceneNode> stack = new ArrayDeque<>();
        stack.push(node);
        while (!stack.isEmpty()) {
            SceneNode current = stack.pop();
            result.add(current);
            for (SceneNode child : current.getChildren()) stack.push(child);
        }
        return result;
    }

    private static boolean isAncestor(SceneNode ancestor, SceneNode node) {
        for (SceneNode n = node; n != null; n = n.getParent()) if (n == ancestor) return true;
        return false;
    }
}
