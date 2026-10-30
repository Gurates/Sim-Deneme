package rsim2.scene;

import org.joml.Vector3f;
import org.joml.Matrix4f;
import rsim2.collision.AABB;
import rsim2.collision.EnvironmentMotionGuard;
import java.util.function.BooleanSupplier;

import java.util.*;

/** Simulation-only, position-based grasping. No contact forces or gravity are simulated. */
public class ObjectInteractionController {
    public static final float CAPTURE_DISTANCE = 0.03f;
    private final Map<SceneNode, SceneNode> originalParents = new IdentityHashMap<>();
    private SceneNode pendingObject;
    private SceneNode pendingTool;
    private Vector3f pendingOffset;
    private List<Joint> pendingChain = List.of();
    private List<Joint> sceneJoints = List.of();
    private Vector3f pendingTarget;
    private Matrix4f objectPoseAtPlan;
    private final EnvironmentMotionGuard motionGuard = new EnvironmentMotionGuard();
    private static final float SURFACE_CLEARANCE = 0.01f;
    private float remainingSeconds;

    public static List<SceneNode> objects(SceneNode root) {
        List<SceneNode> result = new ArrayList<>();
        collectObjects(root, result, false);
        return result;
    }

    private static void collectObjects(SceneNode node, List<SceneNode> result, boolean environmentParent) {
        if (node == null) return;
        boolean environment = environmentParent || !node.isRobotPart();
        if (environment && !environmentParent) result.add(node);
        for (SceneNode child : node.getChildren()) collectObjects(child, result, environment);
    }

    public static Vector3f objectCenter(SceneNode node) {
        return objectBounds(node).getCenter();
    }

    public static AABB objectBounds(SceneNode node) {
        Vector3f min = new Vector3f(Float.POSITIVE_INFINITY);
        Vector3f max = new Vector3f(Float.NEGATIVE_INFINITY);
        collectBounds(node, min, max);
        if (!Float.isFinite(min.x)) {
            node.getWorldTransform().getTranslation(min);
            max.set(min);
        }
        return new AABB(min, max);
    }

    private static void collectBounds(SceneNode node, Vector3f min, Vector3f max) {
        if (!node.getMeshes().isEmpty() || node.getCollisionShape() != null) {
            Vector3f lo = node.getCollisionShape() != null ? node.getCollisionShape().getLocalAABB().getMin() : node.getCombinedBoundingBoxMin();
            Vector3f hi = node.getCollisionShape() != null ? node.getCollisionShape().getLocalAABB().getMax() : node.getCombinedBoundingBoxMax();
            for (int i = 0; i < 8; i++) {
                Vector3f p = new Vector3f((i & 1) == 0 ? lo.x : hi.x,
                        (i & 2) == 0 ? lo.y : hi.y, (i & 4) == 0 ? lo.z : hi.z);
                node.getWorldTransform().transformPosition(p);
                min.min(p);
                max.max(p);
            }
        }
        for (SceneNode child : node.getChildren()) collectBounds(child, min, max);
    }

    public static SceneNode findNode(SceneNode root, String id) {
        if (root == null) return null;
        List<SceneNode> matches = new ArrayList<>();
        findNodes(root, id, matches);
        if (matches.size() > 1) throw new IllegalArgumentException("Ambiguous node ID: " + id);
        return matches.isEmpty() ? null : matches.get(0);
    }

    private static void findNodes(SceneNode node, String id, List<SceneNode> matches) {
        if (node.getId().equals(id)) matches.add(node);
        for (SceneNode child : node.getChildren()) findNodes(child, id, matches);
    }

    public String grasp(SceneNode root, List<Joint> joints, String objectId, String toolId, Vector3f toolOffset) {
        if (pendingObject != null) throw new IllegalArgumentException("A grasp is already approaching its target.");
        SceneNode object = findNode(root, objectId);
        if (object == null || !objects(root).contains(object)) {
            throw new IllegalArgumentException("Environment object not found: " + objectId);
        }
        if (originalParents.containsKey(object)) throw new IllegalArgumentException("Object is already held.");
        SceneNode tool = findNode(root, toolId);
        if (tool == null || tool == root || !tool.isRobotPart() || isAncestor(object, tool)) {
            throw new IllegalArgumentException("Invalid robot tool: " + toolId);
        }
        if (originalParents.keySet().stream().anyMatch(n -> n.getParent() == tool)) {
            throw new IllegalArgumentException("This tool already holds an object.");
        }
        List<Joint> chain = new ArrayList<>();
        if (joints != null) for (SceneNode n = tool; n != null; n = n.getParent()) {
            for (Joint j : joints) if (j.getChildNode() == n && j.getType() == JointType.REVOLUTE) chain.add(j);
        }
        if (chain.isEmpty()) throw new IllegalArgumentException("Tool has no movable ancestor joints.");
        Vector3f offset = new Vector3f(toolOffset);
        if (!offset.isFinite()) throw new IllegalArgumentException("Tool offset must be finite.");
        sceneJoints = List.copyOf(joints);
        float[] start = angles(sceneJoints);
        float[] solution = null;
        Vector3f target = null;
        for (Vector3f candidate : graspTargets(object, tool, offset)) {
            solution = solve(chain, tool, offset, candidate,
                    () -> motionGuard.collisionAtPose(root, sceneJoints) == null
                            && motionGuard.validateStep(root, sceneJoints, start, angles(sceneJoints)) == null);
            if (solution != null) { target = candidate; break; }
        }
        if (solution == null) throw new IllegalArgumentException("No collision-free surface approach found within joint limits. Move the object, adjust the tool point, or clear the path.");
        remainingSeconds = 10;
        for (int i = 0; i < chain.size(); i++) {
            Joint j = chain.get(i);
            remainingSeconds = Math.max(remainingSeconds, 5 + Math.abs(solution[i] - j.getCurrentAngleRadians())
                    / Math.max(0.01f, j.getMaxSpeed() * j.getTargetSpeedRatio()));
            if (j.getMotor() != null) j.getMotor().setTargetAngleRadians(solution[i]);
            else j.setAngle(solution[i]);
        }
        pendingObject = object;
        pendingTool = tool;
        pendingOffset = offset;
        pendingChain = chain;
        pendingTarget = target;
        objectPoseAtPlan = object.getWorldTransform();
        return "Approaching the surface of " + objectId + " with " + toolId + "; attachment requires a clear approach without object penetration.";
    }

    private static float[] angles(List<Joint> joints) {
        float[] result = new float[joints.size()];
        for (int i = 0; i < result.length; i++) result[i] = joints.get(i).getCurrentAngleRadians();
        return result;
    }

    /** Conservative tool envelope: keeps all tool/finger descendants outside the target. */
    private static List<Vector3f> graspTargets(SceneNode object, SceneNode tool, Vector3f offset) {
        AABB bounds = objectBounds(object);
        Vector3f center = bounds.getCenter();
        Vector3f half = bounds.getHalfExtents();
        if (half.lengthSquared() == 0) return List.of(center);
        AABB toolBounds = objectBounds(tool);
        Vector3f tcp = toolPoint(tool, offset);
        float radius = 0;
        for (int i = 0; i < 8; i++) {
            Vector3f corner = new Vector3f((i & 1) == 0 ? toolBounds.getMin().x : toolBounds.getMax().x,
                    (i & 2) == 0 ? toolBounds.getMin().y : toolBounds.getMax().y,
                    (i & 4) == 0 ? toolBounds.getMin().z : toolBounds.getMax().z);
            radius = Math.max(radius, corner.distance(tcp));
        }
        List<Vector3f> targets = new ArrayList<>();
        for (int axis = 0; axis < 3; axis++) for (int sign : new int[]{1, -1}) {
            Vector3f point = new Vector3f(center);
            point.setComponent(axis, center.get(axis) + sign * (half.get(axis) + radius + SURFACE_CLEARANCE));
            targets.add(point);
        }
        targets.sort(Comparator.comparingDouble(p -> p.distanceSquared(tcp)));
        return targets;
    }

    // Coordinate descent evaluates real FK, including imported origin rotations and joint limits.
    // Temporary poses are always restored; only a successful solution becomes motor targets.
    private float[] solve(List<Joint> chain, SceneNode tool, Vector3f offset, Vector3f target, BooleanSupplier valid) {
        float[] original = new float[chain.size()];
        for (int i = 0; i < chain.size(); i++) original[i] = chain.get(i).getCurrentAngleRadians();
        Random random = new Random(0);
        try {
            for (int attempt = 0; attempt < 6; attempt++) {
                for (int i = 0; i < chain.size(); i++) {
                    Joint j = chain.get(i);
                    j.setAngle(attempt == 0 ? original[i] : j.getMinLimit() + random.nextFloat() * (j.getMaxLimit() - j.getMinLimit()));
                }
                float step = 0.5f;
                for (int pass = 0; pass < 300 && step >= 0.0005f; pass++) {
                    float before = toolPoint(tool, offset).distanceSquared(target);
                    if (before <= 0.000025f) {
                        if (!valid.getAsBoolean()) break;
                        float[] result = new float[chain.size()];
                        for (int i = 0; i < chain.size(); i++) result[i] = chain.get(i).getCurrentAngleRadians();
                        return result;
                    }
                    for (Joint j : chain) {
                        float angle = j.getCurrentAngleRadians();
                        float best = angle;
                        float distance = toolPoint(tool, offset).distanceSquared(target);
                        for (int direction : new int[]{-1, 1}) {
                            float candidate = Math.max(j.getMinLimit(), Math.min(j.getMaxLimit(), angle + direction * step));
                            j.setAngle(candidate);
                            float d = toolPoint(tool, offset).distanceSquared(target);
                            if (d < distance) { best = candidate; distance = d; }
                        }
                        j.setAngle(best);
                    }
                    if (before - toolPoint(tool, offset).distanceSquared(target) < 1e-8f) step *= 0.5f;
                }
            }
            return null;
        } finally {
            for (int i = 0; i < chain.size(); i++) chain.get(i).setAngle(original[i]);
        }
    }

    private static Vector3f toolPoint(SceneNode tool, Vector3f offset) {
        return tool.getWorldTransform().transformPosition(new Vector3f(offset));
    }

    private static boolean isAncestor(SceneNode ancestor, SceneNode node) {
        for (SceneNode n = node; n != null; n = n.getParent()) if (n == ancestor) return true;
        return false;
    }

    public String update(float dt, SceneNode root, boolean stopped) {
        originalParents.keySet().removeIf(n -> !isAncestor(root, n));
        if (pendingObject == null) return null;
        if (stopped || !isAncestor(root, pendingObject) || !isAncestor(root, pendingTool)) {
            cancelPending();
            return "Grasp cancelled: stopped or target/tool removed.";
        }
        remainingSeconds -= Math.max(0, dt);
        if (!objectPoseAtPlan.equals(pendingObject.getWorldTransform(), 0.0001f)) {
            cancelPending();
            return "Grasp cancelled: object moved; request a new approach.";
        }
        float tolerance = objectBounds(pendingObject).getHalfExtents().lengthSquared() == 0 ? CAPTURE_DISTANCE : 0.006f;
        if (toolPoint(pendingTool, pendingOffset).distance(pendingTarget) <= tolerance) {
            String collision = motionGuard.collisionAtPose(root, sceneJoints);
            if (collision != null) {
                cancelPending();
                return "Grasp cancelled before attachment: " + collision;
            }
            SceneNode object = pendingObject;
            SceneNode oldParent = object.getParent();
            if (!reparentChecked(object, pendingTool, () -> motionGuard.collisionAtPose(root, sceneJoints) == null)) {
                cancelPending();
                return "Grasp cancelled: attachment would distort geometry or create a collision. Use uniform robot scaling.";
            }
            originalParents.put(object, oldParent);
            cancelPending();
            return "Grasped " + object.getId() + ". Object now follows the tool (kinematic attachment).";
        }
        if (remainingSeconds <= 0) {
            cancelPending();
            return "Grasp failed: tool did not reach the object in time.";
        }
        return null;
    }

    public String release(SceneNode root, String objectId) {
        SceneNode object = findNode(root, objectId);
        if (object != null && object == pendingObject) {
            cancelPending();
            return "Pending grasp cancelled for " + objectId;
        }
        if (!originalParents.containsKey(object)) throw new IllegalArgumentException("Object is not held: " + objectId);
        SceneNode parent = originalParents.get(object);
        if (!reparentChecked(object, isAncestor(root, parent) ? parent : root, () -> true)) {
            throw new IllegalArgumentException("Release would distort geometry; use uniform parent scaling.");
        }
        originalParents.remove(object);
        return "Released " + objectId + " at its current world pose; gravity is not simulated.";
    }

    private static boolean reparentChecked(SceneNode object, SceneNode parent, BooleanSupplier valid) {
        SceneNode originalParent = object.getParent();
        int index = originalParent == null ? -1 : originalParent.indexOfChild(object);
        Matrix4f world = object.getWorldTransform();
        Vector3f position = new Vector3f(object.getLocalPosition());
        Vector3f scale = new Vector3f(object.getLocalScale());
        org.joml.Quaternionf rotation = new org.joml.Quaternionf(object.getLocalRotation());
        boolean accepted = false;
        try {
            object.reparentPreservingWorldTransform(parent);
            accepted = world.equals(object.getWorldTransform(), 0.0001f) && valid.getAsBoolean();
            return accepted;
        } finally {
            if (!accepted) {
                if (object.getParent() != null) object.getParent().removeChild(object);
                if (originalParent != null) originalParent.addChild(index, object);
                object.getLocalPosition().set(position);
                object.getLocalScale().set(scale);
                object.getLocalRotation().set(rotation);
            }
        }
    }

    public void cancelPending() {
        for (Joint j : pendingChain) {
            if (j.getMotor() != null) {
                j.getMotor().setTargetAngleRadians(j.getCurrentAngleRadians());
                j.getMotor().setCurrentVelocity(0);
            }
        }
        clearPending();
    }

    private void clearPending() {
        pendingObject = null;
        pendingTool = null;
        pendingOffset = null;
        pendingChain = List.of();
        pendingTarget = null;
        objectPoseAtPlan = null;
    }

    public String state(SceneNode object) {
        if (object == pendingObject) return "approaching with " + pendingTool.getId();
        return originalParents.containsKey(object) ? "held by " + object.getParent().getId() : "free";
    }
}
