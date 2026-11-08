package rsim2.scene;

import org.joml.Vector3f;
import org.joml.Matrix4f;
import rsim2.collision.AABB;
import rsim2.collision.EnvironmentMotionGuard;
import java.util.function.BooleanSupplier;
import rsim2.motion.MotionPlayer;
import rsim2.motion.MotionSequence;

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
    private MotionPlayer approachPlayer;
    private MotionSequence approachSequence;
    private long approachStopGeneration;
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
        Vector3f offset = captureOffset(tool, toolOffset);
        if (!offset.isFinite()) throw new IllegalArgumentException("Tool offset must be finite.");
        sceneJoints = List.copyOf(joints);
        float[] start = angles(sceneJoints);
        float[] solution = null;
        Vector3f target = null;
        String[] rejection = {null};
        search: for (Vector3f contactOffset : captureOffsets(tool, toolOffset)) {
        for (Vector3f candidate : graspTargets(object, tool, contactOffset)) {
            solution = solve(chain, tool, contactOffset, candidate, () -> {
                String reason = motionGuard.collisionAtPose(root, sceneJoints);
                if (reason != null) reason = "Target pose: " + reason;
                else if (!canCapture(object, tool, contactOffset)) reason = "Tool geometry remains outside capture clearance (TCP " + objectDistance(toolPoint(tool, contactOffset), object) + ", geometry " + geometryDistance(tool, object) + ", tolerance " + captureTolerance(object) + ").";
                else {
                    reason = motionGuard.validateStep(root, sceneJoints, start, angles(sceneJoints));
                    if (reason != null) reason = "Approach path: " + reason;
                }
                if (reason != null) { rejection[0] = reason; if (Boolean.getBoolean("rsim2.grasp.debug")) System.err.println(candidate + " " + contactOffset + " " + reason); }
                return reason == null;
            }, false, captureTolerance(object) * .25f);
            if (solution != null) { target = candidate; offset = contactOffset; break search; }
        }}
        if (solution == null) throw new IllegalArgumentException(rejection[0] == null
                ? "No IK solution reached the sampled surface points within joint limits. Check object position/scale and the selected tool."
                : "Surface approach blocked: " + rejection[0]);
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

    /** A best verified direct approach: partial progress is valid and never attaches the object. */
    public String approach(SceneNode root, List<Joint> joints, String objectId, String toolId, Vector3f toolOffset) {
        SceneNode object = findNode(root, objectId);
        SceneNode tool = findNode(root, toolId);
        if (object == null || !objects(root).contains(object)) throw new IllegalArgumentException("Environment object not found: " + objectId);
        if (tool == null || tool == root || !tool.isRobotPart() || isAncestor(object, tool)) throw new IllegalArgumentException("Invalid robot tool: " + toolId);
        if (toolOffset == null || !toolOffset.isFinite()) throw new IllegalArgumentException("Tool offset must be finite.");
        List<Joint> chain = new ArrayList<>();
        if (joints != null) for (SceneNode node = tool; node != null; node = node.getParent()) {
            for (Joint joint : joints) if (joint.getChildNode() == node && joint.getType() == JointType.REVOLUTE) chain.add(joint);
        }
        if (chain.isEmpty()) throw new IllegalArgumentException("Tool has no movable ancestor joints.");
        AABB bounds = objectBounds(object);
        Vector3f startPoint = toolPoint(tool, toolOffset);
        Vector3f nearest = new Vector3f(startPoint).max(bounds.getMin()).min(bounds.getMax());
        float initialDistance = startPoint.distance(nearest);
        if (initialDistance <= SURFACE_CLEARANCE) return "Tool origin is already within the approach clearance; no motion or attachment requested.";
        Vector3f destination = new Vector3f(nearest).add(new Vector3f(startPoint).sub(nearest).normalize().mul(SURFACE_CLEARANCE));
        List<Joint> all = List.copyOf(joints);
        float[] start = angles(all);
        String[] obstruction = {null};
        BooleanSupplier valid = () -> {
            String reason = motionGuard.validateStep(root, all, start, angles(all));
            if (reason != null) obstruction[0] = reason;
            return reason == null;
        };
        float[] best = null;
        float bestFraction = 0;
        float bestDistance = initialDistance;
        // If the surface is unreachable or blocked, try shorter advances instead of rejecting everything.
        for (int step = 16; step >= 1; step--) {
            float fraction = step / 16f;
            Vector3f target = new Vector3f(startPoint).lerp(destination, fraction);
            float[] solution = solve(chain, tool, toolOffset, target, valid, true);
            if (solution == null) continue;
            for (int i = 0; i < chain.size(); i++) chain.get(i).setAngle(solution[i]);
            Vector3f reached = toolPoint(tool, toolOffset);
            float distance = reached.distance(new Vector3f(reached).max(bounds.getMin()).min(bounds.getMax()));
            for (int i = 0; i < all.size(); i++) all.get(i).setAngle(start[i]);
            if (distance < initialDistance - 0.001f) {
                best = solution; bestFraction = fraction; bestDistance = distance; break;
            }
        }
        if (best == null) throw new IllegalArgumentException("No closer direct approach found. "
                + (obstruction[0] == null ? "Joint limits or IK prevented progress." : "Blocked by: " + obstruction[0]));
        float upper = Math.min(1, bestFraction + 1 / 16f);
        for (int refinement = 0; refinement < 5 && upper - bestFraction > 0.001f; refinement++) {
            float fraction = (bestFraction + upper) * 0.5f;
            float[] solution = solve(chain, tool, toolOffset, new Vector3f(startPoint).lerp(destination, fraction), valid, true);
            if (solution == null) { upper = fraction; continue; }
            for (int i = 0; i < chain.size(); i++) chain.get(i).setAngle(solution[i]);
            Vector3f reached = toolPoint(tool, toolOffset);
            float distance = reached.distance(new Vector3f(reached).max(bounds.getMin()).min(bounds.getMax()));
            for (int i = 0; i < all.size(); i++) all.get(i).setAngle(start[i]);
            if (distance < bestDistance) { best = solution; bestFraction = fraction; bestDistance = distance; }
            else upper = fraction;
        }
        cancelPending();
        for (int i = 0; i < chain.size(); i++) {
            Joint joint = chain.get(i);
            if (joint.getMotor() != null) joint.getMotor().setTargetAngleRadians(best[i]);
            else joint.setAngle(best[i]);
        }
        return String.format(Locale.ROOT,
                "[Approach only] %s -> %s: TCP-to-object-bounds distance %.4f -> %.4f scene units; %.0f%% distance reduction. No grasp or attachment.%s",
                toolId, objectId, initialDistance, bestDistance, (1 - bestDistance / initialDistance) * 100,
                bestDistance > SURFACE_CLEARANCE + 0.006f ? " Remaining distance limited by collisions, reach or sampled solutions; this is not a global shortest-distance guarantee."
                        + (obstruction[0] == null ? "" : " Last rejected candidate: " + obstruction[0]) : "");
    }

    private static float[] angles(List<Joint> joints) {
        float[] result = new float[joints.size()];
        for (int i = 0; i < result.length; i++) result[i] = joints.get(i).getCurrentAngleRadians();
        return result;
    }

    /** Default to the distal end of a mesh tool, rather than its hinge/pivot. Explicit TCPs win. */
    public static Vector3f captureOffset(SceneNode tool, Vector3f requested) {
        if (requested == null || !requested.isFinite()) throw new IllegalArgumentException("Tool offset must be finite.");
        if (requested.lengthSquared() != 0 || tool.getMeshes().isEmpty()) return new Vector3f(requested);
        Vector3f farthest = new Vector3f();
        for (var mesh : tool.getMeshes()) {
            float[] vertices = mesh.getVertices();
            for (int i = 0; vertices != null && i + 2 < vertices.length; i += 3) {
                Vector3f point = new Vector3f(vertices[i], vertices[i + 1], vertices[i + 2]);
                if (point.lengthSquared() > farthest.lengthSquared()) farthest.set(point);
            }
        }
        if (farthest.lengthSquared() == 0) return farthest;
        // Keep an actual surface point. Averaging distal vertices can place a TCP
        // inside the jaw, making a small visual-contact clearance unreachable.
        return farthest;
    }

    private static List<Vector3f> captureOffsets(SceneNode tool, Vector3f requested) {
        Vector3f tip = captureOffset(tool, requested);
        List<Vector3f> result = new ArrayList<>(); result.add(tip);
        if (requested.lengthSquared() != 0 || tool.getMeshes().isEmpty()) return result;
        // A distal corner need not lead the jaw for every object face/orientation.
        // Try real distal surface vertices extremal along each tool-local axis.
        for (int axis = 0; axis < 3; axis++) for (int sign : new int[]{-1,1}) {
            Vector3f support = new Vector3f(tip);
            for (var mesh : tool.getMeshes()) {
                float[] vertices = mesh.getVertices();
                for (int i = 0; vertices != null && i + 2 < vertices.length; i += 3) {
                    Vector3f v = new Vector3f(vertices[i], vertices[i+1], vertices[i+2]);
                    if (v.lengthSquared() >= tip.lengthSquared() * .5f && sign*v.get(axis) > sign*support.get(axis)) support.set(v);
                }
            }
            if (result.stream().noneMatch(v -> v.distanceSquared(support) < 1e-10f)) result.add(support);
        }
        return result;
    }

    private static boolean canCapture(SceneNode object, SceneNode tool, Vector3f offset) {
        AABB bounds = objectBounds(object);
        if (bounds.getHalfExtents().lengthSquared() == 0)
            return toolPoint(tool, offset).distance(bounds.getCenter()) <= CAPTURE_DISTANCE;
        // Clearance targets are planning aids, not evidence of contact. Require the
        // calibrated/derived TCP AND actual tool geometry near the object at capture.
        boolean pointTool = tool.getMeshes().isEmpty() && tool.getCollisionShape() == null;
        float tolerance = captureTolerance(object);
        if (pointTool) return objectDistance(toolPoint(tool, offset), object) <= tolerance;
        return (tool.getMeshes().isEmpty() || objectDistance(toolPoint(tool, offset), object) <= tolerance)
                && geometryDistance(tool, object) <= tolerance;
    }

    private static float captureTolerance(SceneNode object) {
        Vector3f size = objectBounds(object).getExtents();
        float smallest = Float.POSITIVE_INFINITY;
        for (float extent : new float[]{size.x, size.y, size.z}) if (extent > 0) smallest = Math.min(smallest, extent);
        return Float.isFinite(smallest) ? Math.min(.01f, smallest * .05f) : CAPTURE_DISTANCE;
    }

    private static float pointDistance(Vector3f point, AABB bounds) {
        return point.distance(new Vector3f(point).max(bounds.getMin()).min(bounds.getMax()));
    }

    private static float objectDistance(Vector3f point, SceneNode object) {
        float distance = Float.POSITIVE_INFINITY;
        Matrix4f world = object.getWorldTransform();
        if (object.getCollisionShape() != null) distance = shapeDistance(point, object.getCollisionShape(), world);
        else for (var mesh : object.getMeshes()) distance = Math.min(distance, meshDistance(point, mesh, world));
        for (SceneNode child : object.getChildren()) distance = Math.min(distance, objectDistance(point, child));
        return distance;
    }

    private static float shapeDistance(Vector3f point, rsim2.collision.CollisionShape shape, Matrix4f world) {
        if (shape instanceof rsim2.collision.CompoundShape compound) {
            float distance = Float.POSITIVE_INFINITY;
            for (var entry : compound.getChildren()) distance = Math.min(distance,
                    shapeDistance(point, entry.getShape(), new Matrix4f(world).mul(entry.getLocalTransform())));
            return distance;
        }
        AABB local = shape.getLocalAABB();
        Vector3f nearest = new Matrix4f(world).invert().transformPosition(new Vector3f(point)).max(local.getMin()).min(local.getMax());
        return world.transformPosition(nearest).distance(point);
    }

    private static float meshDistance(Vector3f point, rsim2.graphics.Mesh mesh, Matrix4f world) {
        float[] vertices = mesh.getVertices(); int[] indices = mesh.getIndices();
        float distance = Float.POSITIVE_INFINITY;
        for (int i = 0; indices != null && i + 2 < indices.length; i += 3) {
            Vector3f[] triangle = triangle(vertices, indices, i, world);
            Vector3f nearest = new Vector3f();
            org.joml.Intersectionf.findClosestPointOnTriangle(triangle[0], triangle[1], triangle[2], point, nearest);
            if (nearest.isFinite()) distance = Math.min(distance, nearest.distance(point));
        }
        if (Float.isFinite(distance)) return distance;
        for (int i = 0; vertices != null && i + 2 < vertices.length; i += 3)
            distance = Math.min(distance, world.transformPosition(new Vector3f(vertices[i], vertices[i+1], vertices[i+2])).distance(point));
        return distance;
    }

    private static Vector3f[] triangle(float[] vertices, int[] indices, int i, Matrix4f world) {
        Vector3f[] triangle = new Vector3f[3];
        for (int j = 0; j < 3; j++) {
            int v = indices[i+j] * 3;
            triangle[j] = world.transformPosition(new Vector3f(vertices[v], vertices[v+1], vertices[v+2]));
        }
        return triangle;
    }

    private static float geometryDistance(SceneNode tool, SceneNode object) {
        float distance = Float.POSITIVE_INFINITY;
        Matrix4f world = tool.getWorldTransform();
        for (var mesh : tool.getMeshes()) {
            float[] vertices = mesh.getVertices();
            int[] indices = mesh.getIndices();
            for (int i = 0; vertices != null && i + 2 < vertices.length; i += 3)
                distance = Math.min(distance, objectDistance(world.transformPosition(new Vector3f(vertices[i], vertices[i+1], vertices[i+2])), object));
            for (int i = 0; indices != null && i + 2 < indices.length; i += 3) {
                Vector3f[] triangle = triangle(vertices, indices, i, world);
                // A point on a real triangle, rather than the empty space in a
                // whole-tool bounding box (which can include the gripper opening).
                Vector3f nearest = new Vector3f();
                org.joml.Intersectionf.findClosestPointOnTriangle(triangle[0], triangle[1], triangle[2], objectCenter(object), nearest);
                if (nearest.isFinite()) distance = Math.min(distance, objectDistance(nearest, object));
            }
        }
        if (Float.isFinite(distance)) return distance;
        if (tool.getCollisionShape() != null) {
            return collisionGeometryDistance(tool.getCollisionShape(), world, object);
        }
        // Geometry-free tool markers represent an explicitly configured point TCP.
        return objectDistance(toolPoint(tool, new Vector3f()), object);
    }

    private static float collisionGeometryDistance(rsim2.collision.CollisionShape shape, Matrix4f world, SceneNode object) {
        if (shape instanceof rsim2.collision.CompoundShape compound) {
            float distance = Float.POSITIVE_INFINITY;
            for (var entry : compound.getChildren()) distance = Math.min(distance,
                    collisionGeometryDistance(entry.getShape(), new Matrix4f(world).mul(entry.getLocalTransform()), object));
            return distance;
        }
        Vector3f near = new Vector3f();
        if (shape instanceof rsim2.collision.BoxShape) {
            AABB local = shape.getLocalAABB();
            new Matrix4f(world).invert().transformPosition(objectCenter(object), near).max(local.getMin()).min(local.getMax());
            world.transformPosition(near);
        } else {
            Vector3f direction = objectCenter(object).sub(world.transformPosition(shape.getLocalAABB().getCenter()));
            shape.getSupportPoint(direction, world, near);
        }
        return objectDistance(near, object);
    }

    public boolean isHeld(SceneNode object) { return originalParents.containsKey(object); }

    /** Plans a world-up translation using the held object's center as the IK point. */
    public String lift(SceneNode root, List<Joint> joints, String objectId, float height) {
        SceneNode object = findNode(root, objectId);
        if (!isHeld(object)) throw new IllegalArgumentException("Object is not held: " + objectId);
        if (!Float.isFinite(height) || height <= 0) throw new IllegalArgumentException("Lift distance must be finite and positive.");
        SceneNode tool = object.getParent();
        List<Joint> chain = new ArrayList<>();
        for (SceneNode n = tool; n != null; n = n.getParent())
            for (Joint joint : joints) if (joint.getChildNode() == n && joint.getType() == JointType.REVOLUTE) chain.add(joint);
        Vector3f center = objectCenter(object);
        Vector3f offset = new Matrix4f(tool.getWorldTransform()).invert().transformPosition(new Vector3f(center));
        Vector3f destination = new Vector3f(center).add(0, height, 0);
        float[] start = angles(joints);
        float[] solution = solve(chain, tool, offset, destination,
                () -> motionGuard.collisionAtPose(root, joints) == null
                        && motionGuard.validateStep(root, joints, start, angles(joints)) == null, false, Math.min(.005f, height * .1f));
        if (solution == null) throw new IllegalArgumentException("No collision-free upward lift is reachable within joint limits; object remains held.");
        for (int i = 0; i < chain.size(); i++) {
            Joint joint = chain.get(i);
            if (joint.getMotor() != null) joint.getMotor().setTargetAngleRadians(solution[i]);
            else joint.setAngle(solution[i]);
        }
        return "Lifting " + objectId + " upward by " + height + " scene units.";
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
        // Try close surface points too: a sphere around a long tool excludes reachable
        // grasps unnecessarily. Actual geometry is still checked for every solution.
        for (float fraction : new float[]{0, 0.25f, 0.5f, 0.625f, 0.75f, 1})
        for (int axis = 0; axis < 3; axis++) for (int sign : new int[]{1, -1}) {
            Vector3f point = new Vector3f(center);
            point.setComponent(axis, center.get(axis) + sign * (half.get(axis) + radius * fraction + captureTolerance(object) * .5f));
            targets.add(point);
        }
        for (int axis = 0; axis < 3; axis++) for (int sign : new int[]{1,-1}) {
            Vector3f point = new Vector3f(center);
            point.setComponent(axis, center.get(axis) + sign * (half.get(axis) + captureTolerance(object) * .9f));
            targets.add(point);
        }
        // Face centers can place the wrist inside a wide object. Sample near
        // face edges too, where the same real fingertip can approach freely.
        for (int axis = 0; axis < 3; axis++) for (int sign : new int[]{1,-1})
        for (int tangent = 0; tangent < 3; tangent++) if (tangent != axis)
        for (int side : new int[]{1,-1}) {
            Vector3f point = new Vector3f(center);
            point.setComponent(axis, center.get(axis) + sign * (half.get(axis) + captureTolerance(object) * .5f));
            point.setComponent(tangent, center.get(tangent) + side * half.get(tangent) * .8f);
            targets.add(point);
        }
        targets.sort(Comparator.comparingDouble(p -> p.distanceSquared(tcp)));
        return targets;
    }

    // Coordinate descent evaluates real FK, including imported origin rotations and joint limits.
    // Temporary poses are always restored; only a successful solution becomes motor targets.
    private float[] solve(List<Joint> chain, SceneNode tool, Vector3f offset, Vector3f target, BooleanSupplier valid) {
        return solve(chain, tool, offset, target, valid, false);
    }

    private float[] solve(List<Joint> chain, SceneNode tool, Vector3f offset, Vector3f target, BooleanSupplier valid, boolean bestEffort) {
        return solve(chain, tool, offset, target, valid, bestEffort, .005f);
    }

    private float[] solve(List<Joint> chain, SceneNode tool, Vector3f offset, Vector3f target, BooleanSupplier valid, boolean bestEffort, float tolerance) {
        float[] original = new float[chain.size()];
        for (int i = 0; i < chain.size(); i++) original[i] = chain.get(i).getCurrentAngleRadians();
        Random random = new Random(0);
        float[] closest = null;
        float closestDistance = toolPoint(tool, offset).distanceSquared(target);
        try {
            for (int attempt = 0; attempt < 6; attempt++) {
                for (int i = 0; i < chain.size(); i++) {
                    Joint j = chain.get(i);
                    j.setAngle(attempt == 0 ? original[i] : j.getMinLimit() + random.nextFloat() * (j.getMaxLimit() - j.getMinLimit()));
                }
                float step = 0.5f;
                for (int pass = 0; pass < 300 && step >= 0.0005f; pass++) {
                    float before = toolPoint(tool, offset).distanceSquared(target);
                    if (before <= tolerance * tolerance) {
                        if (!valid.getAsBoolean()) break;
                        float[] result = new float[chain.size()];
                        for (int i = 0; i < chain.size(); i++) result[i] = chain.get(i).getCurrentAngleRadians();
                        return result;
                    }
                    improvePosition(chain, tool, offset, target);
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
                float distance = toolPoint(tool, offset).distanceSquared(target);
                if (bestEffort && distance < closestDistance - 1e-8f && valid.getAsBoolean()) {
                    closestDistance = distance;
                    closest = angles(chain);
                }
            }
            return closest;
        } finally {
            for (int i = 0; i < chain.size(); i++) chain.get(i).setAngle(original[i]);
        }
    }

    private static Vector3f toolPoint(SceneNode tool, Vector3f offset) {
        return tool.getWorldTransform().transformPosition(new Vector3f(offset));
    }

    // Damped Jacobian IK handles coupled joints that coordinate descent alone stalls on.
    private static void improvePosition(List<Joint> chain, SceneNode tool, Vector3f offset, Vector3f target) {
        Vector3f point = toolPoint(tool, offset);
        Vector3f error = new Vector3f(target).sub(point);
        Vector3f[] columns = new Vector3f[chain.size()];
        float[] original = angles(chain);
        float xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
        for (int i = 0; i < chain.size(); i++) {
            Joint joint = chain.get(i);
            joint.setAngle(original[i] + 0.001f);
            columns[i] = toolPoint(tool, offset).sub(point).div(0.001f);
            joint.setAngle(original[i]);
            Vector3f v = columns[i];
            xx += v.x*v.x; xy += v.x*v.y; xz += v.x*v.z;
            yy += v.y*v.y; yz += v.y*v.z; zz += v.z*v.z;
        }
        float damping = Math.max(1e-8f, (xx + yy + zz) * 0.0004f);
        org.joml.Matrix3f inverse = new org.joml.Matrix3f(xx+damping, xy, xz,
                xy, yy+damping, yz, xz, yz, zz+damping).invert();
        Vector3f projected = inverse.transform(error, new Vector3f());
        if (!projected.isFinite()) return;
        float[] delta = new float[chain.size()];
        float maximum = 0.25f;
        for (int i = 0; i < delta.length; i++) { delta[i] = columns[i].dot(projected); maximum = Math.max(maximum, Math.abs(delta[i])); }
        for (float fraction : new float[]{1, 0.5f, 0.25f, 0.125f}) {
            for (int i = 0; i < delta.length; i++) {
                Joint joint = chain.get(i);
                joint.setAngle(Math.max(joint.getMinLimit(), Math.min(joint.getMaxLimit(),
                        original[i] + delta[i] * (0.25f / maximum) * fraction)));
            }
            if (toolPoint(tool, offset).distanceSquared(target) < error.lengthSquared()) return;
        }
        for (int i = 0; i < original.length; i++) chain.get(i).setAngle(original[i]);
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
        if (approachPlayer != null) {
            if (approachPlayer.getSequence() != approachSequence || approachPlayer.getStopGeneration() != approachStopGeneration) {
                cancelPending();
                return "Grasp cancelled: another motion replaced the approach.";
            }
            if (!approachPlayer.isPlaying() && approachPlayer.getCurrentTime() < approachSequence.getDurationSeconds()) return null;
        }
        remainingSeconds -= Math.max(0, dt);
        if (!objectPoseAtPlan.equals(pendingObject.getWorldTransform(), 0.0001f)) {
            cancelPending();
            return "Grasp cancelled: object moved; request a new approach.";
        }
        float tolerance = objectBounds(pendingObject).getHalfExtents().lengthSquared() == 0 ? CAPTURE_DISTANCE : captureTolerance(pendingObject) * .5f;
        if (toolPoint(pendingTool, pendingOffset).distance(pendingTarget) <= tolerance
                && canCapture(pendingObject, pendingTool, pendingOffset)) {
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
        if (approachPlayer != null && approachPlayer.getSequence() == approachSequence) approachPlayer.pause();
        for (Joint j : pendingChain) {
            if (j.getMotor() != null) {
                j.getMotor().setTargetAngleRadians(j.getCurrentAngleRadians());
                j.getMotor().setCurrentVelocity(0);
            }
        }
        clearPending();
    }

    public void bindPlayback(MotionPlayer player, MotionSequence sequence) {
        approachPlayer = player;
        approachSequence = sequence;
        approachStopGeneration = player.getStopGeneration();
    }

    private void clearPending() {
        pendingObject = null;
        pendingTool = null;
        pendingOffset = null;
        pendingChain = List.of();
        pendingTarget = null;
        objectPoseAtPlan = null;
        approachPlayer = null;
        approachSequence = null;
    }

    public String state(SceneNode object) {
        if (object == pendingObject) return "approaching with " + pendingTool.getId();
        return originalParents.containsKey(object) ? "held by " + object.getParent().getId() : "free";
    }
}
