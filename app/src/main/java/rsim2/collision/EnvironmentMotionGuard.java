package rsim2.collision;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import rsim2.scene.Joint;
import rsim2.scene.JointType;
import rsim2.scene.SceneNode;

import java.util.*;

/** Conservative geometry guard, independent of display/diagnostic collision filters.
 * Uses oriented bounds (world AABBs for sheared transforms), not finger contact forces.
 */
public final class EnvironmentMotionGuard {
    private static final float EPS = 0.00001f;
    private static final int MAX_SAMPLES = 10000;

    private record Body(SceneNode node, SceneNode environment, boolean movable, AABB local, CollisionShape shape, Matrix4f localTransform) {}
    private record Bounds(OBB box, Matrix4f transform) {}

    /** Returns a reason for rejecting this joint step, or null. Restores the input pose. */
    public String validateStep(SceneNode root, List<Joint> joints, float[] before, float[] after) {
        if (root == null || joints == null) return null;
        if (before.length != joints.size() || after.length != joints.size()) return "Invalid motion sample size.";
        float[] saved = new float[joints.size()];
        for (int i = 0; i < joints.size(); i++) {
            saved[i] = joints.get(i).getCurrentAngleRadians();
            if (!Float.isFinite(before[i]) || !Float.isFinite(after[i])) return "Non-finite joint motion.";
        }
        List<Body> bodies = bodies(root, joints);
        try {
            apply(joints, before, after, 0);
            List<Bounds> previous = bounds(bodies);
            double travel = 0;
            float angularTravel = 0;
            float smallest = Float.POSITIVE_INFINITY;
            for (Bounds bound : previous) {
                Vector3f h = bound.box.getHalfExtents();
                for (float extent : new float[]{h.x, h.y, h.z}) if (extent > EPS) smallest = Math.min(smallest, extent);
            }
            for (int j = 0; j < joints.size(); j++) {
                float delta = Math.abs(after[j] - before[j]);
                angularTravel = Math.max(angularTravel, delta);
                SceneNode pivotNode = joints.get(j).getChildNode();
                if (pivotNode == null || delta == 0) continue;
                Vector3f pivot = pivotNode.getWorldTransform().getTranslation(new Vector3f());
                float radius = 0;
                for (int b = 0; b < bodies.size(); b++) if (ancestor(pivotNode, bodies.get(b).node)) {
                    for (Vector3f corner : previous.get(b).box.getCorners()) radius = Math.max(radius, corner.distance(pivot));
                }
                travel += delta * radius;
            }
            // Resolution follows geometry scale. A hard 0.002 scene-unit cap made
            // scaled robots exceed MAX_SAMPLES on otherwise ordinary movements.
            double distanceStep = Math.max(0.0001, smallest * 0.5);
            double required = Math.max(Math.ceil(angularTravel / 0.01), Math.ceil(travel / distanceStep));
            if (!Double.isFinite(required) || required > MAX_SAMPLES) return "Motion step is too large to collision-check safely.";
            int samples = Math.max(1, (int) required);
            for (int sample = 1; sample <= samples; sample++) {
                apply(joints, before, after, (float) sample / samples);
                List<Bounds> current = bounds(bodies);
                String collision = collision(bodies, current, previous, true);
                if (collision != null) return collision;
                previous = current;
            }
            return null;
        } finally {
            for (int i = 0; i < joints.size(); i++) joints.get(i).setAngle(saved[i]);
        }
    }

    /** Checks the final pose without permitting initial overlaps. */
    public String collisionAtPose(SceneNode root, List<Joint> joints) {
        if (root == null) return null;
        List<Body> bodies = bodies(root, joints == null ? List.of() : joints);
        return collision(bodies, bounds(bodies), null, false);
    }

    private static void apply(List<Joint> joints, float[] from, float[] to, float t) {
        for (int i = 0; i < joints.size(); i++) joints.get(i).setAngle(from[i] + (to[i] - from[i]) * t);
    }

    private static List<Body> bodies(SceneNode root, List<Joint> joints) {
        Set<SceneNode> movingRoots = Collections.newSetFromMap(new IdentityHashMap<>());
        for (Joint joint : joints) if (joint.getType() != JointType.FIXED && joint.getChildNode() != null) movingRoots.add(joint.getChildNode());
        List<Body> result = new ArrayList<>();
        collect(root, null, false, movingRoots, result);
        return result;
    }

    private static void collect(SceneNode node, SceneNode environment, boolean movable, Set<SceneNode> roots, List<Body> result) {
        if (environment == null && !node.isRobotPart()) environment = node;
        movable |= roots.contains(node);
        if (node.getCollisionShape() != null) {
            collectShape(node, environment, movable, node.getCollisionShape(), new Matrix4f(), result);
        } else for (var mesh : node.getMeshes()) {
            // Combining separate finger meshes into one box fills the open gripper gap.
            AABB local = new AABB(mesh.getMinBound(), mesh.getMaxBound());
            if (local.isValid()) result.add(new Body(node, environment, movable, local, new ConvexMeshShape(mesh.getVertices()), new Matrix4f()));
        }
        for (SceneNode child : node.getChildren()) collect(child, environment, movable, roots, result);
    }

    private static void collectShape(SceneNode node, SceneNode environment, boolean movable,
                                     CollisionShape shape, Matrix4f localTransform, List<Body> result) {
        if (shape instanceof CompoundShape compound) {
            // Preserve real openings between imported collision parts. A single
            // enclosing box falsely fills the gripper gap and blocks contact.
            for (var entry : compound.getChildren()) collectShape(node, environment, movable,
                    entry.getShape(), new Matrix4f(localTransform).mul(entry.getLocalTransform()), result);
        } else {
            AABB local = shape.getLocalAABB();
            if (local.isValid()) result.add(new Body(node, environment, movable, local, shape, localTransform));
        }
    }

    private static List<Bounds> bounds(List<Body> bodies) {
        List<Bounds> result = new ArrayList<>(bodies.size());
        for (Body body : bodies) {
            Matrix4f transform = body.node.getWorldTransform().mul(body.localTransform);
            OBB box = new OBB();
            box.fromAABBAndTransform(body.local, transform);
            // Nonuniformly scaled, rotated ancestors may introduce shear. An AABB encloses it safely.
            if (Math.abs(box.getAxis(0).dot(box.getAxis(1))) > 0.0001f
                    || Math.abs(box.getAxis(0).dot(box.getAxis(2))) > 0.0001f
                    || Math.abs(box.getAxis(1).dot(box.getAxis(2))) > 0.0001f) {
                AABB world = new AABB();
                body.local.transform(transform, world);
                box.fromAABBAndTransform(world, new Matrix4f());
            }
            result.add(new Bounds(box, transform));
        }
        return result;
    }

    private static String collision(List<Body> bodies, List<Bounds> current, List<Bounds> previous, boolean checkGround) {
        for (int i = 0; i < bodies.size(); i++) {
            Body a = bodies.get(i);
            Bounds ba = current.get(i);
            if (!ba.box.getCenter().isFinite() || !ba.box.getHalfExtents().isFinite()) return "Invalid geometry: " + a.node.getId();
            if (checkGround && a.movable && ba.box.getMinY() < -EPS) {
                boolean unchanged = previous != null && ba.transform.equals(previous.get(i).transform, 1e-7f);
                // Imported bases can already sit a little below the floor. Rotation along the
                // floor must remain possible; reject only new or deeper penetration.
                boolean escaping = previous != null && previous.get(i).box.getMinY() < -EPS
                        && ba.box.getMinY() >= previous.get(i).box.getMinY() - EPS;
                if (!unchanged && !escaping) return a.node.getId() + " intersects the ground.";
            }
            for (int j = i + 1; j < bodies.size(); j++) {
                Body b = bodies.get(j);
                if (a.environment == null && b.environment == null) continue; // Robot self-collision belongs to CollisionWorld.
                if (a.environment != null && a.environment == b.environment) continue;
                if (!a.movable && !b.movable) continue;
                float overlap = penetration(ba.box, current.get(j).box);
                if (overlap <= EPS || !intersects(a, ba, b, current.get(j))) continue;
                if (previous != null) {
                    if (ba.transform.equals(previous.get(i).transform, 1e-7f)
                            && current.get(j).transform.equals(previous.get(j).transform, 1e-7f)) continue;
                    float oldOverlap = penetration(previous.get(i).box, previous.get(j).box);
                    if (oldOverlap > EPS && intersects(a, previous.get(i), b, previous.get(j)) && overlap < oldOverlap - EPS) continue;
                }
                return a.node.getId() + " intersects " + b.node.getId() + ".";
            }
        }
        return null;
    }

    // Bounds are broad phase only: a bent/rounded jaw need not fill its box.
    private static boolean intersects(Body a, Bounds ba, Body b, Bounds bb) {
        // These support maps currently express their radii in world units.
        // Normalizing their transforms would shrink the effective radius and
        // miss real collisions. Retain the conservative broad-phase result.
        if (a.shape instanceof SphereShape || a.shape instanceof CapsuleShape
                || b.shape instanceof SphereShape || b.shape instanceof CapsuleShape) return true;
        Vector3f center = new Vector3f(ba.box.getCenter()).add(bb.box.getCenter()).mul(.5f);
        float size = Math.max(ba.box.getHalfExtents().length(), bb.box.getHalfExtents().length());
        // GJK's absolute numerical thresholds need a stable geometry scale.
        Matrix4f normalize = new Matrix4f().scaling(100f / Math.max(size, .0001f)).translate(center.negate());
        return CollisionMath.intersectGJKConservative(a.shape, new Matrix4f(normalize).mul(ba.transform),
                b.shape, new Matrix4f(normalize).mul(bb.transform));
    }

    /** Minimum normalized separating-axis overlap; negative means separated. */
    private static float penetration(OBB a, OBB b) {
        float minimum = Float.POSITIVE_INFINITY;
        List<Vector3f> axes = new ArrayList<>(15);
        for (int i = 0; i < 3; i++) { axes.add(new Vector3f(a.getAxis(i))); axes.add(new Vector3f(b.getAxis(i))); }
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) axes.add(new Vector3f(a.getAxis(i)).cross(b.getAxis(j)));
        Vector3f delta = new Vector3f(b.getCenter()).sub(a.getCenter());
        for (Vector3f axis : axes) {
            if (axis.lengthSquared() < 1e-10f) continue;
            axis.normalize();
            float overlap = radius(a, axis) + radius(b, axis) - Math.abs(delta.dot(axis));
            if (overlap < 0) return overlap;
            minimum = Math.min(minimum, overlap);
        }
        return minimum;
    }

    private static float radius(OBB box, Vector3f axis) {
        Vector3f h = box.getHalfExtents();
        return Math.abs(box.getAxis(0).dot(axis)) * h.x + Math.abs(box.getAxis(1).dot(axis)) * h.y + Math.abs(box.getAxis(2).dot(axis)) * h.z;
    }

    private static boolean ancestor(SceneNode ancestor, SceneNode node) {
        for (SceneNode n = node; n != null; n = n.getParent()) if (n == ancestor) return true;
        return false;
    }
}
