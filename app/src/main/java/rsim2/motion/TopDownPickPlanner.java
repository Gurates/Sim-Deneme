package rsim2.motion;

import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;
import rsim2.collision.EnvironmentMotionGuard;
import rsim2.scene.*;
import java.util.*;

/** Offline Cartesian waypoints + orientation IK, compiled to one collision-checked sequence. Y is world-up. */
public final class TopDownPickPlanner {
    public record TcpWaypoint(Vector3f position, Vector3f direction) {}
    public record Plan(MotionSequence sequence, float rotateTime, float descendTime, float captureTime,
                       SceneNode object, SceneNode tool, Vector3f offset, Vector3f axis,
                       Matrix4f objectPose, float initialObjectY, float liftDistance) {}
    private final EnvironmentMotionGuard guard = new EnvironmentMotionGuard();
    private float time;
    private MotionSequence motion;
    private long deadline;

    public MotionSequence planTcpPath(SceneNode root, List<Joint> joints, SceneNode tool, Vector3f offset,
                                      Vector3f axis, List<TcpWaypoint> waypoints) {
        if (!offset.isFinite() || axis != null && (!axis.isFinite() || !Float.isFinite(axis.lengthSquared())
                || axis.lengthSquared() < .0001f)) throw new IllegalArgumentException("Invalid tool calibration.");
        List<Joint> chain = new ArrayList<>();
        for (SceneNode node = tool; node != null; node = node.getParent()) {
            for (Joint joint : joints) {
                if (joint.getChildNode() == node && joint.getMotor() != null) chain.add(joint);
            }
        }
        if (chain.isEmpty()) throw new IllegalArgumentException("Tool has no movable joints.");
        float[] initial = JointMotion.angles(joints);
        time = 0;
        motion = new MotionSequence("MCP Cartesian trajectory", .01f, false);
        deadline = System.nanoTime() + 2_000_000_000L;
        try {
            key(joints, 0);
            for (TcpWaypoint target : waypoints) {
                Vector3f from = point(tool, offset);
                Vector3f facing = axis == null ? null : direction(tool, axis);
                Vector3f endDirection = target.direction() == null ? facing : target.direction();
                Quaternionf rotation = facing == null ? null : new Quaternionf().rotationTo(facing, endDirection);
                float length = from.distance(target.position());
                if (!Float.isFinite(length)) throw new IllegalArgumentException("Invalid Cartesian path length.");
                float radians = facing == null ? 0 : (float) Math.acos(Math.max(-1, Math.min(1, facing.dot(endDirection))));
                int steps = Math.max(1, Math.max((int) Math.ceil(length / .05f), (int) Math.ceil(radians / .15f)));
                if (steps > 240) throw new IllegalArgumentException("Cartesian segment is too long. Add intermediate waypoints.");
                for (int i = 1; i <= steps; i++) {
                    float fraction = i / (float) steps;
                    Vector3f desired = rotation == null ? null
                            : new Quaternionf().slerp(rotation, fraction).transform(new Vector3f(facing));
                    waypoint(root, joints, chain, tool, offset, axis,
                            new Vector3f(from).lerp(target.position(), fraction), desired);
                }
            }
            motion.setDurationSeconds(time);
            return motion;
        } finally {
            for (int i = 0; i < joints.size(); i++) joints.get(i).setAngle(initial[i]);
        }
    }

    public Plan plan(SceneNode root, List<Joint> joints, String objectId, String toolId,
                     Vector3f offset, Vector3f axis, float clearance, float lift) {
        if (!offset.isFinite() || !axis.isFinite() || !Float.isFinite(axis.lengthSquared()) || axis.lengthSquared() < .0001f
                || !Float.isFinite(clearance) || clearance <= .01f || !Float.isFinite(lift) || lift <= 0)
            throw new IllegalArgumentException("Finite TCP offset, nonzero local approach axis, clearance > .01 and positive lift are required.");
        SceneNode object = ObjectInteractionController.findNode(root, objectId);
        SceneNode tool = ObjectInteractionController.findNode(root, toolId);
        if (object == null || !ObjectInteractionController.objects(root).contains(object)) throw new IllegalArgumentException("Object must be a free environment object.");
        for (SceneNode ancestor = object.getParent(); ancestor != null && ancestor != root; ancestor = ancestor.getParent())
            if (ancestor.isRobotPart()) throw new IllegalArgumentException("Object is already attached to the robot; release it before planning another pick.");
        if (tool == null || tool == root || !tool.isRobotPart()) throw new IllegalArgumentException("Invalid robot tool.");
        List<Joint> chain = new ArrayList<>();
        for (SceneNode node = tool; node != null; node = node.getParent())
            for (Joint joint : joints) if (joint.getChildNode() == node && joint.getMotor() != null) chain.add(joint);
        if (chain.isEmpty()) throw new IllegalArgumentException("Tool has no movable joints.");
        axis = new Vector3f(axis).normalize();
        Vector3f startDirection = direction(tool, axis);
        Vector3f down = new Vector3f(0, -1, 0);
        Vector3f startPoint = point(tool, offset);
        var bounds = ObjectInteractionController.objectBounds(object);
        Vector3f contact = bounds.getCenter();
        contact.y = bounds.getMax().y + .001f;
        Vector3f hover = new Vector3f(contact).add(0, clearance, 0);
        Matrix4f objectPose = object.getWorldTransform();
        SceneNode parent = object.getParent();
        int childIndex = parent.indexOfChild(object);
        Vector3f savedPosition = new Vector3f(object.getLocalPosition());
        Vector3f savedScale = new Vector3f(object.getLocalScale());
        Quaternionf savedRotation = new Quaternionf(object.getLocalRotation());
        float[] initial = JointMotion.angles(joints);
        time = 0;
        motion = new MotionSequence("Top-down pick: " + objectId, .01f, false);
        deadline = System.nanoTime() + 2_000_000_000L;
        try {
            key(joints, 0);
            Vector3f from = new Vector3f(startPoint);
            for (int i = 1; i <= 12; i++) waypoint(root, joints, chain, tool, offset, axis,
                    new Vector3f(from).lerp(hover, i / 12f), startDirection);
            float rotate = time;
            Quaternionf turn = new Quaternionf().rotationTo(startDirection, down);
            for (int i = 1; i <= 12; i++) {
                Vector3f targetDirection = new Quaternionf().slerp(turn, i / 12f).transform(new Vector3f(startDirection));
                waypoint(root, joints, chain, tool, offset, axis, hover, targetDirection);
            }
            float descend = time;
            for (int i = 1; i <= 16; i++) waypoint(root, joints, chain, tool, offset, axis,
                    new Vector3f(hover).lerp(contact, i / 16f), down);
            float capture = time;
            // Verify capture and held-object clearance during planning, then roll everything back.
            ObjectInteractionController verification = new ObjectInteractionController();
            verification.captureAtContact(root, joints, object, tool, offset);
            for (int i = 1; i <= 16; i++) waypoint(root, joints, chain, tool, offset, axis,
                    new Vector3f(contact).add(0, lift * i / 16f, 0), down);
            motion.setDurationSeconds(time);
            return new Plan(motion, rotate, descend, capture, object, tool, new Vector3f(offset), axis,
                    objectPose, bounds.getCenter().y, lift);
        } finally {
            if (object.getParent() != parent) {
                object.getParent().removeChild(object);
                parent.addChild(childIndex, object);
            }
            object.getLocalPosition().set(savedPosition);
            object.getLocalRotation().set(savedRotation);
            object.getLocalScale().set(savedScale);
            for (int i = 0; i < joints.size(); i++) joints.get(i).setAngle(initial[i]);
        }
    }

    private void waypoint(SceneNode root, List<Joint> joints, List<Joint> chain, SceneNode tool,
                          Vector3f offset, Vector3f axis, Vector3f target, Vector3f targetAxis) {
        float[] previous = JointMotion.angles(joints);
        solve(chain, tool, offset, axis, target, targetAxis);
        if (System.nanoTime() > deadline) throw new IllegalArgumentException("Pick planning timed out; no motion applied.");
        float[] next = JointMotion.angles(joints);
        String blocked = guard.validateStep(root, joints, previous, next);
        if (blocked != null) throw new IllegalArgumentException("Cartesian path blocked: " + blocked);
        float duration = .03f;
        for (int i = 0; i < joints.size(); i++) duration = Math.max(duration,
                1.5f * Math.abs(next[i] - previous[i]) / Math.max(.01f, joints.get(i).getMaxSpeed() * joints.get(i).getTargetSpeedRatio()));
        time += duration;
        key(joints, time);
    }

    private void key(List<Joint> joints, float at) {
        Keyframe key = new Keyframe(at);
        for (Joint joint : joints) key.addJointAngle(joint.getId(), (float) Math.toDegrees(joint.getCurrentAngleRadians()));
        motion.addKeyframe(key);
    }

    private void solve(List<Joint> chain, SceneNode tool, Vector3f offset, Vector3f axis, Vector3f target, Vector3f desiredAxis) {
        for (int iteration = 0; iteration < 220; iteration++) {
            if (System.nanoTime() > deadline) throw new IllegalArgumentException("Pick planning timed out; no motion applied.");
            Vector3f p = point(tool, offset), d = axis == null ? new Vector3f() : direction(tool, axis);
            if (p.distance(target) <= .0015f && (desiredAxis == null || d.dot(desiredAxis) >= (float) Math.cos(Math.toRadians(2)))) return;
            Vector3f directionError = desiredAxis == null ? new Vector3f() : new Vector3f(desiredAxis).sub(d).mul(.2f);
            double[] error = {target.x-p.x, target.y-p.y, target.z-p.z,
                    directionError.x, directionError.y, directionError.z};
            double[][] jacobian = new double[6][chain.size()];
            for (int j = 0; j < chain.size(); j++) {
                Joint joint = chain.get(j);
                float angle = joint.getCurrentAngleRadians();
                joint.setAngle(angle + .001f);
                Vector3f dp = point(tool, offset).sub(p).div(.001f);
                Vector3f dd = axis == null ? new Vector3f() : direction(tool, axis).sub(d).div(.001f).mul(.2f);
                joint.setAngle(angle);
                jacobian[0][j]=dp.x; jacobian[1][j]=dp.y; jacobian[2][j]=dp.z;
                jacobian[3][j]=dd.x; jacobian[4][j]=dd.y; jacobian[5][j]=dd.z;
            }
            double[][] system = new double[6][7];
            for (int row=0; row<6; row++) {
                for (int col=0; col<6; col++) {
                    for (int j=0; j<chain.size(); j++) system[row][col] += jacobian[row][j]*jacobian[col][j];
                    if (row==col) system[row][col] += .0001;
                }
                system[row][6] = error[row];
            }
            for (int col=0; col<6; col++) {
                int pivot=col;
                for (int row=col+1; row<6; row++) if (Math.abs(system[row][col])>Math.abs(system[pivot][col])) pivot=row;
                double[] swap=system[col]; system[col]=system[pivot]; system[pivot]=swap;
                double divisor=system[col][col];
                for (int k=col; k<=6; k++) system[col][k]/=divisor;
                for (int row=0; row<6; row++) if (row!=col) {
                    double factor=system[row][col];
                    for (int k=col; k<=6; k++) system[row][k]-=factor*system[col][k];
                }
            }
            for (int j=0; j<chain.size(); j++) {
                double delta=0;
                for (int row=0; row<6; row++) delta+=jacobian[row][j]*system[row][6];
                Joint joint=chain.get(j);
                float next=joint.getCurrentAngleRadians()+(float)Math.max(-.15,Math.min(.15,delta));
                joint.setAngle(Math.max(joint.getMinLimit(),Math.min(joint.getMaxLimit(),next)));
            }
        }
        throw new IllegalArgumentException("No reachable Cartesian waypoint with the requested tool orientation. Check reach, limits and calibration.");
    }

    public static Vector3f point(SceneNode tool, Vector3f offset) { return tool.getWorldTransform().transformPosition(new Vector3f(offset)); }
    public static Vector3f direction(SceneNode tool, Vector3f axis) { return tool.getWorldTransform().transformDirection(new Vector3f(axis)).normalize(); }
}
