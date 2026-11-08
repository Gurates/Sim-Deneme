package rsim2.ai;

import org.joml.Vector3f;
import rsim2.motion.JointMotion;
import rsim2.motion.MotionPlayer;
import rsim2.motion.MotionSequence;
import rsim2.scene.Joint;
import rsim2.scene.ObjectInteractionController;
import rsim2.scene.SceneNode;

import java.util.List;
import java.util.Locale;

/** Advances a single simulator task only after verified capture and measured lift. */
public final class GraspLiftCoordinator {
    private SceneNode root, object;
    private String objectId;
    private List<Joint> joints;
    private ObjectInteractionController interactions;
    private MotionPlayer player;
    private MotionSequence sequence;
    private long stopGeneration;
    private float height, initialY;
    private boolean lifting;

    public boolean isActive() { return object != null; }

    public String state() {
        return !isActive() ? "none" : (lifting ? "lifting " : "approaching to grasp ") + objectId;
    }

    public String start(SceneNode root, List<Joint> joints, ObjectInteractionController interactions,
                        MotionPlayer player, String objectId, String toolId, Vector3f offset,
                        float height, boolean graspFirst) {
        if (isActive()) throw new IllegalArgumentException("A grasp-and-lift task is already active. Stop it before replacing it.");
        if (player == null || joints == null) throw new IllegalArgumentException("Lift requires a motion player and robot joints.");
        if (player.isPlaying()) throw new IllegalArgumentException("Pause the current motion before planning a lift.");
        if (!Float.isFinite(height) || height <= 0) throw new IllegalArgumentException("lift_distance must be finite and positive.");
        SceneNode target = ObjectInteractionController.findNode(root, objectId);
        if (target == null) throw new IllegalArgumentException("Environment object not found: " + objectId);
        float[] start = JointMotion.angles(joints);
        boolean alreadyHeld = interactions.isHeld(target);
        if (!graspFirst && !alreadyHeld) throw new IllegalArgumentException("Lift requires a verified held object. Use grasp_and_lift first.");
        String planned = alreadyHeld ? interactions.lift(root, joints, objectId, height)
                : interactions.grasp(root, joints, objectId, toolId, offset);
        this.root = root;
        this.object = target;
        this.objectId = objectId;
        this.joints = List.copyOf(joints);
        this.interactions = interactions;
        this.player = player;
        this.height = height;
        this.initialY = ObjectInteractionController.objectCenter(target).y;
        this.lifting = alreadyHeld;
        this.stopGeneration = player.getStopGeneration();
        load(start);
        return planned + (alreadyHeld ? " Lift is running; completion requires measured upward movement."
                : " Lift is queued until capture is verified.");
    }

    private void load(float[] start) {
        sequence = JointMotion.fromTargets((lifting ? "Lift " : "Grasp ") + objectId, joints, start);
        player.bindJoints(joints);
        if (sequence != null) {
            player.loadSequence(sequence);
            player.play();
            if (!lifting) interactions.bindPlayback(player, sequence);
        }
    }

    /** Call after ObjectInteractionController.update on the simulation thread. */
    public String update(SceneNode currentRoot, boolean stopped, String interactionEvent) {
        if (!isActive()) return null;
        String interruption = cancelIfInterrupted(currentRoot, stopped);
        if (interruption != null) return interruption;
        if (!lifting) {
            if (!interactions.isHeld(object)) {
                if (interactionEvent != null) return cancel(interactionEvent);
                return null;
            }
            float[] start = JointMotion.angles(joints);
            initialY = ObjectInteractionController.objectCenter(object).y;
            try {
                String planned = interactions.lift(root, joints, objectId, height);
                lifting = true;
                load(start);
                return planned + " Capture verified; lifting now.";
            } catch (RuntimeException error) {
                return cancel("capture succeeded but lift could not be planned: " + error.getMessage());
            }
        }
        if (!interactions.isHeld(object)) return cancel("object is no longer held");
        if (sequence != null && (player.isPlaying() || player.getCurrentTime() < sequence.getDurationSeconds())) return null;
        float rise = ObjectInteractionController.objectCenter(object).y - initialY;
        float tolerance = Math.min(0.006f, height * 0.1f);
        if (rise < height - tolerance) return cancel(String.format(Locale.ROOT,
                "lift finished short: measured rise %.4f, requested %.4f scene units", rise, height));
        String event = String.format(Locale.ROOT, "Grasp-and-lift completed: %s rose %.4f scene units (requested %.4f).", objectId, rise, height);
        clear();
        return event;
    }

    /** Check before capture update so Stop can never produce a late attachment. */
    public String cancelIfInterrupted(SceneNode currentRoot, boolean stopped) {
        if (!isActive()) return null;
        if (stopped || currentRoot != root || player.getStopGeneration() != stopGeneration
                || (sequence != null && player.getSequence() != sequence)
                || ObjectInteractionController.findNode(root, objectId) != object) {
            return cancel("stopped, scene changed, or motion replaced");
        }
        return null;
    }

    public String cancel(String reason) {
        if (!isActive()) return null;
        String event = "Grasp-and-lift cancelled: " + reason + ".";
        interactions.cancelPending();
        if (player.getSequence() == sequence) player.pause();
        clear();
        return event;
    }

    private void clear() {
        root = null;
        object = null;
        sequence = null;
    }
}
