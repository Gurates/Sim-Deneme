package rsim2.collision;

import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.util.ArrayList;
import java.util.List;

/** Retains the last accepted joint pose across frames, including editor timeline seeks. */
public final class MotionCollisionState {
    private final EnvironmentMotionGuard guard = new EnvironmentMotionGuard();
    private SceneNode sceneRoot;
    private List<Joint> sceneJoints = List.of();
    private float[] accepted = new float[0];
    private boolean initialized;

    public void beginStep(SceneNode root, List<Joint> joints) {
        List<Joint> current = joints == null ? List.of() : joints;
        boolean changed = !initialized || sceneRoot != root || sceneJoints.size() != current.size();
        if (!changed) for (int i = 0; i < current.size(); i++) {
            if (sceneJoints.get(i) != current.get(i)) { changed = true; break; }
        }
        if (changed) {
            sceneRoot = root;
            sceneJoints = new ArrayList<>(current);
            accepted = angles(current);
            initialized = true;
        }
    }

    /** Restores rejected poses and stops their motors. The caller pauses trajectory playback. */
    public String validateAndCommit(SceneNode root, List<Joint> joints) {
        beginStep(root, joints);
        float[] current = angles(sceneJoints);
        String blocked = guard.validateStep(root, sceneJoints, accepted, current);
        if (blocked == null) {
            accepted = current;
        } else {
            for (int i = 0; i < sceneJoints.size(); i++) {
                Joint joint = sceneJoints.get(i);
                joint.setAngle(accepted[i]);
                if (joint.getMotor() != null) {
                    joint.getMotor().setTargetAngleRadians(accepted[i]);
                    joint.getMotor().setCurrentVelocity(0);
                }
            }
        }
        return blocked;
    }

    private static float[] angles(List<Joint> joints) {
        float[] result = new float[joints.size()];
        for (int i = 0; i < result.length; i++) result[i] = joints.get(i).getCurrentAngleRadians();
        return result;
    }
}
