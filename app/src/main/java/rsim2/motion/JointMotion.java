package rsim2.motion;

import rsim2.scene.Joint;
import java.util.List;


public final class JointMotion {
    private JointMotion() {}

    public static float[] angles(List<Joint> joints) {
        float[] values = new float[joints.size()];
        for (int i = 0; i < values.length; i++) values[i] = joints.get(i).getCurrentAngleRadians();
        return values;
    }

    public static MotionSequence fromTargets(String name, List<Joint> joints, float[] start) {
        float duration = 0.5f;
        boolean changed = false;
        for (int i = 0; i < joints.size(); i++) {
            Joint joint = joints.get(i);
            float target = joint.getMotor() == null ? joint.getCurrentAngleRadians() : joint.getMotor().getTargetAngleRadians();
            float distance = Math.abs(target - start[i]);
            changed |= distance > 0.0001f;
            duration = Math.max(duration, 1.5f * distance / Math.max(0.01f, joint.getMaxSpeed() * joint.getTargetSpeedRatio()));
        }
        if (!changed) return null;
        MotionSequence seq = new MotionSequence(name, duration, false);

        for (int frame = 0; frame <= 30; frame++) {
            float t = frame / 30f;
            float blend = t * t * (3 - 2 * t);
            Keyframe key = new Keyframe(duration * t);
            for (int i = 0; i < joints.size(); i++) {
                Joint joint = joints.get(i);
                float target = joint.getMotor() == null ? joint.getCurrentAngleRadians() : joint.getMotor().getTargetAngleRadians();
                key.addJointAngle(joint.getId(), (float) Math.toDegrees(start[i] + (target - start[i]) * blend));
            }
            seq.addKeyframe(key);
        }
        return seq;
    }

    public static MotionSequence forInteraction(String name, List<Joint> joints, float[] start) {
        MotionSequence motion = fromTargets(name, joints, start);
        if (motion != null) return motion;
        motion = new MotionSequence(name, 0.01f, false);
        Keyframe frame = new Keyframe(0);
        for (int i = 0; i < joints.size(); i++) {
            frame.addJointAngle(joints.get(i).getId(), (float) Math.toDegrees(start[i]));
        }
        motion.addKeyframe(frame);
        return motion;
    }
}
