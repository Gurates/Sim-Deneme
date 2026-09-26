package rsim2.ai;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import rsim2.collision.CollisionWorld;
import rsim2.motion.MotionPlayer;
import rsim2.motion.MotionSequence;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.util.*;

public class RobotContextBuilder {

    public static String buildContext(SceneNode rootNode, List<Joint> joints, CollisionWorld collisionWorld, MotionPlayer motionPlayer) {
        StringBuilder sb = new StringBuilder();

        sb.append("You are an expert robotics kinematics and motion planning assistant embedded in RSim2 (3D Robot Simulator & Sim2Real Studio).\n");
        sb.append("You have full spatial awareness of the 3D scene, joint kinematics, forward kinematics (FK), collisions, and hardware controls.\n\n");

        if (joints == null || joints.isEmpty()) {
            sb.append("=== CURRENT SCENE STATE ===\n");
            sb.append("Status: No robot or active joints currently loaded in the scene.\n");
            sb.append("Prompt the user to load a URDF or project JSON file to begin.\n");
            return sb.toString();
        }

        sb.append("=== ROBOT CONFIGURATION & KINEMATICS ===\n");
        sb.append(String.format("Degrees of Freedom (Active Joints): %d\n", joints.size()));

        // Identify Base and End-Effector
        SceneNode baseNode = findBaseNode(joints);
        SceneNode endEffector = findEndEffector(joints);

        Vector3f basePos = new Vector3f();
        if (baseNode != null) {
            baseNode.getWorldTransform().getTranslation(basePos);
            sb.append(String.format("Base Node: \"%s\" at World Pos (X=%.3f, Y=%.3f, Z=%.3f)\n", baseNode.getId(), basePos.x, basePos.y, basePos.z));
        }

        Vector3f eePos = new Vector3f();
        if (endEffector != null) {
            endEffector.getWorldTransform().getTranslation(eePos);
            float reach = basePos.distance(eePos);
            sb.append(String.format("End-Effector (Tool/Tip): \"%s\" at World Pos (X=%.3f, Y=%.3f, Z=%.3f) | Current Reach: %.3f meters\n",
                    endEffector.getId(), eePos.x, eePos.y, eePos.z, reach));
        }

        sb.append("\n=== DETAILED JOINTS & 3D FORWARD KINEMATICS ===\n");
        for (int i = 0; i < joints.size(); i++) {
            Joint j = joints.get(i);
            float currentDeg = (float) Math.toDegrees(j.getCurrentAngleRadians());
            float minDeg = (float) Math.toDegrees(j.getMinLimit());
            float maxDeg = (float) Math.toDegrees(j.getMaxLimit());

            Vector3f childPos = new Vector3f();
            if (j.getChildNode() != null) {
                j.getChildNode().getWorldTransform().getTranslation(childPos);
            }

            Vector3f axis = j.getAxis();
            String parentId = j.getParentNode() != null ? j.getParentNode().getId() : "world";
            String childId = j.getChildNode() != null ? j.getChildNode().getId() : "null";

            sb.append(String.format("Joint [%d] \"%s\" (%s):\n", i + 1, j.getId(), j.getType().name()));
            sb.append(String.format("  - Link Connection: %s -> %s\n", parentId, childId));
            sb.append(String.format("  - Rotation Axis: (%.1f, %.1f, %.1f)\n", axis.x, axis.y, axis.z));
            sb.append(String.format("  - Current Angle: %.2f° (Range: [%.1f°, %.1f°])\n", currentDeg, minDeg, maxDeg));
            sb.append(String.format("  - Link 3D World Pos: (X=%.3f, Y=%.3f, Z=%.3f)\n", childPos.x, childPos.y, childPos.z));

            if (j.getMotor() != null) {
                sb.append(String.format("  - Motor: Speed=%.0f%%, Accel=%.1f rad/s²",
                        j.getMotor().getTargetSpeedRatio() * 100.0f, j.getMotor().getAcceleration()));
                if (j.getPin() >= 0) {
                    sb.append(String.format(", Hardware Pin=%d", j.getPin()));
                }
                sb.append("\n");
            }
        }

        sb.append("\n=== SAFETY & COLLISION STATUS ===\n");
        if (collisionWorld != null && collisionWorld.getLastResult() != null) {
            boolean hasCol = collisionWorld.getLastResult().hasCollision();
            if (hasCol) {
                sb.append(String.format("ALERT: Active Collisions Detected (%d contacts):\n", collisionWorld.getLastResult().getCollisionCount()));
                if (collisionWorld.getLastResult().getContacts() != null) {
                    for (rsim2.collision.ContactPair contact : collisionWorld.getLastResult().getContacts()) {
                        sb.append(String.format("  - Contact between \"%s\" and \"%s\" (type: %s)\n",
                                contact.getNodeA().getId(), contact.getNodeB().getId(), contact.getType().name()));
                    }
                }
            } else {
                sb.append("Collision Status: CLEAR (No self-collision or ground intersection detected).\n");
            }
        } else {
            sb.append("Collision Status: CLEAR.\n");
        }

        boolean eStopActive = rsim2.bridge.BridgeManager.getInstance().getSafetyFilter().isEmergencyStopActive();
        sb.append(String.format("Hardware E-Stop: %s\n", eStopActive ? "ACTIVATED" : "INACTIVE"));

        sb.append("\n=== MOTION PLAYER STATE ===\n");
        if (motionPlayer != null && motionPlayer.hasSequence()) {
            MotionSequence seq = motionPlayer.getSequence();
            sb.append(String.format("Active Sequence: \"%s\" (Duration: %.2fs, Loop: %s, Playing: %s, Current Time: %.2fs)\n",
                    seq.getName(), seq.getDurationSeconds(), seq.isLoop(), motionPlayer.isPlaying(), motionPlayer.getCurrentTime()));
        } else {
            sb.append("Motion Sequence: None currently loaded.\n");
        }

        sb.append("\n=== ACTION PROTOCOL & TOOL EXECUTION ===\n");
        sb.append("When the user asks you to move the robot, assume a pose, wave, clear collisions, or execute motion, you MUST provide your clear conversational response first, and then append a JSON block formatted exactly as below:\n\n");

        sb.append("1. PRESET POSES (Home / Zero / Rest):\n");
        sb.append("```json\n");
        sb.append("{\n");
        sb.append("  \"pose\": \"home\"\n");
        sb.append("}\n");
        sb.append("```\n");
        sb.append("(Supported poses: \"home\" [all joints to 0°/home], \"zero\", \"rest\")\n\n");

        sb.append("2. MULTI-JOINT COORDINATED POSITIONING:\n");
        sb.append("```json\n");
        sb.append("{\n");
        sb.append("  \"actions\": [\n");
        sb.append("    { \"joint\": \"joint_id_1\", \"target_deg\": 45.0, \"speed_pct\": 70 },\n");
        sb.append("    { \"joint\": \"joint_id_2\", \"target_deg\": -30.0 }\n");
        sb.append("  ]\n");
        sb.append("}\n");
        sb.append("```\n\n");

        sb.append("3. TIMED TRAJECTORY / ANIMATION (Gestures, Waving, Pick-and-Place, Walk):\n");
        sb.append("```json\n");
        sb.append("{\n");
        sb.append("  \"animation\": {\n");
        sb.append("    \"name\": \"Wave Gesture\",\n");
        sb.append("    \"duration\": 2.0,\n");
        sb.append("    \"loop\": true,\n");
        sb.append("    \"keyframes\": [\n");
        sb.append("      { \"time\": 0.0, \"joints\": { \"joint_1\": 0.0, \"joint_2\": 20.0 } },\n");
        sb.append("      { \"time\": 1.0, \"joints\": { \"joint_1\": 35.0, \"joint_2\": 40.0 } },\n");
        sb.append("      { \"time\": 2.0, \"joints\": { \"joint_1\": 0.0, \"joint_2\": 20.0 } }\n");
        sb.append("    ]\n");
        sb.append("  }\n");
        sb.append("}\n");
        sb.append("```\n\n");

        sb.append("4. EMERGENCY STOP:\n");
        sb.append("```json\n");
        sb.append("{\n");
        sb.append("  \"system\": {\n");
        sb.append("    \"estop\": true\n");
        sb.append("  }\n");
        sb.append("}\n");
        sb.append("```\n\n");

        sb.append("Always verify joint angular limits before commanding angles, and ensure movements are smooth and physically sound.\n");

        return sb.toString();
    }

    private static SceneNode findBaseNode(List<Joint> joints) {
        if (joints == null || joints.isEmpty()) return null;
        Set<SceneNode> children = new HashSet<>();
        for (Joint j : joints) {
            if (j.getChildNode() != null) {
                children.add(j.getChildNode());
            }
        }
        for (Joint j : joints) {
            if (j.getParentNode() != null && !children.contains(j.getParentNode())) {
                return j.getParentNode();
            }
        }
        return joints.get(0).getParentNode();
    }

    private static SceneNode findEndEffector(List<Joint> joints) {
        if (joints == null || joints.isEmpty()) return null;
        Set<SceneNode> parents = new HashSet<>();
        for (Joint j : joints) {
            if (j.getParentNode() != null) {
                parents.add(j.getParentNode());
            }
        }
        for (int i = joints.size() - 1; i >= 0; i--) {
            Joint j = joints.get(i);
            if (j.getChildNode() != null && !parents.contains(j.getChildNode())) {
                return j.getChildNode();
            }
        }
        return joints.get(joints.size() - 1).getChildNode();
    }
}
