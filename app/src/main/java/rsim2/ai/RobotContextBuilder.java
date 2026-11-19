package rsim2.ai;

import rsim2.scene.SceneObservation;
import org.joml.Vector3f;
import rsim2.collision.CollisionWorld;
import rsim2.motion.MotionPlayer;
import rsim2.motion.MotionSequence;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;
import rsim2.scene.ObjectInteractionController;

import java.util.*;

public class RobotContextBuilder {

    public static String buildContext(SceneNode rootNode, List<Joint> joints, CollisionWorld collisionWorld, MotionPlayer motionPlayer) {
        return buildContext(rootNode, joints, collisionWorld, motionPlayer, null);
    }

    public static String buildContext(SceneNode rootNode, List<Joint> joints, CollisionWorld collisionWorld,
                                      MotionPlayer motionPlayer, ObjectInteractionController interactions) {
        return buildContext(rootNode, joints, collisionWorld, motionPlayer, interactions, null);
    }

    public static String buildContext(SceneNode rootNode, List<Joint> joints, CollisionWorld collisionWorld,
                                      MotionPlayer motionPlayer, ObjectInteractionController interactions, SceneNode selected) {
        StringBuilder sb = new StringBuilder();

        sb.append("You are an expert robotics kinematics and motion planning assistant embedded in RSim2 (3D Robot Simulator & Sim2Real Studio).\n");
        sb.append("You receive the scene state and kinematics below. Reachability and capture must be verified by the local planner; do not assume a target is reachable or a physical grip is proven.\n\n");

        sb.append("=== SCENE OBJECTS (world coordinates, scene units) ===\n");
        sb.append("No dynamics engine: grasping is a simulation-only kinematic attachment. Release holds the world pose; no gravity, force, friction or finger contact simulation.\n");
        sb.append("Grasp uses position-only IK with conservative surface targets and collision-checked joint paths. It may reject blocked approaches; it does not plan finger closure or tool orientation. Use exact object and robot tool IDs; never invent joint angles to grasp.\n");
        var filter = collisionWorld == null ? null : collisionWorld.getFilter();
        var collisions = collisionWorld == null ? null : collisionWorld.getLastResult();
        var summary = SceneObservation.describe(rootNode, joints, selected, interactions, filter, collisions);
        sb.append("End-Effector candidates and environment objects (world frame):\n").append(summary).append('\n');
        sb.append("Use selected_node_id for 'this'. If active_tool_id is null, clarify the tool. Convert physical lengths using meters_per_scene_unit. If it is null, ask for the scale before executing a metric request.\n");
        if (joints == null || joints.isEmpty()) {
            sb.append("=== CURRENT SCENE STATE ===\n");
            sb.append("Status: No robot or active joints currently loaded in the scene.\n");
            sb.append("Prompt the user to load a URDF or project JSON file to begin.\n");
            return sb.toString();
        }

        sb.append("=== ROBOT CONFIGURATION & KINEMATICS ===\n");
        sb.append(String.format(Locale.ROOT, "Degrees of Freedom (Active Joints): %d\n", joints.size()));

        SceneNode baseNode = findBaseNode(joints);


        Vector3f basePos = new Vector3f();
        if (baseNode != null) {
            baseNode.getWorldTransform().getTranslation(basePos);
            sb.append(String.format(Locale.ROOT, "Base Node: \"%s\" at World Pos (X=%.3f, Y=%.3f, Z=%.3f)\n", baseNode.getId(), basePos.x, basePos.y, basePos.z));
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

            sb.append(String.format(Locale.ROOT, "Joint [%d] \"%s\" (%s):\n", i + 1, j.getId(), j.getType().name()));
            sb.append(String.format(Locale.ROOT, "  - Link Connection: %s -> %s\n", parentId, childId));
            sb.append(String.format(Locale.ROOT, "  - Rotation Axis: (%.1f, %.1f, %.1f)\n", axis.x, axis.y, axis.z));
            sb.append(String.format(Locale.ROOT, "  - Current Angle: %.2f° (Range: [%.1f°, %.1f°])\n", currentDeg, minDeg, maxDeg));
            sb.append(String.format(Locale.ROOT, "  - Link 3D World Pos: (X=%.3f, Y=%.3f, Z=%.3f)\n", childPos.x, childPos.y, childPos.z));

            if (j.getMotor() != null) {
                sb.append(String.format(Locale.ROOT, "  - Motor: Speed=%.0f%%, Accel=%.1f rad/s²",
                        j.getMotor().getTargetSpeedRatio() * 100.0f, j.getMotor().getAcceleration()));
                if (j.getPin() >= 0) {
                    sb.append(String.format(Locale.ROOT, ", Hardware Pin=%d", j.getPin()));
                }
                sb.append("\n");
            }
        }

        sb.append("\n=== SAFETY & COLLISION STATUS ===\n");
        String status = summary.get("collision_status").getAsString();
        sb.append("Collision Status: ").append(status).append('\n');
        if ("contacts".equals(status)) {
            for (var contact : collisions.getContacts()) {
                sb.append(String.format("  - Contact between \"%s\" and \"%s\" (type: %s)\n",
                        contact.getNodeA() == null ? "Unknown" : contact.getNodeA().getId(),
                        contact.getNodeB() == null ? "Ground" : contact.getNodeB().getId(), contact.getType()));
            }
        }

        boolean eStopActive = rsim2.bridge.BridgeManager.getInstance().getSafetyFilter().isEmergencyStopActive();
        sb.append(String.format(Locale.ROOT, "Hardware E-Stop: %s\n", eStopActive ? "ACTIVATED" : "INACTIVE"));

        sb.append("\n=== MOTION PLAYER STATE ===\n");
        if (motionPlayer != null && motionPlayer.hasSequence()) {
            MotionSequence seq = motionPlayer.getSequence();
            sb.append(String.format(Locale.ROOT, "Active Sequence: \"%s\" (Duration: %.2fs, Loop: %s, Playing: %s, Current Time: %.2fs)\n",
                    seq.getName(), seq.getDurationSeconds(), seq.isLoop(), motionPlayer.isPlaying(), motionPlayer.getCurrentTime()));
        } else {
            sb.append("Motion Sequence: None currently loaded.\n");
        }

        sb.append("\n=== ACTION PROTOCOL & TOOL EXECUTION ===\n");
        sb.append("Playback: start/basla resumes the loaded motion, pause/duraklat pauses, stop/durdur stops. Never interpret start as the HOME pose. Direct joint/pose and grasp approaches create replayable toolbar motions. If an approach was rejected there is no new motion to play; ask for a new goal or corrected target.\n");
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

        sb.append("Return one executable JSON command for the entire user request. Put successive joint poses in ordered keyframes of one animation. For a request to approach, grasp and lift, use one grasp_and_lift interaction that includes all three stages. Always verify joint angular limits and ensure movements are smooth.\n");
        sb.append("\n5. OBJECT INTERACTION (send as a standalone command, never combine with pose/actions/animation):\n");
        sb.append("APPROACH WITHOUT GRASP: for 'Simply bring the robotic arm closer to the box, as close as possible', 'move closer', or 'sadece yaklas', emit type=approach. Do not use grasp, attach, or demand a reachable grasp surface for these requests. The local planner may execute a safe partial advance if it cannot reach the surface; it reports measured TCP distance and stops before penetration. It is a sampled direct approach, not a proof of globally closest reach.\n");
        sb.append("```json\n{\"interaction\":{\"type\":\"approach\",\"object\":\"exact_object_id\",\"tool\":\"exact_robot_link_id\"}}\n```\n");
        sb.append("For 'approach the box and grasp it', emit the grasp JSON below: it performs both approach and attachment. A conversational promise alone cannot move the robot. Never claim execution before the simulator reports it.\n");
        sb.append("```json\n{\"interaction\":{\"type\":\"grasp\",\"object\":\"exact_object_id\",\"tool\":\"exact_robot_link_id\",\"tool_offset\":[0,0,0]}}\n```\n");
        sb.append("For approach, omit tool_offset to use the configured TCP or geometry hint. For grasp, omit it or use zero for automatic contact selection; a configured TCP takes precedence. Explicit nonzero offsets are tool-local scene units.\n");
        sb.append("For feasibility questions or choosing the nearest reachable object, first return a standalone read-only query: {\"preview_approach\":{\"tool\":\"tool_id\",\"objects\":[\"object_id_1\",\"object_id_2\"]}}. Query up to 8 relevant candidates. Results describe a sampled direct approach, not proof of graspability. Partial means it can get closer, not that the surface was reached.\n");
        sb.append("The simulator chooses an external surface approach accounting for tool geometry, solves joint-limited IK and checks intermediate poses. It attaches only near the chosen external target without penetration. All motor/animation steps are checked against environment bounds; blocked motion is stopped. Report approaching until completion, not immediate success. Rejected goals do not change the pose.\n");
        sb.append("```json\n{\"interaction\":{\"type\":\"release\",\"object\":\"exact_object_id\"}}\n```\n");
        sb.append("For grasp and lift, pick up, or kavra ve kaldır, emit interaction.type=grasp_and_lift with object, tool, and optional positive lift_distance (default 0.05 scene units, world +Y). The simulator waits for verified capture, plans a collision-checked upward lift, and reports completion only after measured object rise. Never split this into separate simultaneous grasp and joint commands. If already held, grasp_and_lift lifts the held object without regrasping. For lift-only of a held object use interaction.type=lift with object and optional lift_distance. Do not claim success from a loaded plan; report pending until simulator completion. This does not command or verify a physical hardware gripper.\n");

        sb.append("```json\n{\"interaction\":{\"type\":\"grasp_and_lift\",\"object\":\"exact_object_id\",\"tool\":\"exact_robot_link_id\",\"lift_distance\":0.05}}\n```\n");
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

}
