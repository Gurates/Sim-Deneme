package rsim2.mcp;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import org.joml.Vector3f;
import rsim2.motion.Keyframe;
import rsim2.motion.MotionSequence;
import rsim2.scene.Joint;
import rsim2.scene.ObjectInteractionController;
import rsim2.scene.SceneNode;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

final class McpJointTrajectory {
    record Event(float time, SceneNode object, SceneNode tool, Vector3f offset) {
        boolean isRelease() { return tool == null; }
    }

    record Plan(MotionSequence sequence, List<Event> events) {}

    static Plan build(JsonObject args, SceneNode root, List<Joint> joints) {
        JsonElement input = args.get("waypoints");
        if (args.size() != 1 || input == null || !input.isJsonArray())
            throw new IllegalArgumentException("Provide a waypoints array.");
        JsonArray waypoints = input.getAsJsonArray();
        if (waypoints.isEmpty() || waypoints.size() > 256)
            throw new IllegalArgumentException("Provide between 1 and 256 waypoints.");
        Map<String, Joint> byId = new LinkedHashMap<>();
        Map<String, Float> pose = new LinkedHashMap<>();
        for (Joint joint : joints) {
            byId.put(joint.getId(), joint);
            if (joint.getMotor() != null) pose.put(joint.getId(), (float) Math.toDegrees(joint.getCurrentAngleRadians()));
        }
        if (pose.isEmpty()) throw new IllegalArgumentException("Robot has no movable joints.");
        MotionSequence sequence = new MotionSequence("MCP joint trajectory", .01f, false);
        sequence.addKeyframe(new Keyframe(0, pose));
        List<Event> events = new ArrayList<>();
        float time = 0;
        for (JsonElement waypoint : waypoints) {
            if (!waypoint.isJsonObject()) throw new IllegalArgumentException("Each waypoint must be an object.");
            JsonObject step = waypoint.getAsJsonObject();
            if (step.isEmpty() || !Set.of("seconds", "targets_degrees", "event").containsAll(step.keySet()))
                throw new IllegalArgumentException("A waypoint accepts targets_degrees, seconds and event.");
            float minimum = .03f;
            if (step.has("targets_degrees")) {
                if (!step.get("targets_degrees").isJsonObject()) throw new IllegalArgumentException("Expected joint targets object.");
                for (var entry : step.getAsJsonObject("targets_degrees").entrySet()) {
                    Joint joint = byId.get(entry.getKey());
                    float degrees = McpRobotController.number(entry.getValue());
                    double radians = Math.toRadians(degrees);
                    if (joint == null || joint.getMotor() == null || radians < joint.getMinLimit() - 1e-6
                            || radians > joint.getMaxLimit() + 1e-6)
                        throw new IllegalArgumentException("Unknown, fixed or out-of-range joint: " + entry.getKey());
                    float distance = (float) Math.toRadians(Math.abs(degrees - pose.get(entry.getKey())));
                    float speed = joint.getMotor().getEffectiveCruiseSpeed();
                    float acceleration = joint.getMotor().getAcceleration();
                    if (!(speed > 0) || !Float.isFinite(speed) || !(acceleration > 0) || !Float.isFinite(acceleration))
                        throw new IllegalArgumentException("Invalid motor limits for " + entry.getKey());
                    minimum = Math.max(minimum, Math.max(1.5f * distance / speed, (float) Math.sqrt(6 * distance / acceleration)));
                    pose.put(entry.getKey(), degrees);
                }
            }
            float seconds = step.has("seconds") ? McpRobotController.number(step.get("seconds")) : minimum;
            if (seconds + .00001f < minimum)
                throw new IllegalArgumentException("Waypoint " + sequence.getKeyframes().size()
                        + " requires at least " + minimum + " seconds for the current motor limits.");
            time += seconds;
            if (time > 3600) throw new IllegalArgumentException("Trajectory duration exceeds one hour.");
            sequence.addKeyframe(new Keyframe(time, pose));
            if (step.has("event")) events.add(event(step.getAsJsonObject("event"), root, time));
        }
        return new Plan(sequence, List.copyOf(events));
    }

    static Event event(JsonObject args, SceneNode root, float time) {
        String type = McpRobotController.string(args, "type");
        SceneNode object = ObjectInteractionController.findNode(root, McpRobotController.string(args, "object"));
        if (object == null || object == root || object.isRobotPart())
            throw new IllegalArgumentException("Object must identify an environment object.");
        if ("release".equals(type) && args.size() == 2) return new Event(time, object, null, null);
        if (!"capture".equals(type) || args.size() != 4)
            throw new IllegalArgumentException("Expected capture (object, tool, tool_offset) or release (object).");
        SceneNode tool = McpRobotController.tool(root, args);
        return new Event(time, object, tool, McpRobotController.vector(args.get("tool_offset")));
    }
}
