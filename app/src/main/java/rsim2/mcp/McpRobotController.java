package rsim2.mcp;

import com.google.gson.*;
import rsim2.bridge.BridgeManager;
import rsim2.collision.CollisionResult;
import rsim2.motion.*;
import rsim2.scene.*;
import org.joml.Vector3f;
import java.util.*;

/** All methods are called on the simulation thread. Tracks the most recent MCP motion. */
public final class McpRobotController {
    private String taskId;
    private String taskState = "idle";
    private String detail = "";
    private MotionSequence sequence;
    private SceneNode taskRoot;
    private List<Joint> taskJoints = List.of();
    private long stopGeneration;
    private long seekGeneration;
    private List<McpJointTrajectory.Event> events = List.of();
    private int nextEvent;
    private boolean advancedTrajectory;
    private SceneNode interactionRoot;
    private TopDownPickPlanner.Plan pick;
    private boolean captured;
    private ObjectInteractionController interactions = new ObjectInteractionController();

    public void useInteractions(SceneNode root, ObjectInteractionController controller) {
        interactionRoot = root;
        interactions = controller;
    }

    public JsonObject execute(String tool, JsonObject args, SceneNode root, List<Joint> joints,
                              MotionPlayer player, boolean aiBusy, Runnable takeControl) {
        return execute(tool, args, root, joints, player, null, aiBusy, takeControl);
    }

    public JsonObject execute(String tool, JsonObject args, SceneNode root, List<Joint> joints,
                              MotionPlayer player, CollisionResult collisions, boolean aiBusy, Runnable takeControl) {
        if (root != interactionRoot) {
            interactions = new ObjectInteractionController();
            interactionRoot = root;
        }
        if (!Set.of("move_joints", "pick_object_top_down", "release_object", "execute_trajectory",
                "move_tcp", "capture_object", "get_geometry").contains(tool) && !args.isEmpty())
            throw new IllegalArgumentException("This tool accepts no arguments.");
        return switch (tool) {
            case "get_connection_status" -> {
                JsonObject value = new JsonObject();
                value.addProperty("simulatorConnected", true);
                value.addProperty("robotControlAvailable", canMove() && !aiBusy);
                value.addProperty("stage", "simulation-control");
                value.addProperty("trajectory_control", true);
                yield value;
            }
            case "get_robot_state" -> robotState(joints, player);
            case "get_scene" -> scene(root);
            case "get_motion" -> McpRobotSnapshot.motion(player);
            case "get_geometry" -> McpRobotSnapshot.geometry(root, args);
            case "get_robot_snapshot" -> {
                JsonObject snapshot = robotState(joints, player);
                snapshot.add("scene", scene(root));
                snapshot.add("motion", McpRobotSnapshot.motion(player));
                snapshot.add("collisions", McpRobotSnapshot.collisions(collisions));
                yield snapshot;
            }
            case "get_task_status" -> taskStatus();
            case "stop_motion" -> {
                takeControl.run();
                player.bindJoints(joints);
                player.stop();
                finish("stopped", "Stop requested via MCP");
                yield taskStatus();
            }
            case "move_joints" -> args.has("waypoints")
                    ? executeTrajectory(args, root, joints, player, aiBusy, takeControl)
                    : move(args, root, joints, player, aiBusy, takeControl);
            case "execute_trajectory" -> executeTrajectory(args, root, joints, player, aiBusy, takeControl);
            case "move_tcp" -> moveTcp(args, root, joints, player, aiBusy, takeControl);
            case "pick_object_top_down" -> pick(args, root, joints, player, aiBusy, takeControl);
            case "pause_motion" -> {
                if (!taskState.equals("running") || player.getSequence() != sequence)
                    throw new IllegalArgumentException("No running MCP task.");
                takeControl.run();
                player.bindJoints(joints);
                player.pause();
                taskState = "paused";
                yield taskStatus();
            }
            case "resume_motion" -> {
                if (!canMove() || aiBusy || !taskState.equals("paused") || !currentTask(root, joints, player)
                        || player.isPlaying() || player.isLooping())
                    throw new IllegalArgumentException("No resumable MCP task in the current scene.");
                takeControl.run();
                player.play();
                taskState = "running";
                yield taskStatus();
            }
            case "capture_object" -> {
                requireIdle(player, aiBusy);
                requireStationary(joints);
                JsonObject capture = args.deepCopy();
                capture.addProperty("type", "capture");
                var event = McpJointTrajectory.event(capture, root, 0);
                takeControl.run();
                applyEvent(root, joints, event);
                JsonObject value = new JsonObject();
                value.addProperty("held_by", event.tool().getId());
                yield value;
            }
            case "release_object" -> {
                requireIdle(player, aiBusy);
                requireStationary(joints);
                if (args.size()!=1 || !args.has("object")) throw new IllegalArgumentException("Provide object ID only.");
                takeControl.run();
                JsonObject value = new JsonObject();
                value.addProperty("message", interactions.release(root, string(args, "object")));
                yield value;
            }
            default -> throw new IllegalArgumentException("Unknown tool");
        };
    }

    private boolean taskActive() { return taskState.equals("running") || taskState.equals("paused"); }

    private JsonObject executeTrajectory(JsonObject args, SceneNode root, List<Joint> joints,
                                         MotionPlayer player, boolean aiBusy, Runnable takeControl) {
        requireIdle(player, aiBusy);
        validateJointIds(joints);
        var plan = McpJointTrajectory.build(args, root, joints);
        takeControl.run();
        begin(root, joints, player, plan.sequence(), plan.events(), null, true);
        return taskStatus();
    }

    private boolean currentTask(SceneNode root, List<Joint> joints, MotionPlayer player) {
        return taskRoot == root && taskJoints.equals(joints) && player.getSequence() == sequence
                && player.getStopGeneration() == stopGeneration && player.getSeekGeneration() == seekGeneration;
    }

    private void requireIdle(MotionPlayer player, boolean aiBusy) {
        if (!canMove()) throw new IllegalArgumentException("Clear E-stop and disable Sim2Real sync before controlling the simulation.");
        if (aiBusy || player.isPlaying() || taskActive()) throw new IllegalArgumentException("Motion is busy. Stop the current task before starting another.");
    }

    private static void requireStationary(List<Joint> joints) {
        for (Joint joint : joints) {
            var motor = joint.getMotor();
            if (motor != null && (Math.abs(motor.getCurrentVelocity()) > .001f
                    || Math.abs(motor.getTargetAngleRadians() - joint.getCurrentAngleRadians()) > .001f))
                throw new IllegalArgumentException("Stop joint motion before attaching or releasing an object.");
        }
    }

    private static void validateJointIds(List<Joint> joints) {
        Set<String> ids = new HashSet<>();
        for (Joint joint : joints) {
            String id = joint.getId();
            if (id == null || id.isBlank() || !id.equals(id.trim()) || !ids.add(id.toLowerCase(Locale.ROOT)))
                throw new IllegalArgumentException("Robot joint IDs must be unique and nonblank.");
        }
    }

    private boolean canMove() {
        var bridge = BridgeManager.getInstance();
        return !bridge.getSafetyFilter().isEmergencyStopActive()
                && !bridge.isLiveSyncEnabled() && !bridge.isDigitalTwinEnabled();
    }

    private JsonObject move(JsonObject args, SceneNode root, List<Joint> joints, MotionPlayer player,
                            boolean aiBusy, Runnable takeControl) {
        if (args.size() != 1 || !args.has("targets_degrees") || !args.get("targets_degrees").isJsonObject())
            throw new IllegalArgumentException("Provide only targets_degrees: an object mapping exact joint IDs to degrees.");
        requireIdle(player, aiBusy);
        JsonObject targets = args.getAsJsonObject("targets_degrees");
        if (targets.isEmpty()) throw new IllegalArgumentException("At least one target is required.");
        validateJointIds(joints);
        Map<Joint, Float> validated = new LinkedHashMap<>();
        for (var entry : targets.entrySet()) {
            List<Joint> matches = joints.stream().filter(j -> entry.getKey().equals(j.getId())).toList();
            if (matches.size() != 1) throw new IllegalArgumentException("Unknown or ambiguous joint: " + entry.getKey());
            Joint joint = matches.get(0);
            JsonElement value = entry.getValue();
            if (!value.isJsonPrimitive() || !value.getAsJsonPrimitive().isNumber()) throw new IllegalArgumentException("Joint target must be a number.");
            double degrees = value.getAsDouble();
            double radians = Math.toRadians(degrees);
            if (!Double.isFinite(radians) || joint.getType() == JointType.FIXED || joint.getMotor() == null
                    || radians < joint.getMinLimit() - 1e-7 || radians > joint.getMaxLimit() + 1e-7)
                throw new IllegalArgumentException("Invalid target or limit exceeded for " + entry.getKey());
            validated.put(joint, (float) radians);
        }
        // Validate the entire request before changing any motors.
        takeControl.run();
        player.bindJoints(joints);
        player.pause();
        float[] start = JointMotion.angles(joints);
        validated.forEach((joint, radians) -> joint.getMotor().setTargetAngleRadians(radians));
        begin(root, joints, player, JointMotion.fromTargets("MCP move", joints, start), List.of(), null, false);
        return taskStatus();
    }

    private void begin(SceneNode root, List<Joint> joints, MotionPlayer player, MotionSequence motion,
                       List<McpJointTrajectory.Event> events, TopDownPickPlanner.Plan pick, boolean advancedTrajectory) {
        player.bindJoints(joints);
        sequence = motion;
        this.pick = pick;
        this.events = events;
        this.advancedTrajectory = advancedTrajectory;
        captured = false;
        nextEvent = 0;
        taskId = UUID.randomUUID().toString();
        taskRoot = root;
        taskJoints = List.copyOf(joints);
        detail = "";
        taskState = sequence == null ? "completed" : "running";
        if (sequence != null) {
            player.loadSequence(sequence);
            player.setPlaybackSpeed(1);
            player.play();
        }
        stopGeneration = player.getStopGeneration();
        seekGeneration = player.getSeekGeneration();
    }

    public void afterStep(SceneNode root, List<Joint> joints, MotionPlayer player, String blocked) {
        if (!taskActive()) return;
        if (taskRoot != root || !taskJoints.equals(joints)) {
            player.bindJoints(taskJoints);
            player.stop();
            player.bindJoints(joints);
            finish("interrupted", "Scene changed");
        } else if (!canMove()) {
            player.bindJoints(joints);
            player.stop();
            finish("stopped", "E-stop or hardware synchronization enabled");
        } else if (blocked != null) finish("blocked", blocked);
        else if (!currentTask(root, joints, player)) {
            if (player.getSequence() == sequence) player.stop();
            finish("interrupted", "Motion replaced, sought or stopped");
        } else if (taskState.equals("paused")) {
            if (player.isPlaying()) {
                player.pause();
                finish("interrupted", "Use Resume Motion to resume the MCP task.");
            }
        }
        else if (pick != null) {
            try {
                if (player.isLooping() || sequence.isLoop()) throw new IllegalArgumentException("Pick trajectory cannot loop.");
                if (!player.isPlaying() && player.getCurrentTime() < player.getDuration()) {
                    finish("interrupted", "Pick paused");
                    return;
                }
                detail = player.getCurrentTime() < pick.rotateTime() ? "approaching_above"
                        : player.getCurrentTime() < pick.descendTime() ? "orienting_downward"
                        : !captured ? "descending_to_capture" : "lifting";
                if (ObjectInteractionController.findNode(root, pick.object().getId()) != pick.object()
                        || ObjectInteractionController.findNode(root, pick.tool().getId()) != pick.tool())
                    throw new IllegalArgumentException("Pick object or tool removed.");
                if (!captured && !pick.objectPose().equals(pick.object().getWorldTransform(), .0001f))
                    throw new IllegalArgumentException("Object moved after pick planning.");
                if (!captured && player.getCurrentTime() >= pick.captureTime() - .00001f) {
                    if (TopDownPickPlanner.direction(pick.tool(), pick.axis()).dot(new Vector3f(0,-1,0)) < .999f)
                        throw new IllegalArgumentException("Tool is not vertical at capture.");
                    interactions.captureAtContact(root, joints, pick.object(), pick.tool(), pick.offset());
                    captured = true;
                }
                if (captured && !interactions.isHeld(pick.object())) throw new IllegalArgumentException("Object is no longer held.");
                if (!player.isPlaying()) {
                    if (player.getCurrentTime() < player.getDuration()) finish("interrupted", "Pick paused");
                    else if (!captured || ObjectInteractionController.objectCenter(pick.object()).y - pick.initialObjectY() < pick.liftDistance() - .005f)
                        throw new IllegalArgumentException("Pick ended without verified capture and requested lift.");
                    else finish("completed", "Capture and upward lift verified");
                }
            } catch (RuntimeException e) {
                player.bindJoints(joints);
                player.stop();
                finish("blocked", e.getMessage());
            }
        } else if (advancedTrajectory) {
            try {
                if (player.isLooping()) throw new IllegalArgumentException("MCP trajectories run once.");
                if (!player.isPlaying() && player.getCurrentTime() < player.getDuration()) {
                    finish("interrupted", "Motion paused outside MCP");
                    return;
                }
                if (nextEvent < events.size() && player.getCurrentTime() >= events.get(nextEvent).time() - .00001f) {
                    applyEvent(root, joints, events.get(nextEvent));
                    nextEvent++;
                }
                if (!player.isPlaying()) finish("completed", "Trajectory finished");
            } catch (RuntimeException e) {
                player.stop();
                finish("blocked", e.getMessage());
            }
        } else if (!player.isPlaying()) finish(player.getCurrentTime() >= player.getDuration() ? "completed" : "interrupted", "");
    }

    /** Never skip the capture event, even with a long render frame. */
    public float limitDelta(float delta, MotionPlayer player) {
        if (!taskState.equals("running")) return delta;
        if (advancedTrajectory || pick != null) {
            player.setPlaybackSpeed(1);
            float time = player.getCurrentTime();
            // Respect intermediate corners even when a render frame spans several waypoints.
            for (Keyframe frame : sequence.getKeyframes())
                if (frame.getTimeSeconds() > time) return Math.min(delta, frame.getTimeSeconds() - time);
        }
        return delta;
    }

    private void applyEvent(SceneNode root, List<Joint> joints, McpJointTrajectory.Event event) {
        if (ObjectInteractionController.findNode(root, event.object().getId()) != event.object()
                || !event.isRelease() && ObjectInteractionController.findNode(root, event.tool().getId()) != event.tool())
            throw new IllegalArgumentException("Trajectory event object or tool was removed or replaced.");
        if (event.isRelease()) interactions.release(root, event.object().getId());
        else interactions.captureAtContact(root, joints, event.object(), event.tool(), event.offset());
    }

    private JsonObject moveTcp(JsonObject args, SceneNode root, List<Joint> joints, MotionPlayer player,
                              boolean aiBusy, Runnable takeControl) {
        requireIdle(player, aiBusy);
        validateJointIds(joints);
        if (!Set.of("tool", "tool_offset", "tool_axis", "waypoints").containsAll(args.keySet())
                || !args.has("waypoints") || !args.get("waypoints").isJsonArray())
            throw new IllegalArgumentException("Provide tool, tool_offset and Cartesian waypoints.");
        JsonArray input = args.getAsJsonArray("waypoints");
        if (input.isEmpty() || input.size() > 32) throw new IllegalArgumentException("Provide 1 to 32 Cartesian waypoints.");
        Vector3f axis = args.has("tool_axis") ? unitVector(args.get("tool_axis")) : null;
        List<TopDownPickPlanner.TcpWaypoint> waypoints = new ArrayList<>();
        for (JsonElement item : input) {
            JsonObject waypoint = item.getAsJsonObject();
            if (!Set.of("position", "direction").containsAll(waypoint.keySet()) || !waypoint.has("position"))
                throw new IllegalArgumentException("A Cartesian waypoint accepts position and optional direction.");
            Vector3f direction = waypoint.has("direction") ? unitVector(waypoint.get("direction")) : null;
            if (direction != null && axis == null) throw new IllegalArgumentException("Specify tool_axis to request a direction.");
            waypoints.add(new TopDownPickPlanner.TcpWaypoint(vector(waypoint.get("position")), direction));
        }
        MotionSequence motion = new TopDownPickPlanner().planTcpPath(root, joints, tool(root, args),
                vector(args.get("tool_offset")), axis, waypoints);
        takeControl.run();
        begin(root, joints, player, motion, List.of(), null, true);
        return taskStatus();
    }

    private JsonObject pick(JsonObject args, SceneNode root, List<Joint> joints, MotionPlayer player, boolean aiBusy, Runnable takeControl) {
        Set<String> allowed = Set.of("object", "tool", "tool_offset", "tool_axis", "clearance", "lift_distance");
        if (!allowed.containsAll(args.keySet()) || !args.keySet().containsAll(Set.of("object", "tool", "tool_offset", "tool_axis")))
            throw new IllegalArgumentException("Provide object, tool, calibrated tool_offset and local tool_axis; optional clearance and lift_distance.");
        requireIdle(player, aiBusy);
        validateJointIds(joints);
        float clearance = args.has("clearance") ? number(args.get("clearance")) : .15f;
        float lift = args.has("lift_distance") ? number(args.get("lift_distance")) : .1f;
        TopDownPickPlanner.Plan planned = new TopDownPickPlanner().plan(root, joints,
                string(args, "object"), string(args, "tool"),
                vector(args.get("tool_offset")), vector(args.get("tool_axis")), clearance, lift);
        takeControl.run();
        begin(root, joints, player, planned.sequence(), List.of(), planned, false);
        detail = "Single trajectory: above object, orient downward, descend, capture, lift";
        return taskStatus();
    }

    static float number(JsonElement value) {
        if (value == null || !value.isJsonPrimitive() || !value.getAsJsonPrimitive().isNumber() || !Float.isFinite(value.getAsFloat()))
            throw new IllegalArgumentException("Expected a finite number.");
        return value.getAsFloat();
    }

    static Vector3f vector(JsonElement value) {
        if (value == null || !value.isJsonArray() || value.getAsJsonArray().size()!=3) throw new IllegalArgumentException("Expected a three-number vector.");
        JsonArray a=value.getAsJsonArray();
        return new Vector3f(number(a.get(0)),number(a.get(1)),number(a.get(2)));
    }

    private static Vector3f unitVector(JsonElement value) {
        Vector3f vector = vector(value);
        if (!Float.isFinite(vector.lengthSquared()) || vector.lengthSquared() < .000001f)
            throw new IllegalArgumentException("Expected a finite, nonzero direction.");
        return vector.normalize();
    }

    static String string(JsonObject args, String name) {
        JsonElement value = args.get(name);
        if (value == null || !value.isJsonPrimitive() || !value.getAsJsonPrimitive().isString() || value.getAsString().isBlank())
            throw new IllegalArgumentException("Expected a nonblank string for " + name);
        return value.getAsString();
    }

    static SceneNode tool(SceneNode root, JsonObject args) {
        SceneNode tool = ObjectInteractionController.findNode(root, string(args, "tool"));
        if (tool == null || tool == root || !tool.isRobotPart()) throw new IllegalArgumentException("Invalid robot tool.");
        return tool;
    }

    public void disable(MotionPlayer player, List<Joint> joints) {
        if (taskActive()) {
            player.bindJoints(joints);
            player.stop();
            finish("stopped", "MCP disabled");
        }
    }

    private void finish(String state, String reason) {
        if (taskActive()) { taskState = state; detail = reason; }
    }

    private JsonObject taskStatus() {
        JsonObject value = new JsonObject();
        value.addProperty("task_id", taskId);
        value.addProperty("state", taskState);
        value.addProperty("detail", detail);
        if (pick != null) value.addProperty("capture_verified", captured);
        if (!events.isEmpty()) {
            value.addProperty("events_completed", nextEvent);
            value.addProperty("events_total", events.size());
        }
        return value;
    }

    private JsonObject robotState(List<Joint> joints, MotionPlayer player) {
        JsonObject value = new JsonObject();
        JsonArray array = new JsonArray();
        for (Joint joint : joints) {
            JsonObject item = new JsonObject();
            item.addProperty("id", joint.getId());
            item.addProperty("type", joint.getType().name());
            item.addProperty("angle_degrees", Math.toDegrees(joint.getCurrentAngleRadians()));
            item.addProperty("min_degrees", Math.toDegrees(joint.getMinLimit()));
            item.addProperty("max_degrees", Math.toDegrees(joint.getMaxLimit()));
            item.addProperty("parent", joint.getParentNode() == null ? null : joint.getParentNode().getId());
            item.addProperty("child", joint.getChildNode() == null ? null : joint.getChildNode().getId());
            item.add("axis_local", coordinates(joint.getAxis()));
            item.add("zero_rotation_xyzw", McpRobotSnapshot.rotation(joint.getOriginRotation()));
            item.addProperty("motor_type", joint.getMotorType());
            item.addProperty("pin", joint.getPin());
            item.addProperty("inverted", joint.isInverted());
            item.addProperty("zero_offset_degrees", joint.getZeroOffsetDeg());
            var motor = joint.getMotor();
            if (motor != null) {
                item.addProperty("target_degrees", Math.toDegrees(motor.getTargetAngleRadians()));
                item.addProperty("velocity_degrees_per_second", Math.toDegrees(motor.getCurrentVelocity()));
                item.addProperty("speed_limit_degrees_per_second", Math.toDegrees(motor.getEffectiveCruiseSpeed()));
                item.addProperty("acceleration_degrees_per_second_squared", Math.toDegrees(motor.getAcceleration()));
                item.addProperty("motor_status", motor.getMotionStatus());
            }
            array.add(item);
        }
        value.add("joints", array);
        value.addProperty("playing", player.isPlaying());
        value.addProperty("e_stop", BridgeManager.getInstance().getSafetyFilter().isEmergencyStopActive());
        value.addProperty("motion_allowed", canMove());
        value.add("last_task", taskStatus());
        return value;
    }

    private JsonObject scene(SceneNode root) {
        JsonObject value = new JsonObject();
        JsonArray nodes = new JsonArray();
        if (root != null) visit(root, nodes);
        value.add("nodes", nodes);
        value.addProperty("position_units", "scene units");
        value.add("world_up", coordinates(new Vector3f(0, 1, 0)));
        value.addProperty("matrix_order", "column_major");
        return value;
    }

    private void visit(SceneNode node, JsonArray nodes) {
        JsonObject item = new JsonObject();
        item.addProperty("id", node.getId());
        item.addProperty("parent", node.getParent() == null ? null : node.getParent().getId());
        Vector3f position = node.getWorldTransform().getTranslation(new Vector3f());
        JsonArray xyz = new JsonArray();
        xyz.add(position.x); xyz.add(position.y); xyz.add(position.z);
        item.add("world_position", xyz);
        item.addProperty("robot_part", node.isRobotPart());
        item.add("local_position", coordinates(node.getLocalPosition()));
        item.add("local_scale", coordinates(node.getLocalScale()));
        item.add("local_rotation_xyzw", McpRobotSnapshot.rotation(node.getLocalRotation()));
        item.add("world_transform", McpRobotSnapshot.matrix(node.getWorldTransform()));
        item.add("meshes", McpRobotSnapshot.meshes(node));
        if (node.getCollisionShape() != null) item.add("collision_shape", McpRobotSnapshot.shape(node.getCollisionShape()));
        if (interactions.isHeld(node) && node.getParent() != null) item.addProperty("held_by", node.getParent().getId());
        var bounds = ObjectInteractionController.objectBounds(node);
        item.add("bounds_min", coordinates(bounds.getMin()));
        item.add("bounds_max", coordinates(bounds.getMax()));
        if (node.isRobotPart()) {
            item.add("derived_tcp_offset", coordinates(ObjectInteractionController.captureOffset(node, new Vector3f())));
            item.add("local_x_world", coordinates(TopDownPickPlanner.direction(node, new Vector3f(1,0,0))));
            item.add("local_y_world", coordinates(TopDownPickPlanner.direction(node, new Vector3f(0,1,0))));
            item.add("local_z_world", coordinates(TopDownPickPlanner.direction(node, new Vector3f(0,0,1))));
        }
        nodes.add(item);
        for (SceneNode child : node.getChildren()) visit(child, nodes);
    }

    static JsonArray coordinates(Vector3f point) {
        JsonArray value = new JsonArray();
        value.add(point.x); value.add(point.y); value.add(point.z);
        return value;
    }
}
