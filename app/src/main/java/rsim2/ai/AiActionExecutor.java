package rsim2.ai;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import rsim2.motion.Keyframe;
import rsim2.motion.MotionPlayer;
import rsim2.motion.MotionSequence;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;
import rsim2.scene.ObjectInteractionController;
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class AiActionExecutor {
    public static boolean isMotionRequest(String prompt) {
        return prompt != null && Pattern.compile("(?iu)\\b(hareket|tut|yakala|kavra|bırak|birak|taşı|tasi|döndür|dondur|kaldır|kaldir|yaklaş|yaklas|move|lift|grasp|grab|pick|release|rotate|home)\\b").matcher(prompt).find();
    }

    private static final Pattern JSON_BLOCK_PATTERN = Pattern.compile("```(?:json|action)?\\s*(\\{[\\s\\S]*?\\})\\s*```", Pattern.CASE_INSENSITIVE);

    public static class ExecutionResult {
        public String cleanedMessage;
        public List<String> appliedActions = new ArrayList<>();
        public MotionSequence generatedSequence = null;
        public boolean hasActions = false;
        public boolean rejected;
        public JsonArray previewResults;
        public boolean hasAnimation = false;
    }

    public static ExecutionResult processAndExecute(String rawResponse, List<Joint> joints, MotionPlayer motionPlayer) {
        return processAndExecute(rawResponse, joints, motionPlayer, null, null);
    }

    public static ExecutionResult processAndExecute(String rawResponse, List<Joint> joints, MotionPlayer motionPlayer,
                                                   SceneNode root, ObjectInteractionController interactions) {
        return processAndExecute(rawResponse, joints, motionPlayer, root, interactions, null);
    }

    public static ExecutionResult processAndExecute(String rawResponse, List<Joint> joints, MotionPlayer motionPlayer,
                                                   SceneNode root, ObjectInteractionController interactions,
                                                   GraspLiftCoordinator task) {
        ExecutionResult result = new ExecutionResult();
        result.cleanedMessage = rawResponse;

        if (rawResponse == null || rawResponse.trim().isEmpty()) {
            return result;
        }

        Matcher matcher = JSON_BLOCK_PATTERN.matcher(rawResponse);
        String jsonContent = null;

        if (matcher.find()) {
            jsonContent = matcher.group(1);
            if (matcher.find()) {
                result.rejected = true;
                result.cleanedMessage = "[Command rejected] Submit one complete motion as a single JSON command.";
                return result;
            }
        } else {
            Matcher actionStart = Pattern.compile("\\{\\s*\"(?:pose|actions|animation|trajectory|keyframes|system|interaction|preview_approach)\"\\s*:").matcher(rawResponse);
            int startIdx = actionStart.find() ? actionStart.start() : -1;

            if (startIdx != -1) {
                int endIdx = rawResponse.lastIndexOf("}");
                if (endIdx > startIdx) {
                    jsonContent = rawResponse.substring(startIdx, endIdx + 1);
                }
            }
        }

        if (jsonContent != null) {
            try {
                JsonObject obj = JsonParser.parseString(jsonContent).getAsJsonObject();
                if (obj.has("interaction") && obj.size() != 1)
                    throw new IllegalArgumentException("Send object interaction as a standalone command.");
                if (obj.has("preview_approach")) {
                    result.previewResults = preview(obj, root, joints, interactions);
                    result.cleanedMessage = "[Approach preview] " + result.previewResults;
                    return result;
                }
                List<JointTarget> targets = List.of();
                MotionSequence animation = null;
                if (!obj.has("interaction")) {
                    boolean validKeys = obj.has("keyframes")
                            ? Set.of("keyframes", "name", "duration", "loop").containsAll(obj.keySet())
                            : obj.size() == 1 && Set.of("pose", "actions", "animation", "trajectory", "system").containsAll(obj.keySet());
                    if (!validKeys) throw new IllegalArgumentException("Provide one complete robot command.");
                    if (!obj.has("system") && (joints == null || joints.isEmpty()))
                        throw new IllegalArgumentException("No robot joints are loaded.");
                    if (obj.has("pose") && !Set.of("home", "zero", "rest").contains(obj.get("pose").getAsString().trim().toLowerCase(java.util.Locale.ROOT)))
                        throw new IllegalArgumentException("Unknown pose: " + obj.get("pose"));
                    if (obj.has("actions")) targets = parseTargets(obj.getAsJsonArray("actions"), joints);
                    if (obj.has("keyframes")) animation = parseAnimationSequence(obj, joints);
                    if (obj.has("animation")) animation = parseAnimationSequence(obj.getAsJsonObject("animation"), joints);
                    if (obj.has("trajectory")) animation = parseAnimationSequence(obj.getAsJsonObject("trajectory"), joints);
                    if (obj.has("system")) {
                        JsonElement stop = obj.getAsJsonObject("system").get("estop");
                        if (stop == null || !stop.isJsonPrimitive() || !stop.getAsJsonPrimitive().isBoolean())
                            throw new IllegalArgumentException("Expected a boolean estop.");
                    }
                }
                float[] startAngles = joints == null ? null : rsim2.motion.JointMotion.angles(joints);
                if (motionPlayer != null) motionPlayer.bindJoints(joints);
                boolean interactionFailed = false;

                if (obj.has("interaction")) {
                    try {
                        if (interactions == null || root == null) throw new IllegalArgumentException("Scene interaction is unavailable.");
                        if (rsim2.bridge.BridgeManager.getInstance().getSafetyFilter().isEmergencyStopActive()) {
                            throw new IllegalArgumentException("Emergency stop is active.");
                        }
                        JsonObject action = obj.getAsJsonObject("interaction");
                        String type = action.get("type").getAsString();
                        String objectId = action.get("object").getAsString();
                        String message;
                        if ("grasp_and_lift".equals(type) || "lift".equals(type)) {
                            if (task == null) throw new IllegalArgumentException("Grasp-and-lift task execution is unavailable.");
                            Vector3f offset = readOffset(action, new Vector3f());
                            float height = action.has("lift_distance") ? action.get("lift_distance").getAsFloat() : 0.05f;
                            String tool = action.has("tool") ? action.get("tool").getAsString() : null;
                            message = task.start(root, joints, interactions, motionPlayer, objectId, tool, offset, height,
                                    "grasp_and_lift".equals(type));
                            result.generatedSequence = motionPlayer.getSequence();
                            result.hasAnimation = true;
                        } else if ("grasp".equals(type) || "approach".equals(type)) {
                            if (task != null && task.isActive()) throw new IllegalArgumentException("Stop the active grasp-and-lift task before planning another interaction.");
                            if (motionPlayer != null && (motionPlayer.isPlaying() || motionPlayer.isReplayingEvents())) {
                                throw new IllegalArgumentException("Finish or stop the current motion before planning a new approach or grasp.");
                            }
                            SceneNode toolNode = ObjectInteractionController.findNode(root, action.get("tool").getAsString());
                            if (toolNode == null || !toolNode.isRobotPart()) throw new IllegalArgumentException("Invalid robot tool.");
                            Vector3f offset = readOffset(action, "approach".equals(type)
                                    ? ObjectInteractionController.workingOffset(toolNode) : new Vector3f());
                            message = "approach".equals(type)
                                    ? interactions.approach(root, joints, objectId, action.get("tool").getAsString(), offset)
                                    : interactions.grasp(root, joints, objectId, action.get("tool").getAsString(), offset);
                            if (motionPlayer != null) {
                                boolean grasp = "grasp".equals(type);
                                MotionSequence approach = grasp
                                        ? rsim2.motion.JointMotion.forInteraction(type + " " + objectId, joints, startAngles)
                                        : rsim2.motion.JointMotion.fromTargets(type + " " + objectId, joints, startAngles);
                                if (approach != null) {
                                    approach.setIncomplete(grasp);
                                    motionPlayer.loadSequence(approach);
                                    motionPlayer.play();
                                    if (grasp) interactions.bindPlayback(motionPlayer, approach, true);
                                    result.generatedSequence = approach;
                                    result.hasAnimation = true;
                                }
                            }
                        } else if ("release".equals(type)) {
                            message = interactions.release(root, objectId);
                            if (task != null) task.cancel("release requested");
                            if (motionPlayer != null) motionPlayer.cancelReplay();
                        } else throw new IllegalArgumentException("Unknown interaction type: " + type);
                        result.appliedActions.add("[Interaction] " + message);
                    } catch (RuntimeException e) {
                        interactionFailed = true;
                        result.appliedActions.add("[Interaction failed] " + e.getMessage());
                    }
                } else if (interactions != null && (obj.has("pose") || obj.has("actions") || obj.has("animation")
                        || obj.has("trajectory") || obj.has("keyframes") || obj.has("system"))) {
                    interactions.cancelPending();
                    if (task != null) task.cancel("another robot command was requested");
                    if (motionPlayer != null) motionPlayer.cancelReplay();
                }

                if (obj.has("pose") && joints != null) {
                    String pose = obj.get("pose").getAsString().trim().toLowerCase();
                    if (motionPlayer != null) motionPlayer.pause();
                    executePose(pose, joints, result);
                }

                if (obj.has("system") && obj.get("system").isJsonObject()) {
                    JsonObject sys = obj.getAsJsonObject("system");
                    if (sys.has("estop")) {
                        boolean estop = sys.get("estop").getAsBoolean();
                        if (estop) {
                            rsim2.bridge.BridgeManager.getInstance().triggerEmergencyStop();
                            result.appliedActions.add("[Safety] Hardware EMERGENCY STOP (E-STOP) triggered.");
                            result.hasActions = true;
                        } else {
                            rsim2.bridge.BridgeManager.getInstance().resetEmergencyStop();
                            result.appliedActions.add("[Safety] Hardware EMERGENCY STOP reset.");
                            result.hasActions = true;
                        }
                    }
                }

                if (animation != null) {
                    result.generatedSequence = animation;
                    result.hasAnimation = true;
                    if (motionPlayer != null) {
                        motionPlayer.loadSequence(animation);
                        motionPlayer.play();
                    }
                    result.appliedActions.add("Loaded " + animation.getName());
                }
                if (!targets.isEmpty()) {
                    if (motionPlayer != null) motionPlayer.pause();
                    for (JointTarget target : targets) {
                        applyJointTarget(target.joint, (float) Math.toRadians(target.degrees),
                                target.degrees, target.speed, target.acceleration, result);
                    }
                }

                if (!result.appliedActions.isEmpty()) {
                    if (!interactionFailed && !obj.has("interaction") && animation == null && joints != null
                            && motionPlayer != null && (obj.has("pose") || obj.has("actions"))) {
                        MotionSequence direct = rsim2.motion.JointMotion.fromTargets("AI Joint Motion", joints, startAngles);
                        if (direct != null) {
                            motionPlayer.loadSequence(direct);
                            motionPlayer.play();
                            result.generatedSequence = direct;
                            result.hasAnimation = true;
                        }
                    }
                    result.hasActions = !interactionFailed;
                    result.rejected = interactionFailed;
                    StringBuilder sb = new StringBuilder();
                    if (interactionFailed) sb.append("[Command rejected]\n");
                    else if (result.generatedSequence != null && motionPlayer != null) sb.append("[Motion started]\n");
                    else sb.append("[Command applied]\n");
                    for (String msg : result.appliedActions) {
                        sb.append("  • ").append(msg).append("\n");
                    }
                    result.cleanedMessage = sb.toString().trim();
                    return result;
                }

            } catch (Exception e) {
                result.rejected = true;
                result.cleanedMessage = "[Command rejected] " + e.getMessage();
                return result;
            }
        }

        result.cleanedMessage = rawResponse + "\n\n[No action applied] No executable robot command was found in this response.";
        return result;
    }

    private static JsonArray preview(JsonObject obj, SceneNode root, List<Joint> joints, ObjectInteractionController interactions) {
        if (obj.size() != 1 || root == null || interactions == null)
            throw new IllegalArgumentException("Preview requires a scene and a standalone command.");
        JsonObject query = obj.getAsJsonObject("preview_approach");
        SceneNode tool = ObjectInteractionController.findNode(root, query.get("tool").getAsString());
        if (tool == null || !tool.isRobotPart() || tool == root) throw new IllegalArgumentException("Unknown robot tool.");
        JsonArray objects = query.getAsJsonArray("objects");
        if (objects == null || objects.isEmpty() || objects.size() > 8)
            throw new IllegalArgumentException("Preview between one and eight object IDs at a time.");
        JsonArray observations = new JsonArray();
        Vector3f offset = readOffset(query, ObjectInteractionController.workingOffset(tool));
        for (JsonElement id : objects) {
            observations.add(interactions.previewApproach(root, joints, id.getAsString(), tool.getId(), offset));
        }
        return observations;
    }

    private static void executePose(String pose, List<Joint> joints, ExecutionResult result) {
        if ("home".equals(pose) || "zero".equals(pose)) {
            for (Joint j : joints) {
                float targetRad = 0.0f;
                float clampedRad = Math.max(j.getMinLimit(), Math.min(j.getMaxLimit(), targetRad));
                if (j.getMotor() != null) {
                    j.getMotor().setTargetAngleRadians(clampedRad);
                } else {
                    j.setAngle(clampedRad);
                }
            }
            result.appliedActions.add("[Pose] HOME joint targets set.");
            result.hasActions = true;
        } else if ("rest".equals(pose)) {
            for (Joint j : joints) {
                float targetRad = (j.getMinLimit() + j.getMaxLimit()) * 0.5f;
                if (j.getMotor() != null) {
                    j.getMotor().setTargetAngleRadians(targetRad);
                } else {
                    j.setAngle(targetRad);
                }
            }
            result.appliedActions.add("[Pose] REST joint targets set.");
            result.hasActions = true;
        } else {
            result.appliedActions.add("[Pose] Unknown pose requested: " + pose);
        }
    }

    private static MotionSequence parseAnimationSequence(JsonObject animObj, List<Joint> joints) {
        String name = animObj.has("name") ? animObj.get("name").getAsString() : "AI Robot Motion";
        float duration = animObj.has("duration") ? animObj.get("duration").getAsFloat() : 1.0f;
        boolean loop = animObj.has("loop") && animObj.get("loop").getAsBoolean();

        if (!Float.isFinite(duration) || duration <= 0) throw new IllegalArgumentException("Invalid animation duration.");
        MotionSequence sequence = new MotionSequence(name, duration, loop);
        JsonArray frames = animObj.getAsJsonArray("keyframes");
        if (frames == null || frames.isEmpty()) throw new IllegalArgumentException("Animation needs keyframes.");
        for (JsonElement el : frames) {
            JsonObject frame = el.getAsJsonObject();
            JsonElement timestamp = first(frame, "time", "t");
            float time = timestamp == null ? 0 : finite(timestamp, "keyframe time");
            if (time < 0 || time > duration) throw new IllegalArgumentException("Keyframe outside animation duration.");
            Keyframe key = new Keyframe(time);
            JsonElement values = first(frame, "joints", "angles");
            if (values == null || values.getAsJsonObject().isEmpty()) throw new IllegalArgumentException("Empty joint keyframe.");
            for (var entry : values.getAsJsonObject().entrySet()) {
                float degrees = finite(entry.getValue(), "joint angle");
                for (Joint joint : resolveJoints(joints, entry.getKey()))
                    key.addJointAngle(joint.getId(), degrees);
            }
            sequence.addKeyframe(key);
        }
        sequence.sortKeyframes();
        return sequence;
    }

    private static Vector3f readOffset(JsonObject action, Vector3f fallback) {
        if (!action.has("tool_offset")) return fallback;
        JsonArray offset = action.getAsJsonArray("tool_offset");
        if (offset.size() != 3) throw new IllegalArgumentException("tool_offset requires three coordinates.");
        return new Vector3f(finite(offset.get(0), "offset"), finite(offset.get(1), "offset"), finite(offset.get(2), "offset"));
    }

    private record JointTarget(Joint joint, float degrees, Float speed, Float acceleration) {}

    private static List<JointTarget> parseTargets(JsonArray actions, List<Joint> joints) {
        if (actions.isEmpty()) throw new IllegalArgumentException("Empty joint command.");
        List<JointTarget> targets = new ArrayList<>();
        for (JsonElement element : actions) {
            JsonObject action = element.getAsJsonObject();
            JsonElement id = first(action, "joint", "joint_id", "id");
            if (id == null) throw new IllegalArgumentException("Missing joint ID.");
            float degrees = finite(first(action, "target_deg", "angle_deg", "angle", "target", "value"), "target angle");
            JsonElement speed = first(action, "speed_ratio", "speed_pct", "speed");
            Float ratio = speed == null ? null : finite(speed, "speed");
            if (ratio != null) {
                if (!action.has("speed_ratio") && action.has("speed_pct")) ratio /= 100;
                if (ratio <= 0 || ratio > 1) throw new IllegalArgumentException("Speed ratio must be in (0, 1].");
            }
            JsonElement accel = first(action, "acceleration", "accel");
            Float acceleration = accel == null ? null : finite(accel, "acceleration");
            if (acceleration != null && acceleration <= 0) throw new IllegalArgumentException("Acceleration must be positive.");
            for (Joint joint : resolveJoints(joints, id.getAsString()))
                targets.add(new JointTarget(joint, degrees, ratio, acceleration));
        }
        return targets;
    }

    private static List<Joint> resolveJoints(List<Joint> joints, String id) {
        boolean all = "ALL".equalsIgnoreCase(id) || "*".equals(id);
        var matches = joints.stream().filter(j -> j.getType() != rsim2.scene.JointType.FIXED
                && (all || j.getId().equalsIgnoreCase(id))).toList();
        if (matches.isEmpty() || (!all && matches.size() != 1))
            throw new IllegalArgumentException("Unknown, fixed or ambiguous joint: " + id);
        return matches;
    }

    private static JsonElement first(JsonObject object, String... aliases) {
        for (String name : aliases) if (object.has(name)) return object.get(name);
        return null;
    }

    private static float finite(JsonElement value, String field) {
        if (value == null || !value.isJsonPrimitive() || !value.getAsJsonPrimitive().isNumber()
                || !Float.isFinite(value.getAsFloat())) throw new IllegalArgumentException("Invalid " + field);
        return value.getAsFloat();
    }

    private static void applyJointTarget(Joint j, float targetRad, float deg, Float speedRatio, Float acceleration, ExecutionResult result) {
        float clampedRad = Math.max(j.getMinLimit(), Math.min(j.getMaxLimit(), targetRad));
        float clampedDeg = (float) Math.toDegrees(clampedRad);

        if (j.getMotor() != null) {
            if (speedRatio != null && speedRatio > 0.0f) {
                j.getMotor().setTargetSpeedRatio(speedRatio);
            }
            if (acceleration != null && acceleration > 0.0f) {
                j.getMotor().setAcceleration(acceleration);
            }
            j.getMotor().setTargetAngleRadians(clampedRad);
        } else {
            j.setAngle(clampedRad);
        }

        StringBuilder sb = new StringBuilder(String.format("%s: %.1f°", j.getId(), clampedDeg));
        if (Math.abs(clampedDeg - deg) > 0.01f) {
            sb.append(String.format(" (clamped by limit from %.1f°)", deg));
        }
        if (speedRatio != null) {
            sb.append(String.format(" @ %.0f%% speed", speedRatio * 100.0f));
        }
        result.appliedActions.add(sb.toString());
    }

}
