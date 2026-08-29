package rsim2.ai;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import rsim2.motion.Keyframe;
import rsim2.motion.MotionPlayer;
import rsim2.motion.MotionSequence;
import rsim2.scene.Joint;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class AiActionExecutor {

    private static final Pattern JSON_BLOCK_PATTERN = Pattern.compile("```(?:json|action)?\\s*(\\{[\\s\\S]*?\\})\\s*```", Pattern.CASE_INSENSITIVE);

    public static class ExecutionResult {
        public String cleanedMessage;
        public List<String> appliedActions = new ArrayList<>();
        public MotionSequence generatedSequence = null;
        public boolean hasActions = false;
        public boolean hasAnimation = false;
    }

    public static ExecutionResult processAndExecute(String rawResponse, List<Joint> joints, MotionPlayer motionPlayer) {
        ExecutionResult result = new ExecutionResult();
        result.cleanedMessage = rawResponse;

        if (rawResponse == null || rawResponse.trim().isEmpty()) {
            return result;
        }

        Matcher matcher = JSON_BLOCK_PATTERN.matcher(rawResponse);
        String jsonContent = null;
        String matchedBlock = null;

        if (matcher.find()) {
            matchedBlock = matcher.group(0);
            jsonContent = matcher.group(1);
        } else {
            int startIdx = rawResponse.indexOf("{\"animation\"");
            if (startIdx == -1) startIdx = rawResponse.indexOf("{\n  \"animation\"");
            if (startIdx == -1) startIdx = rawResponse.indexOf("{\"trajectory\"");
            if (startIdx == -1) startIdx = rawResponse.indexOf("{\"actions\"");
            if (startIdx == -1) startIdx = rawResponse.indexOf("{\n  \"actions\"");

            if (startIdx != -1) {
                int endIdx = rawResponse.lastIndexOf("}");
                if (endIdx > startIdx) {
                    jsonContent = rawResponse.substring(startIdx, endIdx + 1);
                    matchedBlock = jsonContent;
                }
            }
        }

        if (jsonContent != null) {
            try {
                JsonObject obj = JsonParser.parseString(jsonContent).getAsJsonObject();

                JsonObject animObj = null;
                if (obj.has("animation") && obj.get("animation").isJsonObject()) {
                    animObj = obj.getAsJsonObject("animation");
                } else if (obj.has("trajectory") && obj.get("trajectory").isJsonObject()) {
                    animObj = obj.getAsJsonObject("trajectory");
                } else if (obj.has("keyframes") && obj.get("keyframes").isJsonArray()) {
                    animObj = obj;
                }

                if (animObj != null) {
                    MotionSequence seq = parseAnimationSequence(animObj, joints);
                    if (seq != null && !seq.getKeyframes().isEmpty()) {
                        result.generatedSequence = seq;
                        result.hasAnimation = true;

                        if (motionPlayer != null) {
                            motionPlayer.loadSequence(seq);
                            motionPlayer.play();
                        }

                        String clean = (matchedBlock != null) ? rawResponse.replace(matchedBlock, "").trim() : rawResponse;
                        StringBuilder sb = new StringBuilder(clean);
                        if (sb.length() > 0) {
                            sb.append("\n\n");
                        }
                        sb.append(String.format("[Motion Loaded]: \"%s\"\n", seq.getName()));
                        sb.append(String.format("  - Duration: %.2f seconds\n", seq.getDurationSeconds()));
                        sb.append(String.format("  - Keyframes: %d\n", seq.getKeyframes().size()));
                        sb.append(String.format("  - Loop: %s\n", seq.isLoop() ? "Enabled" : "Disabled"));
                        sb.append("  - Playback loaded. You can control it using the [Play / Pause] buttons on the top bar.");
                        result.cleanedMessage = sb.toString().trim();
                        return result;
                    }
                }

                if (obj.has("actions") && obj.get("actions").isJsonArray() && joints != null) {
                    JsonArray actions = obj.getAsJsonArray("actions");
                    for (JsonElement elem : actions) {
                        if (elem.isJsonObject()) {
                            JsonObject act = elem.getAsJsonObject();
                            executeSingleAction(act, joints, result);
                        }
                    }

                    if (!result.appliedActions.isEmpty()) {
                        result.hasActions = true;
                        String clean = (matchedBlock != null) ? rawResponse.replace(matchedBlock, "").trim() : rawResponse;
                        StringBuilder sb = new StringBuilder(clean);
                        if (sb.length() > 0) {
                            sb.append("\n\n");
                        }
                        sb.append("[Robot Movement]:\n");
                        for (String msg : result.appliedActions) {
                            sb.append("  - ").append(msg).append("\n");
                        }
                        result.cleanedMessage = sb.toString().trim();
                    }
                }

            } catch (Exception e) {
                System.err.println("Failed to parse AI action/animation JSON: " + e.getMessage());
            }
        }

        return result;
    }

    private static MotionSequence parseAnimationSequence(JsonObject animObj, List<Joint> joints) {
        String name = animObj.has("name") ? animObj.get("name").getAsString() : "AI Robot Motion";
        float duration = animObj.has("duration") ? animObj.get("duration").getAsFloat() : 1.0f;
        boolean loop = animObj.has("loop") && animObj.get("loop").getAsBoolean();

        MotionSequence sequence = new MotionSequence(name, duration, loop);

        if (animObj.has("keyframes") && animObj.get("keyframes").isJsonArray()) {
            JsonArray kfArray = animObj.getAsJsonArray("keyframes");
            for (JsonElement el : kfArray) {
                if (el.isJsonObject()) {
                    JsonObject kfObj = el.getAsJsonObject();
                    float time = kfObj.has("time") ? kfObj.get("time").getAsFloat() : (kfObj.has("t") ? kfObj.get("t").getAsFloat() : 0.0f);

                    Keyframe keyframe = new Keyframe(time);

                    JsonObject jointsMap = null;
                    if (kfObj.has("joints") && kfObj.get("joints").isJsonObject()) {
                        jointsMap = kfObj.getAsJsonObject("joints");
                    } else if (kfObj.has("angles") && kfObj.get("angles").isJsonObject()) {
                        jointsMap = kfObj.getAsJsonObject("angles");
                    }

                    if (jointsMap != null) {
                        for (Map.Entry<String, JsonElement> entry : jointsMap.entrySet()) {
                            try {
                                String jointId = entry.getKey();
                                float angleDeg = entry.getValue().getAsFloat();

                                if ("ALL".equalsIgnoreCase(jointId) && joints != null) {
                                    for (Joint j : joints) {
                                        keyframe.addJointAngle(j.getId(), angleDeg);
                                    }
                                } else {
                                    keyframe.addJointAngle(jointId, angleDeg);
                                }
                            } catch (Exception ignored) {
                            }
                        }
                    }

                    sequence.addKeyframe(keyframe);
                }
            }
        }

        sequence.sortKeyframes();
        return sequence;
    }

    private static void executeSingleAction(JsonObject act, List<Joint> joints, ExecutionResult result) {
        String jointId = null;
        if (act.has("joint")) jointId = act.get("joint").getAsString();
        else if (act.has("joint_id")) jointId = act.get("joint_id").getAsString();
        else if (act.has("id")) jointId = act.get("id").getAsString();

        if (jointId == null) {
            return;
        }

        Double targetDeg = null;
        if (act.has("target_deg")) targetDeg = act.get("target_deg").getAsDouble();
        else if (act.has("angle_deg")) targetDeg = act.get("angle_deg").getAsDouble();
        else if (act.has("angle")) targetDeg = act.get("angle").getAsDouble();
        else if (act.has("target")) targetDeg = act.get("target").getAsDouble();
        else if (act.has("value")) targetDeg = act.get("value").getAsDouble();

        if (targetDeg == null) {
            return;
        }

        float deg = targetDeg.floatValue();
        float targetRad = (float) Math.toRadians(deg);

        if ("ALL".equalsIgnoreCase(jointId) || "*".equals(jointId)) {
            for (Joint j : joints) {
                applyJointTarget(j, targetRad, deg, result);
            }
        } else {
            Joint targetJoint = findJoint(joints, jointId);
            if (targetJoint != null) {
                applyJointTarget(targetJoint, targetRad, deg, result);
            } else {
                result.appliedActions.add("[Warning] Joint not found: " + jointId);
            }
        }
    }

    private static void applyJointTarget(Joint j, float targetRad, float deg, ExecutionResult result) {
        float clampedRad = Math.max(j.getMinLimit(), Math.min(j.getMaxLimit(), targetRad));
        float clampedDeg = (float) Math.toDegrees(clampedRad);

        if (j.getMotor() != null) {
            j.getMotor().setTargetAngleRadians(clampedRad);
        } else {
            j.setAngle(clampedRad);
        }

        if (Math.abs(clampedDeg - deg) > 0.01f) {
            result.appliedActions.add(String.format("%s: %.1f deg (Clamped by limit to %.1f deg)", j.getId(), deg, clampedDeg));
        } else {
            result.appliedActions.add(String.format("%s: %.1f deg", j.getId(), clampedDeg));
        }
    }

    private static Joint findJoint(List<Joint> joints, String jointId) {
        for (Joint j : joints) {
            if (j.getId().equalsIgnoreCase(jointId)) {
                return j;
            }
        }
        for (Joint j : joints) {
            if (j.getId().toLowerCase().contains(jointId.toLowerCase()) ||
                jointId.toLowerCase().contains(j.getId().toLowerCase())) {
                return j;
            }
        }
        return null;
    }
}
