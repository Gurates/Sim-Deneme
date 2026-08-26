package rsim2.ai;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import rsim2.scene.Joint;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class AiActionExecutor {

    private static final Pattern JSON_BLOCK_PATTERN = Pattern.compile("```(?:json|action)?\\s*(\\{[\\s\\S]*?\\})\\s*```", Pattern.CASE_INSENSITIVE);

    public static class ExecutionResult {
        public String cleanedMessage;
        public List<String> appliedActions = new ArrayList<>();
        public boolean hasActions = false;
    }

    public static ExecutionResult processAndExecute(String rawResponse, List<Joint> joints) {
        ExecutionResult result = new ExecutionResult();
        result.cleanedMessage = rawResponse;

        if (rawResponse == null || rawResponse.trim().isEmpty() || joints == null || joints.isEmpty()) {
            return result;
        }

        Matcher matcher = JSON_BLOCK_PATTERN.matcher(rawResponse);
        String jsonContent = null;
        String matchedBlock = null;

        if (matcher.find()) {
            matchedBlock = matcher.group(0);
            jsonContent = matcher.group(1);
        } else {
            int startIdx = rawResponse.indexOf("{\"actions\"");
            if (startIdx == -1) {
                startIdx = rawResponse.indexOf("{\n  \"actions\"");
            }
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
                if (obj.has("actions") && obj.get("actions").isJsonArray()) {
                    JsonArray actions = obj.getAsJsonArray("actions");
                    for (JsonElement elem : actions) {
                        if (elem.isJsonObject()) {
                            JsonObject act = elem.getAsJsonObject();
                            executeSingleAction(act, joints, result);
                        }
                    }
                }

                if (!result.appliedActions.isEmpty()) {
                    result.hasActions = true;
                    String clean = rawResponse.replace(matchedBlock, "").trim();
                    StringBuilder sb = new StringBuilder(clean);
                    if (sb.length() > 0) {
                        sb.append("\n\n");
                    }
                    sb.append("\n[Robot Movement]:\n");
                    for (String msg : result.appliedActions) {
                        sb.append("  - ").append(msg).append("\n");
                    }
                    result.cleanedMessage = sb.toString().trim();
                }
            } catch (Exception e) {
                System.err.println("Failed to parse AI action JSON: " + e.getMessage());
            }
        }

        return result;
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
                result.appliedActions.add("⚠️ Joint not found: " + jointId);
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
            result.appliedActions.add(String.format("%s: %.1f° (Clamped to %.1f° due to joint limits)", j.getId(), deg, clampedDeg));
        } else {
            result.appliedActions.add(String.format("%s: %.1f°", j.getId(), clampedDeg));
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
