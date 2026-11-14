package rsim2.mcp;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import org.joml.Matrix4f;
import org.joml.Quaternionf;
import rsim2.collision.CollisionResult;
import rsim2.collision.CollisionShape;
import rsim2.collision.CompoundShape;
import rsim2.collision.CapsuleShape;
import rsim2.collision.CylinderShape;
import rsim2.collision.SphereShape;
import rsim2.graphics.Mesh;
import rsim2.motion.MotionPlayer;
import rsim2.scene.ObjectInteractionController;
import rsim2.scene.SceneNode;

import java.util.Set;

final class McpRobotSnapshot {
    private static final Gson GSON = new Gson();

    static JsonObject motion(MotionPlayer player) {
        JsonObject result = new JsonObject();
        result.addProperty("playing", player.isPlaying());
        result.addProperty("looping", player.isLooping());
        result.addProperty("elapsed_seconds", player.getCurrentTime());
        result.addProperty("duration_seconds", player.getDuration());
        result.addProperty("playback_speed", player.getPlaybackSpeed());
        if (player.hasSequence()) result.add("sequence", GSON.toJsonTree(player.getSequence()));
        return result;
    }

    static JsonObject collisions(CollisionResult state) {
        JsonObject result = new JsonObject();
        JsonArray contacts = new JsonArray();
        result.addProperty("available", state != null);
        if (state != null) {
            for (var contact : state.getContacts()) {
                JsonObject value = new JsonObject();
                value.addProperty("node_a", contact.getNodeA() == null ? null : contact.getNodeA().getId());
                value.addProperty("node_b", contact.getNodeB() == null ? null : contact.getNodeB().getId());
                value.addProperty("type", contact.getType().name());
                value.addProperty("penetration", contact.getPenetrationDepth());
                if (contact.getContactPoint() != null) value.add("point", McpRobotController.coordinates(contact.getContactPoint()));
                contacts.add(value);
            }
        }
        result.add("contacts", contacts);
        return result;
    }

    static JsonArray matrix(Matrix4f matrix) {
        return GSON.toJsonTree(matrix.get(new float[16])).getAsJsonArray();
    }

    static JsonArray rotation(Quaternionf rotation) {
        return GSON.toJsonTree(new float[]{rotation.x, rotation.y, rotation.z, rotation.w}).getAsJsonArray();
    }

    static JsonObject shape(CollisionShape shape) {
        JsonObject value = new JsonObject();
        value.addProperty("type", shape.getClass().getSimpleName());
        value.add("bounds_min", McpRobotController.coordinates(shape.getLocalAABB().getMin()));
        value.add("bounds_max", McpRobotController.coordinates(shape.getLocalAABB().getMax()));
        if (shape instanceof CompoundShape compound) {
            JsonArray children = new JsonArray();
            for (var child : compound.getChildren()) {
                JsonObject entry = shape(child.getShape());
                entry.add("transform", matrix(child.getLocalTransform()));
                children.add(entry);
            }
            value.add("children", children);
        } else if (shape instanceof SphereShape sphere) {
            value.addProperty("radius", sphere.getRadius());
            value.add("center", McpRobotController.coordinates(sphere.getLocalCenter()));
        } else if (shape instanceof CapsuleShape capsule) {
            value.addProperty("radius", capsule.getRadius());
            value.add("start", McpRobotController.coordinates(capsule.getLocalP0()));
            value.add("end", McpRobotController.coordinates(capsule.getLocalP1()));
        } else if (shape instanceof CylinderShape cylinder) {
            value.addProperty("radius", cylinder.getRadius());
            value.addProperty("height", cylinder.getHeight());
        }
        return value;
    }

    static JsonArray meshes(SceneNode node) {
        JsonArray values = new JsonArray();
        for (Mesh mesh : node.getMeshes()) {
            JsonObject value = new JsonObject();
            value.addProperty("mesh_index", values.size());
            value.addProperty("vertices_length", mesh.getVertices().length);
            value.addProperty("normals_length", mesh.getNormals() == null ? 0 : mesh.getNormals().length);
            value.addProperty("indices_length", mesh.getIndices() == null ? 0 : mesh.getIndices().length);
            value.add("source_transform", matrix(mesh.getSourceTransform()));
            values.add(value);
        }
        return values;
    }

    static JsonObject geometry(SceneNode root, JsonObject args) {
        if (!Set.of("node", "mesh_index", "field", "offset", "limit").containsAll(args.keySet()))
            throw new IllegalArgumentException("Unknown geometry argument.");
        SceneNode node = ObjectInteractionController.findNode(root, McpRobotController.string(args, "node"));
        int index = integer(args, "mesh_index", 0, Integer.MAX_VALUE);
        if (node == null || index >= node.getMeshes().size()) throw new IllegalArgumentException("Unknown node or mesh index.");
        Mesh mesh = node.getMeshes().get(index);
        String field = McpRobotController.string(args, "field");
        int offset = integer(args, "offset", 0, Integer.MAX_VALUE);
        int limit = integer(args, "limit", 1024, 4096);
        if (limit == 0) throw new IllegalArgumentException("Limit must be positive.");
        float[] floats = switch (field) {
            case "vertices" -> mesh.getVertices();
            case "normals" -> mesh.getNormals();
            case "indices" -> null;
            default -> throw new IllegalArgumentException("Expected vertices, normals or indices.");
        };
        int[] indices = field.equals("indices") ? mesh.getIndices() : null;
        int length = floats != null ? floats.length : indices != null ? indices.length : 0;
        int end = (int) Math.min(length, (long) offset + limit);
        JsonArray values = new JsonArray();
        for (int i = offset; i < end; i++) {
            if (indices == null) values.add(floats[i]); else values.add(indices[i]);
        }
        JsonObject result = new JsonObject();
        result.addProperty("field", field);
        result.addProperty("total", length);
        result.addProperty("offset", offset);
        result.addProperty("next_offset", end < length ? end : null);
        result.add("values", values);
        return result;
    }

    private static int integer(JsonObject args, String field, int fallback, int max) {
        if (!args.has(field)) return fallback;
        var value = args.get(field);
        if (!value.isJsonPrimitive() || !value.getAsJsonPrimitive().isNumber())
            throw new IllegalArgumentException("Expected integer " + field);
        try {
            int number = value.getAsBigDecimal().intValueExact();
            if (number >= 0 && number <= max) return number;
        } catch (ArithmeticException | NumberFormatException ignored) {}
        throw new IllegalArgumentException("Out-of-range integer " + field);
    }
}
