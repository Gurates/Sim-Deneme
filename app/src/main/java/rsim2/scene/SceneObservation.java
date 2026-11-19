package rsim2.scene;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import org.joml.Quaternionf;
import org.joml.Vector3f;
import rsim2.collision.CollisionFilter;
import rsim2.collision.CollisionResult;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public final class SceneObservation {
    private SceneObservation() {}

    public static JsonArray xyz(Vector3f v) {
        JsonArray values = new JsonArray();
        values.add(v.x);
        values.add(v.y);
        values.add(v.z);
        return values;
    }

    public static String collisionStatus(CollisionFilter filter, CollisionResult result) {
        if (filter != null && (!filter.isEnabled()
                || (!filter.isGroundCollisionEnabled() && !filter.isSelfCollisionEnabled()))) return "disabled";
        return result == null ? "unknown" : result.status();
    }

    public static JsonObject describe(SceneNode root, List<Joint> joints, SceneNode selected,
                                      ObjectInteractionController interactions, CollisionFilter filter, CollisionResult collisions) {
        JsonObject summary = new JsonObject();
        summary.addProperty("coordinate_frame", "world");
        summary.addProperty("position_units", "scene units");
        summary.addProperty("meters_per_scene_unit", root == null ? null : root.getMetersPerSceneUnit());
        summary.add("world_up", xyz(new Vector3f(0, 1, 0)));
        summary.addProperty("ground_y", filter == null ? CollisionFilter.DEFAULT_GROUND_HEIGHT : filter.getGroundHeight());
        summary.addProperty("collision_status", collisionStatus(filter, collisions));
        if (filter != null) {
            summary.addProperty("ground_check_enabled", filter.isEnabled() && filter.isGroundCollisionEnabled());
            summary.addProperty("self_check_enabled", filter.isEnabled() && filter.isSelfCollisionEnabled());
        }
        SceneNode ancestor = selected;
        while (ancestor != null && ancestor != root) ancestor = ancestor.getParent();
        if (ancestor == null) selected = null;
        summary.addProperty("selected_node_id", selected == null ? null : selected.getId());

        Set<SceneNode> movingLinks = new HashSet<>();
        if (joints != null) {
            for (Joint joint : joints) {
                if (joint.getType() != JointType.FIXED) movingLinks.add(joint.getChildNode());
            }
        }
        List<SceneNode> configured = new ArrayList<>();
        List<SceneNode> leaves = new ArrayList<>();
        collectTools(root, movingLinks, false, configured, leaves);
        boolean calibrated = !configured.isEmpty();
        List<SceneNode> tools = calibrated ? configured : leaves;
        SceneNode active = tools.size() == 1 ? tools.get(0) : null;
        if (tools.contains(selected)) active = selected;
        summary.addProperty("active_tool_id", active == null ? null : active.getId());
        JsonArray candidates = new JsonArray();
        Vector3f activeTcp = null;
        for (SceneNode tool : tools) {
            Vector3f offset = ObjectInteractionController.workingOffset(tool);
            Vector3f tcp = tool.getWorldTransform().transformPosition(new Vector3f(offset));
            JsonObject item = new JsonObject();
            item.addProperty("id", tool.getId());
            item.addProperty("source", calibrated ? "configured" : "geometry_hint");
            item.add("offset_local", xyz(offset));
            item.add("world_position", xyz(tcp));
            candidates.add(item);
            if (tool == active) activeTcp = tcp;
        }
        summary.add("tools", candidates);

        JsonArray objects = new JsonArray();
        for (SceneNode object : ObjectInteractionController.objects(root)) {
            var bounds = ObjectInteractionController.objectBounds(object);
            var world = object.getWorldTransform();
            JsonObject item = new JsonObject();
            item.addProperty("id", object.getId());
            item.add("world_position", xyz(world.getTranslation(new Vector3f())));
            item.add("center_world", xyz(bounds.getCenter()));
            item.add("bounds_min", xyz(bounds.getMin()));
            item.add("bounds_max", xyz(bounds.getMax()));
            item.add("world_scale", xyz(world.getScale(new Vector3f())));
            Quaternionf q = world.getUnnormalizedRotation(new Quaternionf());
            JsonArray rotation = new JsonArray();
            rotation.add(q.x);
            rotation.add(q.y);
            rotation.add(q.z);
            rotation.add(q.w);
            item.add("rotation_xyzw", rotation);
            item.addProperty("state", interactions == null ? "unknown" : interactions.state(object));
            if (activeTcp != null) {
                Vector3f nearest = new Vector3f(activeTcp).max(bounds.getMin()).min(bounds.getMax());
                item.addProperty("tcp_to_bounds_distance", activeTcp.distance(nearest));
            }
            objects.add(item);
        }
        summary.add("objects", objects);
        return summary;
    }

    private static void collectTools(SceneNode node, Set<SceneNode> movingLinks, boolean movable,
                                     List<SceneNode> configured, List<SceneNode> leaves) {
        if (node == null || !node.isRobotPart()) return;
        movable |= movingLinks.contains(node);
        if (node.getTcpOffset() != null) configured.add(node);
        boolean leaf = true;
        for (SceneNode child : node.getChildren()) {
            if (!child.isRobotPart()) continue;
            leaf = false;
            collectTools(child, movingLinks, movable, configured, leaves);
        }
        if (leaf && movable) leaves.add(node);
    }
}
