package rsim2.mcp;

import com.google.gson.*;
import org.joml.Matrix4d;
import org.joml.Vector3d;
import rsim2.collision.*;
import rsim2.mcp.McpSurfaceGeometry.Part;
import rsim2.scene.ObjectInteractionController;
import rsim2.scene.SceneNode;

import java.util.*;
import static rsim2.mcp.McpSurfaceGeometry.xyz;

final class McpGraspState {
    private McpGraspState() {}

    private record Jaw(SceneNode node, Integer mesh, Set<Integer> triangles, List<Integer> collisionPath) {}
    private record Pair(Part jaw, Part object, SurfaceProximity.Measurement result) {}

    static JsonObject read(JsonObject args, SceneNode root) {
        Set<String> required = Set.of("object","fixed_jaw","moving_jaw");
        if (!args.keySet().containsAll(required) || !Set.of("object","fixed_jaw","moving_jaw","grasp_section").containsAll(args.keySet()))
            throw new IllegalArgumentException("Provide object, fixed_jaw, moving_jaw and optionally grasp_section.");
        SceneNode target = ObjectInteractionController.findNode(root, McpRobotController.string(args,"object"));
        if (target == null || !ObjectInteractionController.objects(root).contains(target))
            throw new IllegalArgumentException("Expected an environment object ID.");
        Jaw fixed = jaw(root,args.get("fixed_jaw")), moving = jaw(root,args.get("moving_jaw"));
        if (overlap(fixed,moving)) throw new IllegalArgumentException("Select two distinct jaw surfaces.");
        GraspAperture.Section section = args.has("grasp_section") ? section(args.get("grasp_section"), fixed.node) : null;
        List<Part> fixedParts = jawParts(fixed), movingParts = jawParts(moving);
        List<Part> objects = new ArrayList<>();
        collectObject(target, objects);
        if (objects.isEmpty()) throw new IllegalArgumentException("Object has no measurable surface: " + target.getId());

        JsonObject result = new JsonObject();
        result.addProperty("object",target.getId());
        result.addProperty("coordinate_frame","world");
        result.addProperty("position_units","scene units");
        result.addProperty("measurement_basis","geometric surfaces at the current pose");
        result.addProperty("meters_per_scene_unit",root.getMetersPerSceneUnit());
        result.add("fixed_jaw", measure(fixed,fixedParts,objects));
        result.add("moving_jaw", measure(moving,movingParts,objects));
        result.add("aperture", aperture(fixedParts, movingParts, objects, section));
        return result;
    }

    private static GraspAperture.Section section(JsonElement value, SceneNode fixed) {
        if (!value.isJsonObject() || !value.getAsJsonObject().keySet().equals(Set.of("point_local","axis_local")))
            throw new IllegalArgumentException("grasp_section requires point_local and axis_local in the fixed jaw frame.");
        JsonObject args = value.getAsJsonObject();
        Matrix4d world = new Matrix4d(fixed.getWorldTransform());
        Vector3d point = new Vector3d(McpRobotController.vector(args.get("point_local")));
        Vector3d axis = new Vector3d(McpRobotController.vector(args.get("axis_local")));
        return new GraspAperture.Section(world.transformPosition(point), world.transformDirection(axis));
    }

    private static JsonObject aperture(List<Part> fixed, List<Part> moving, List<Part> objects, GraspAperture.Section section) {
        var measurement = GraspAperture.measure(fixed.stream().map(Part::surface).toList(),
                moving.stream().map(Part::surface).toList(), objects.stream().map(Part::surface).toList(),
                section, McpBridge::ensureRequestActive);
        JsonObject data = new JsonObject();
        data.addProperty("section_source", section == null ? "inferred_from_jaw_surfaces" : "fixed_jaw_local_section");
        if (measurement.isEmpty()) {
            data.addProperty("state", "unavailable");
            data.addProperty("detail", "Could not resolve a section through both jaws. Select inner pad surfaces or specify grasp_section.");
            return data;
        }
        var m = measurement.get();
        double gap = m.signedOpening();
        data.addProperty("state", gap < 0 ? "overlapping" : gap == 0 ? "closed" : "open");
        data.addProperty("jaw_opening", Math.max(0, gap));
        data.add("axis_fixed_to_moving_world", xyz(m.axis()));
        data.add("fixed_inner_point_world", xyz(m.fixedPoint()));
        data.add("moving_inner_point_world", xyz(m.movingPoint()));
        if (gap >= 0) data.add("grasp_center_world", xyz(m.center()));
        data.addProperty("object_width_along_axis", m.objectWidth());
        if (gap >= 0) data.addProperty("object_center_offset_along_axis", m.objectCenterOffset());
        data.addProperty("width_fits", m.widthFits());
        data.addProperty("object_within_opening_along_axis", m.withinOpening());
        return data;
    }

    private static Jaw jaw(SceneNode root, JsonElement selector) {
        JsonObject args = new JsonObject();
        if (selector.isJsonPrimitive()) args.add("node",selector);
        else if (selector.isJsonObject()) args = selector.getAsJsonObject();
        else throw new IllegalArgumentException("Expected a jaw node ID or a surface selector.");
        if (!Set.of("node","mesh_index","triangles","collision_path").containsAll(args.keySet()))
            throw new IllegalArgumentException("Unknown jaw selector field.");
        SceneNode node = ObjectInteractionController.findNode(root,McpRobotController.string(args,"node"));
        if (node == null || node == root) throw new IllegalArgumentException("Expected a robot jaw node.");
        for (SceneNode n=node; n!=null; n=n.getParent())
            if (!n.isRobotPart()) throw new IllegalArgumentException("Jaw must belong to the robot.");
        Integer mesh = args.has("mesh_index") ? index(args.get("mesh_index")) : null;
        Set<Integer> faces = Set.of();
        if (args.has("triangles")) {
            if (mesh == null) throw new IllegalArgumentException("triangles requires mesh_index.");
            faces = Set.copyOf(indices(args.get("triangles")));
            if (faces.isEmpty()) throw new IllegalArgumentException("Select at least one triangle.");
        }
        List<Integer> path = args.has("collision_path") ? indices(args.get("collision_path")) : null;
        if (mesh != null && path != null) throw new IllegalArgumentException("Choose a mesh or collision shape.");
        return new Jaw(node,mesh,faces,path);
    }

    private static int index(JsonElement value) {
        if (value.isJsonPrimitive() && value.getAsJsonPrimitive().isNumber()) {
            try {
                int i = value.getAsBigDecimal().intValueExact();
                if (i >= 0) return i;
            } catch (ArithmeticException | NumberFormatException ignored) {}
        }
        throw new IllegalArgumentException("Surface indices must be nonnegative integers.");
    }

    private static List<Integer> indices(JsonElement value) {
        if (!value.isJsonArray()) throw new IllegalArgumentException("Expected an array of surface indices.");
        List<Integer> indices = new ArrayList<>();
        for (JsonElement element : value.getAsJsonArray()) indices.add(index(element));
        return List.copyOf(indices);
    }

    private static boolean overlap(Jaw a, Jaw b) {
        if (a.node != b.node) return false;
        if (a.collisionPath != null && b.collisionPath != null) {
            int common = Math.min(a.collisionPath.size(), b.collisionPath.size());
            return a.collisionPath.subList(0,common).equals(b.collisionPath.subList(0,common));
        }
        if (a.mesh == null || b.mesh == null) return true;
        return a.mesh.equals(b.mesh) && (a.triangles.isEmpty() || b.triangles.isEmpty()
                || !Collections.disjoint(a.triangles,b.triangles));
    }

    private static void collectObject(SceneNode node, List<Part> parts) {
        collect(new Jaw(node,null,Set.of(),null),parts);
        for (SceneNode child : node.getChildren()) collectObject(child,parts);
    }

    private static List<Part> jawParts(Jaw jaw) {
        List<Part> jaws = new ArrayList<>();
        collect(jaw,jaws);
        if (jaws.isEmpty()) throw new IllegalArgumentException("Jaw has no measurable surface: " + jaw.node.getId());
        return jaws;
    }

    private static JsonObject measure(Jaw jaw, List<Part> jaws, List<Part> objects) {
        Pair nearest = null, penetration = null;
        for (Part part : jaws) for (Part object : objects) {
            var result = SurfaceProximity.measure(part.surface(),object.surface(),McpBridge::ensureRequestActive);
            Pair pair = new Pair(part,object,result);
            if (nearest == null || result.surfaceDistance() < nearest.result.surfaceDistance()) nearest = pair;
            if (result.state().equals("penetrating") && (penetration == null
                    || result.surfaceDistance() < penetration.result.surfaceDistance())) penetration = pair;
        }
        JsonObject data = new JsonObject();
        data.addProperty("node",jaw.node.getId());
        String state = penetration == null ? nearest.result.state() : "penetrating";
        data.addProperty("state",state);
        data.addProperty("gap",state.equals("penetrating") ? 0 : nearest.result.surfaceDistance());
        data.addProperty("surface_distance",nearest.result.surfaceDistance());
        data.addProperty("contact_tolerance",nearest.result.tolerance());
        data.add("jaw_surface", witness(nearest.jaw,nearest.result.jawSurface(),nearest.result.jawPoint()));
        data.add("object_surface", witness(nearest.object,nearest.result.objectSurface(),nearest.result.objectPoint()));
        if (penetration != null && penetration != nearest) {
            data.add("penetrating_jaw_surface", witness(penetration.jaw,penetration.result.jawSurface(),penetration.result.jawPoint()));
            data.add("penetrating_object_surface", witness(penetration.object,penetration.result.objectSurface(),penetration.result.objectPoint()));
        }
        return data;
    }

    private static JsonObject witness(Part part, SurfaceProximity.Triangle face, Vector3d point) {
        JsonObject value = new JsonObject();
        value.addProperty("node",part.node());
        value.addProperty("geometry",part.geometry());
        if (part.mesh() != null) value.addProperty("mesh_index",part.mesh());
        if (part.path() != null) {
            JsonArray indices = new JsonArray();
            part.path().forEach(i -> indices.add(i));
            value.add("collision_path",indices);
        }
        value.addProperty("triangle_index",face.index());
        if ("collision_box".equals(part.geometry())) value.addProperty("face_local", BOX_FACES[face.index()/2]);
        value.addProperty("closed_surface",part.surface().closed());
        value.add("point_world",xyz(point));
        value.add("winding_normal_world",xyz(face.normal()));
        return value;
    }

    private static final String[] BOX_FACES = {"-X","+X","-Y","+Y","-Z","+Z"};

    private static void collect(Jaw selection, List<Part> parts) {
        McpSurfaceGeometry.collect(selection.node, selection.mesh, selection.triangles, selection.collisionPath,
                new Matrix4d(selection.node.getWorldTransform()), parts);
    }
}
