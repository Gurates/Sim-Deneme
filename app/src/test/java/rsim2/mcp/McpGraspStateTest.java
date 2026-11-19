package rsim2.mcp;

import com.google.gson.*;
import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.junit.jupiter.api.Test;
import rsim2.collision.*;
import rsim2.graphics.Mesh;
import rsim2.io.PrimitiveMeshFactory;
import rsim2.motion.MotionPlayer;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class McpGraspStateTest {
    private final SceneNode root = new SceneNode("world");
    private final SceneNode fixed = box(root, "fixed", -2);
    private final SceneNode moving = box(root, "moving", 3);
    private final SceneNode object = box(root, "box", 0);
    private final MotionPlayer player = new MotionPlayer();
    private final McpRobotController robot = new McpRobotController();
    private final List<Joint> joints = List.of(new Joint("hinge",root,moving,new Vector3f(0,0,1)));

    McpGraspStateTest() { object.setRobotPart(false); }

    private static SceneNode box(SceneNode parent, String id, float x) {
        SceneNode node = new SceneNode(id);
        node.getLocalPosition().x = x;
        node.setCollisionShape(new BoxShape(-.5f,-.5f,-.5f,.5f,.5f,.5f));
        parent.addChild(node);
        return node;
    }

    private JsonObject arguments() {
        return JsonParser.parseString("""
                {"object":"box","fixed_jaw":"fixed","moving_jaw":"moving"}
                """).getAsJsonObject();
    }

    private JsonObject read(JsonObject args) {
        return robot.execute("get_grasp_state",args,root,joints,player,true,() -> fail("Measurement took control"));
    }

    private JsonObject fixed() { return read(arguments()).getAsJsonObject("fixed_jaw"); }

    private JsonObject aperture() { return read(arguments()).getAsJsonObject("aperture"); }

    private static void assertPoint(JsonObject data, String field, double x, double y, double z) {
        var vector = data.getAsJsonArray(field);
        assertArrayEquals(new double[]{x, y, z}, new double[]{vector.get(0).getAsDouble(),
                vector.get(1).getAsDouble(), vector.get(2).getAsDouble()}, 1e-6);
    }

    @Test void measuresInnerOpeningAndDistinguishesWidthFromAlignment() {
        var data = aperture();
        assertEquals("open", data.get("state").getAsString());
        assertEquals(4, data.get("jaw_opening").getAsDouble(), 1e-8);
        assertPoint(data, "fixed_inner_point_world", -1.5, 0, 0);
        assertPoint(data, "moving_inner_point_world", 2.5, 0, 0);
        assertPoint(data, "grasp_center_world", .5, 0, 0);
        assertEquals(1, data.get("object_width_along_axis").getAsDouble(), 1e-8);
        assertEquals(-.5, data.get("object_center_offset_along_axis").getAsDouble(), 1e-8);
        assertTrue(data.get("object_within_opening_along_axis").getAsBoolean());
        moving.getLocalPosition().x = 0;
        data = aperture();
        assertEquals(1, data.get("jaw_opening").getAsDouble(), 1e-8);
        assertPoint(data, "grasp_center_world", -1, 0, 0);
        assertTrue(data.get("width_fits").getAsBoolean());
        assertFalse(data.get("object_within_opening_along_axis").getAsBoolean());
        box(object, "handle", 2);
        data = aperture();
        assertEquals(3, data.get("object_width_along_axis").getAsDouble(), 1e-8);
        assertFalse(data.get("width_fits").getAsBoolean());
        assertPoint(data, "grasp_center_world", -1, 0, 0);
    }

    @Test void measuresStaggeredParallelPadsPerpendicularToTheirFacesInWorldSpace() {
        moving.getLocalPosition().y = .5f;
        root.getLocalPosition().set(1, 2, 3);
        root.getLocalRotation().rotateZ((float)Math.PI / 2);
        root.getLocalScale().set(2, 1, 1);
        var data = aperture();
        assertEquals(8, data.get("jaw_opening").getAsDouble(), 1e-5);
        assertPoint(data, "axis_fixed_to_moving_world", 0, 1, 0);
        assertPoint(data, "grasp_center_world", .75, 3, 3);
        assertEquals(2, data.get("object_width_along_axis").getAsDouble(), 1e-6);
    }

    @Test void measuresHingedJawAtSpecifiedDepthAndKeepsContactsWhenSectionMisses() {
        moving.getLocalRotation().rotateZ((float)Math.PI / 4);
        var args = arguments();
        var section = JsonParser.parseString("""
                {"point_local":[0,0.25,0],"axis_local":[4,0,0]}
                """).getAsJsonObject();
        args.add("grasp_section", section);
        var data = read(args).getAsJsonObject("aperture");
        assertEquals("fixed_jaw_local_section", data.get("section_source").getAsString());
        double movingX = 3 - Math.sqrt(.5) + .25;
        assertEquals(movingX + 1.5, data.get("jaw_opening").getAsDouble(), 1e-6);
        assertPoint(data, "grasp_center_world", (movingX - 1.5) / 2, .25, 0);
        section.add("axis_local", JsonParser.parseString("[-4,0,0]"));
        root.getLocalRotation().rotateZ((float)Math.PI / 2);
        var rotated = read(args).getAsJsonObject("aperture");
        assertEquals(movingX + 1.5, rotated.get("jaw_opening").getAsDouble(), 1e-6);
        assertPoint(rotated, "grasp_center_world", -.25, (movingX - 1.5) / 2, 0);
        assertPoint(rotated, "axis_fixed_to_moving_world", 0, 1, 0);
        section.add("point_local", JsonParser.parseString("[0,2,0]"));
        var report = read(args);
        data = report.getAsJsonObject("aperture");
        assertEquals("unavailable", data.get("state").getAsString());
        assertFalse(data.has("grasp_center_world"));
        assertTrue(report.getAsJsonObject("fixed_jaw").has("gap"));
        section.add("axis_local", JsonParser.parseString("[0,0,0]"));
        assertThrows(IllegalArgumentException.class, () -> read(args));
    }

    @Test void closedAndOverlappingJawsHaveNoPositiveOpening() {
        moving.getLocalPosition().x = -1;
        var data = aperture();
        assertEquals("closed", data.get("state").getAsString());
        assertEquals(0, data.get("jaw_opening").getAsDouble());
        assertPoint(data, "grasp_center_world", -1.5, 0, 0);
        moving.getLocalPosition().x = -1.25f;
        data = aperture();
        assertEquals("overlapping", data.get("state").getAsString());
        assertEquals(0, data.get("jaw_opening").getAsDouble());
        assertFalse(data.has("grasp_center_world"));
        assertFalse(data.get("width_fits").getAsBoolean());
    }

    @Test void coincidentFlatPadsStillDefineAClosedSectionAndDisjointPadsReportUnavailable() {
        fixed.addMesh(PrimitiveMeshFactory.box(.5f,.5f,.5f));
        moving.addMesh(PrimitiveMeshFactory.box(.5f,.5f,.5f));
        moving.getLocalPosition().x = -1;
        var args = arguments();
        args.add("fixed_jaw", JsonParser.parseString("""
                {"node":"fixed","mesh_index":0,"triangles":[10,11]}"""));
        args.add("moving_jaw", JsonParser.parseString("""
                {"node":"moving","mesh_index":0,"triangles":[8,9]}"""));
        var data = read(args).getAsJsonObject("aperture");
        assertEquals("closed", data.get("state").getAsString());
        assertPoint(data, "grasp_center_world", -1.5, 0, 0);
        moving.getLocalPosition().set(3, 2, 0);
        data = read(args).getAsJsonObject("aperture");
        assertEquals("unavailable", data.get("state").getAsString());
        assertFalse(data.has("jaw_opening"));
    }

    @Test void reportsIndependentWitnessesWhilePlaying() {
        var move = JsonParser.parseString("{\"targets_degrees\":{\"hinge\":25}}").getAsJsonObject();
        robot.execute("move_joints", move, root, joints, player, false, () -> {});
        var sequence = player.getSequence();
        float target = joints.get(0).getMotor().getTargetAngleRadians();
        JsonObject state = read(arguments());
        var left = state.getAsJsonObject("fixed_jaw");
        var right = state.getAsJsonObject("moving_jaw");
        assertEquals("separated",left.get("state").getAsString());
        assertEquals(1,left.get("gap").getAsDouble(),1e-8);
        assertEquals(2,right.get("gap").getAsDouble(),1e-8);
        assertEquals("+X",left.getAsJsonObject("jaw_surface").get("face_local").getAsString());
        assertEquals("-X",left.getAsJsonObject("object_surface").get("face_local").getAsString());
        assertEquals(-1.5,left.getAsJsonObject("jaw_surface").getAsJsonArray("point_world").get(0).getAsDouble(),1e-8);
        assertEquals(-.5,left.getAsJsonObject("object_surface").getAsJsonArray("point_world").get(0).getAsDouble(),1e-8);
        assertTrue(player.isPlaying());
        assertSame(sequence,player.getSequence());
        assertEquals(target,joints.get(0).getMotor().getTargetAngleRadians());
        assertEquals(root,object.getParent());
    }

    @Test void separatesFaceContactVolumeOverlapAndContainment() {
        fixed.getLocalPosition().x = -1;
        assertEquals("contact",fixed().get("state").getAsString());
        fixed.getLocalPosition().x = -.8f;
        assertEquals("penetrating",fixed().get("state").getAsString());
        fixed.getLocalPosition().zero();
        assertEquals("penetrating",fixed().get("state").getAsString());
        fixed.getLocalScale().set(.2f);
        JsonObject contained = fixed();
        assertEquals("penetrating",contained.get("state").getAsString());
        assertEquals(0,contained.get("gap").getAsDouble());
        assertEquals(.4,contained.get("surface_distance").getAsDouble(),1e-6);
    }

    @Test void measuresSelectedInnerPadsFromOneMesh() {
        fixed.addMesh(PrimitiveMeshFactory.box(.5f,.5f,.5f));
        var args = arguments();
        args.add("fixed_jaw",JsonParser.parseString("{\"node\":\"fixed\",\"mesh_index\":0,\"triangles\":[10,11]}"));
        args.add("moving_jaw",JsonParser.parseString("{\"node\":\"fixed\",\"mesh_index\":0,\"triangles\":[8,9]}"));
        fixed.getLocalScale().set(2,1,1);
        fixed.getLocalRotation().rotateZ((float)Math.PI);
        JsonObject state = read(args);
        assertEquals(2, state.getAsJsonObject("aperture").get("jaw_opening").getAsDouble(), 1e-6);
        assertPoint(state.getAsJsonObject("aperture"), "grasp_center_world", -2, 0, 0);
        assertEquals(2.5,state.getAsJsonObject("fixed_jaw").get("gap").getAsDouble(),1e-6);
        assertEquals(.5,state.getAsJsonObject("moving_jaw").get("gap").getAsDouble(),1e-6);
        assertTrue(state.getAsJsonObject("fixed_jaw").getAsJsonObject("jaw_surface").get("triangle_index").getAsInt() >= 10);
        args.add("moving_jaw",args.get("fixed_jaw"));
        assertThrows(IllegalArgumentException.class,()->read(args));
    }

    @Test void detectsIntersectionThroughMiddleOfTriangle() {
        fixed.getLocalPosition().zero();
        fixed.addMesh(new Mesh(new float[]{-2,0,0,2,0,0,0,3,0},null,new int[]{0,1,2}));
        JsonObject state=fixed();
        assertEquals("penetrating",state.get("state").getAsString());
        assertEquals(0,state.get("surface_distance").getAsDouble(),1e-8);
        assertEquals("mesh",state.getAsJsonObject("jaw_surface").get("geometry").getAsString());
    }

    @Test void resolvesCompoundChildAndEnvironmentDescendants() {
        var shape = new CompoundShape();
        shape.addShape(new BoxShape(-.1f,-.1f,-.1f,.1f,.1f,.1f),new Matrix4f().translation(1,0,0));
        fixed.setCollisionShape(shape);
        object.setCollisionShape(null);
        var child=box(object,"part",.4f);
        var args=arguments();
        args.add("fixed_jaw",JsonParser.parseString("{\"node\":\"fixed\",\"collision_path\":[0]}"));
        JsonObject state=read(args).getAsJsonObject("fixed_jaw");
        assertEquals(.8,state.get("gap").getAsDouble(),1e-6);
        assertEquals(child.getId(),state.getAsJsonObject("object_surface").get("node").getAsString());
        args.addProperty("moving_jaw","part");
        assertThrows(IllegalArgumentException.class,()->read(args));
        args.addProperty("moving_jaw","moving");
        args.add("fixed_jaw",JsonParser.parseString("{\"node\":\"fixed\",\"mesh_index\":99}"));
        assertThrows(IllegalArgumentException.class,()->read(args));
    }

    @Test void preservesMcpEnvelopeAndReportsInvalidGeometry() {
        McpServer server = new McpServer((tool,args)->read(args));
        var request = JsonParser.parseString("""
                {"id":1,"jsonrpc":"2.0","method":"tools/call","params":{"name":"get_grasp_state"}}
                """).getAsJsonObject();
        request.getAsJsonObject("params").add("arguments",arguments());
        var reply = server.handle(request.toString()).getAsJsonObject("result");
        assertFalse(reply.get("isError").getAsBoolean());
        assertTrue(reply.getAsJsonObject("structuredContent").getAsJsonObject("aperture").has("grasp_center_world"));
        assertEquals(reply.get("structuredContent"), JsonParser.parseString(reply.getAsJsonArray("content").get(0)
                .getAsJsonObject().get("text").getAsString()));
        fixed.setCollisionShape(null);
        assertTrue(server.handle(request.toString()).getAsJsonObject("result").get("isError").getAsBoolean());
    }

    @Test void measuresCollisionOnlyConvexAndCurvedPartsAndLabelsTheirGeometry() {
        var cloud = new ConvexMeshShape(PrimitiveMeshFactory.box(.5f,.5f,.5f).getVertices());
        fixed.setCollisionShape(cloud);
        moving.setCollisionShape(new SphereShape(.5f));
        var result = read(arguments());
        var left = result.getAsJsonObject("fixed_jaw");
        var right = result.getAsJsonObject("moving_jaw");
        assertEquals(1, left.get("gap").getAsDouble(), 1e-6);
        assertEquals("collision_convex_hull", left.getAsJsonObject("jaw_surface").get("geometry").getAsString());
        assertTrue(left.getAsJsonObject("jaw_surface").get("closed_surface").getAsBoolean());
        assertEquals("tessellated_collision_sphere", right.getAsJsonObject("jaw_surface").get("geometry").getAsString());
        assertTrue(right.getAsJsonObject("jaw_surface").get("closed_surface").getAsBoolean());
        assertEquals(2, right.get("gap").getAsDouble(), 1e-6);
    }

    @Test void isolatedTriangleEdgesCanBeEachOthersNearestFeatures() {
        fixed.getLocalPosition().zero();
        fixed.addMesh(new Mesh(new float[]{-1,0,0,1,0,0,0,-1,0},null,new int[]{0,1,2}));
        object.addMesh(new Mesh(new float[]{0,1,1,0,1,-1,0,2,0},null,new int[]{0,1,2}));
        var result = fixed();
        assertEquals(1, result.get("gap").getAsDouble(), 1e-7);
        assertEquals(0, result.getAsJsonObject("jaw_surface").getAsJsonArray("point_world").get(0).getAsDouble(),1e-7);
        assertEquals(0, result.getAsJsonObject("object_surface").getAsJsonArray("point_world").get(2).getAsDouble(),1e-7);
    }
}
