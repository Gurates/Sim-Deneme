package rsim2;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import rsim2.graphics.Mesh;
import rsim2.io.MjcfLoader;
import rsim2.io.RobotJsonIO;
import rsim2.scene.SceneNode;
import java.nio.file.Files;
import java.nio.file.Path;
import static org.junit.jupiter.api.Assertions.*;

public class MjcfVisualTransformTest {
    @TempDir Path directory;

    @Test public void eachGeomUsesItsOwnFrameAndSurvivesSaveLoad() throws Exception {
        Files.writeString(directory.resolve("part.obj"), "v 1 0 0\nv 0 1 0\nv 0 0 1\nvn 1 1 0\nf 1//1 2//1 3//1\n");
        Path model = directory.resolve("model.xml");
        Files.writeString(model, """
            <mujoco model="visual_frames"><compiler angle="radian"/>
              <asset><mesh name="part" file="part.obj" scale="2 1 1"/></asset>
              <worldbody><body name="base">
                <geom type="mesh" mesh="part" pos="1 2 3" quat="1 0 0 1"/>
                <geom type="mesh" mesh="part" pos="4 5 6"/>
                <body name="fixed_camera" pos="0 0 2"><geom type="mesh" mesh="part"/></body>
                <body name="arm" pos="0 0 1"><joint name="hinge"/><geom type="mesh" mesh="part"/></body>
              </body></worldbody>
            </mujoco>
            """);
        var imported = MjcfLoader.load(model.toString(), false);
        SceneNode base = imported.rootNode;
        assertVector(new Vector3f(1, 4, 3), vertex(base.getMeshes().get(0)));
        assertVector(new Vector3f(6, 5, 6), vertex(base.getMeshes().get(1)));
        assertVector(new Vector3f(0, 0, 1), imported.linksById.get("arm").getWorldTransform().getTranslation(new Vector3f()));
        Vector3f normal = new Vector3f(base.getMeshes().get(0).getNormals());
        assertVector(new Vector3f(-1, .5f, 0).normalize(), normal);
        Path saved = directory.resolve("robot.json");
        RobotJsonIO.save(RobotJsonIO.fromSceneGraph(base, imported.joints, imported.robotName), saved.toString());
        var restored = RobotJsonIO.toSceneGraph(RobotJsonIO.load(saved.toString()), directory.toString());
        for (var entry : imported.linksById.entrySet()) {
            SceneNode copy = restored.nodesById.get(entry.getKey());
            assertEquals(entry.getValue().getMeshes().size(), copy.getMeshes().size());
            for (int i = 0; i < copy.getMeshes().size(); i++) {
                assertArrayEquals(entry.getValue().getMeshes().get(i).getVertices(), copy.getMeshes().get(i).getVertices(), 1e-5f);
            }
            assertTrue(entry.getValue().getWorldTransform().equals(copy.getWorldTransform(), 1e-5f), entry.getKey());
        }
    }

    @Test public void cameraPrimitivesRenderAndPersistWhileCollisionGroupStaysHidden() throws Exception {
        Path model = directory.resolve("camera.xml");
        Files.writeString(model, """
            <mujoco><worldbody><body name="base"><body name="camera" pos="0 0 2">
              <geom type="box" size=".019 .019 .0005" pos="1 2 3" group="2"/>
              <geom type="cylinder" size=".007 .00475" pos="4 5 6" group="1"/>
              <geom type="box" size="1 1 1" group="3"/>
            </body></body></worldbody></mujoco>
            """);
        var imported = MjcfLoader.load(model.toString(), false);
        SceneNode camera = imported.linksById.get("camera");
        assertEquals(2, camera.getMeshes().size());
        assertEquals(1, imported.joints.size());
        assertEquals(rsim2.scene.JointType.FIXED, imported.joints.get(0).getType());
        assertVector(new Vector3f(.981f, 1.981f, 2.9995f), camera.getMeshes().get(0).getMinBound());
        Path saved = directory.resolve("camera.json");
        RobotJsonIO.save(RobotJsonIO.fromSceneGraph(imported.rootNode, imported.joints, "camera"), saved.toString());
        var copy = RobotJsonIO.toSceneGraph(RobotJsonIO.load(saved.toString()), directory.toString()).nodesById.get("camera");
        assertEquals("base", copy.getParent().getId());
        assertTrue(camera.getWorldTransform().equals(copy.getWorldTransform(), 1e-5f));
        for (int i = 0; i < 2; i++) {
            assertArrayEquals(camera.getMeshes().get(i).getVertices(), copy.getMeshes().get(i).getVertices(), 1e-5f);
        }
    }

    private static Vector3f vertex(Mesh mesh) {
        float[] vertices = mesh.getVertices();
        return new Vector3f(vertices[0], vertices[1], vertices[2]);
    }
    private static void assertVector(Vector3f expected, Vector3f actual) {
        assertEquals(expected.x, actual.x, 1e-5f);
        assertEquals(expected.y, actual.y, 1e-5f);
        assertEquals(expected.z, actual.z, 1e-5f);
    }
}
