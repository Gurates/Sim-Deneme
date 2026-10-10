package rsim2;

import org.junit.jupiter.api.Test;
import rsim2.collision.CollisionFilter;
import rsim2.data.RobotDefinitionDTO;
import rsim2.editor.EnvironmentModelImporter;
import rsim2.io.RobotJsonIO;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;
import rsim2.script.ModelImportScript;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class ModelImportScriptTest {

    public static void main(String[] args) {
        System.out.println("=== RUNNING MODEL IMPORT SCRIPT TESTS ===");
        ModelImportScriptTest test = new ModelImportScriptTest();

        try {
            test.testImportObjModel();
            System.out.println("[PASS] testImportObjModel");

            test.testImportStlModel();
            System.out.println("[PASS] testImportStlModel");

            test.testHierarchyPanelExclusion();
            System.out.println("[PASS] testHierarchyPanelExclusion");

            test.testRobotJsonExportExclusion();
            System.out.println("[PASS] testRobotJsonExportExclusion");

            test.testEnvironmentModelImporterAlias();
            System.out.println("[PASS] testEnvironmentModelImporterAlias");

            test.testCollisionGroundExclusion();
            System.out.println("[PASS] testCollisionGroundExclusion");

            test.testInvalidFilePath();
            System.out.println("[PASS] testInvalidFilePath");

            System.out.println("\nALL 7 MODEL IMPORT SCRIPT TESTS PASSED SUCCESSFULLY (100%)!");
        } catch (Throwable t) {
            System.err.println("\nTEST FAILED: " + t.getMessage());
            t.printStackTrace();
            System.exit(1);
        }
    }

    @Test
    public void testImportObjModel() {
        File objFile = new File("app/src/main/resources/models/test.obj");
        if (!objFile.exists()) {
            objFile = new File("src/main/resources/models/test.obj");
        }
        assertTrue(objFile.exists(), "Test OBJ file must exist on disk");

        SceneNode root = new SceneNode("world");
        SceneNode importedNode = ModelImportScript.importModel(root, objFile.getAbsolutePath());

        assertNotNull(importedNode, "Imported scene node should not be null");
        assertFalse(importedNode.getMeshes().isEmpty(), "Imported node must contain mesh data");
        assertFalse(importedNode.isRobotPart(), "Imported model must NOT be marked as a robot part");
        assertTrue(importedNode.isExcludedFromHierarchy(), "Imported model must be excluded from HierarchyPanel");
        assertEquals(root, importedNode.getParent(), "Imported node parent should be the world root");
        assertTrue(root.getChildren().contains(importedNode), "World root must contain imported node");
        assertTrue(importedNode.getLocalPosition().y >= 0.0f, "Imported model should be placed on or above ground plane");
    }

    @Test
    public void testImportStlModel() throws IOException {
        // Create a temporary ASCII STL file
        File tempStl = File.createTempFile("test_model", ".stl");
        tempStl.deleteOnExit();

        try (FileWriter writer = new FileWriter(tempStl)) {
            writer.write("solid test_cube\n");
            writer.write("  facet normal 0 0 1\n");
            writer.write("    outer loop\n");
            writer.write("      vertex 0 0 0\n");
            writer.write("      vertex 1 0 0\n");
            writer.write("      vertex 1 1 0\n");
            writer.write("    endloop\n");
            writer.write("  endfacet\n");
            writer.write("  facet normal 0 0 1\n");
            writer.write("    outer loop\n");
            writer.write("      vertex 0 0 0\n");
            writer.write("      vertex 1 1 0\n");
            writer.write("      vertex 0 1 0\n");
            writer.write("    endloop\n");
            writer.write("  endfacet\n");
            writer.write("endsolid test_cube\n");
        }

        SceneNode root = new SceneNode("world");
        SceneNode importedNode = ModelImportScript.importModel(root, tempStl.getAbsolutePath());

        assertNotNull(importedNode, "Imported STL node should not be null");
        assertFalse(importedNode.getMeshes().isEmpty(), "Imported STL node must contain mesh data");
        assertFalse(importedNode.isRobotPart(), "Imported STL model must NOT be marked as a robot part");
        assertTrue(importedNode.isExcludedFromHierarchy(), "Imported STL model must be excluded from HierarchyPanel");
    }

    @Test
    public void testHierarchyPanelExclusion() {
        SceneNode root = new SceneNode("world");
        SceneNode robotLink = new SceneNode("base_link");
        robotLink.setRobotPart(true);
        root.addChild(robotLink);

        File objFile = new File("app/src/main/resources/models/test.obj");
        if (!objFile.exists()) {
            objFile = new File("src/main/resources/models/test.obj");
        }

        SceneNode envObject = ModelImportScript.importModel(root, objFile.getAbsolutePath());
        assertNotNull(envObject);

        // Verify that robotLink is part of hierarchy while envObject is excluded
        assertFalse(robotLink.isExcludedFromHierarchy(), "Robot link should be in hierarchy");
        assertTrue(envObject.isExcludedFromHierarchy(), "Environment object must be excluded from hierarchy");
    }

    @Test
    public void testRobotJsonExportExclusion() {
        SceneNode root = new SceneNode("world");
        SceneNode robotLink = new SceneNode("base_link");
        robotLink.setRobotPart(true);
        root.addChild(robotLink);

        File objFile = new File("app/src/main/resources/models/test.obj");
        if (!objFile.exists()) {
            objFile = new File("src/main/resources/models/test.obj");
        }

        SceneNode envObject = ModelImportScript.importModel(root, objFile.getAbsolutePath());
        assertNotNull(envObject);

        List<Joint> joints = new ArrayList<>();
        RobotDefinitionDTO dto = RobotJsonIO.fromSceneGraph(root, joints, "TestRobot");

        assertNotNull(dto.links);
        boolean containsRobotLink = false;
        boolean containsEnvObject = false;

        for (var link : dto.links) {
            if ("base_link".equals(link.id)) {
                containsRobotLink = true;
            }
            if (envObject.getId().equals(link.id)) {
                containsEnvObject = true;
            }
        }

        assertTrue(containsRobotLink, "Exported robot definition must contain robot link");
        assertFalse(containsEnvObject, "Exported robot definition must NOT contain imported environment model");
    }

    @Test
    public void testEnvironmentModelImporterAlias() {
        File objFile = new File("app/src/main/resources/models/test.obj");
        if (!objFile.exists()) {
            objFile = new File("src/main/resources/models/test.obj");
        }

        SceneNode root = new SceneNode("world");
        SceneNode node = EnvironmentModelImporter.importModel(root, objFile.getAbsolutePath());

        assertNotNull(node);
        assertTrue(node.isExcludedFromHierarchy());
        assertFalse(node.isRobotPart());
    }

    @Test
    public void testCollisionGroundExclusion() {
        CollisionFilter filter = new CollisionFilter();
        filter.setEnabled(true);
        filter.setGroundCollisionEnabled(true);

        File objFile = new File("app/src/main/resources/models/test.obj");
        if (!objFile.exists()) {
            objFile = new File("src/main/resources/models/test.obj");
        }

        SceneNode envObject = ModelImportScript.importModel(null, objFile.getAbsolutePath());
        assertNotNull(envObject);

        SceneNode robotLink = new SceneNode("base_link", envObject.getMesh());
        robotLink.setRobotPart(true);

        assertTrue(filter.shouldCheckGround(robotLink), "Robot link should check ground collision");
        assertFalse(filter.shouldCheckGround(envObject), "Environment object should not trigger ground collision");
    }

    @Test
    public void testInvalidFilePath() {
        SceneNode node = ModelImportScript.importModel("non_existent_file_path_12345.obj");
        assertNull(node, "Importing a non-existent file should return null without crashing");
    }
}
