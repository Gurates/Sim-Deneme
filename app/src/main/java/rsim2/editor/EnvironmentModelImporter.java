package rsim2.editor;

import rsim2.scene.SceneNode;
import rsim2.script.ModelImportScript;

import java.io.File;
import java.util.List;

/**
 * Editor-level importer for environment and static models (.stl and .obj).
 * Ensures models are marked as non-robot objects and excluded from HierarchyPanel.
 *
 * Delegates to ModelImportScript for unified implementation.
 */
public class EnvironmentModelImporter {

    public static SceneNode importModel() {
        return ModelImportScript.importModel();
    }

    public static SceneNode importModel(String filePath) {
        return ModelImportScript.importModel(filePath);
    }

    public static SceneNode importModel(File file) {
        return ModelImportScript.importModel(file);
    }

    public static SceneNode importModel(SceneNode root) {
        return ModelImportScript.importModel(root);
    }

    public static SceneNode importModel(SceneNode root, String filePath) {
        return ModelImportScript.importModel(root, filePath);
    }

    public static SceneNode importModel(SceneNode root, String filePath, UpAxis upAxis) {
        return ModelImportScript.importModel(root, filePath, upAxis);
    }

    public static SceneNode importModel(SceneNode root, String filePath, UpAxis upAxis, float targetSize, boolean autoGround) {
        return ModelImportScript.importModel(root, filePath, upAxis, targetSize, autoGround);
    }

    public static List<SceneNode> importModels(String... filePaths) {
        return ModelImportScript.importModels(filePaths);
    }
}
