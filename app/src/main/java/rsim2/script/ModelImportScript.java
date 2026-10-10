package rsim2.script;

import org.joml.Vector3f;
import org.lwjgl.PointerBuffer;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.util.tinyfd.TinyFileDialogs;
import rsim2.core.Engine;
import rsim2.editor.UpAxis;
import rsim2.graphics.Mesh;
import rsim2.io.ObjLoader;
import rsim2.io.StlLoader;
import rsim2.scene.SceneNode;

import java.io.File;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static org.lwjgl.system.MemoryStack.stackPush;

/**
 * Script for importing 3D models (.stl and .obj) directly from computer filesystem files.
 *
 * Models imported by this script are marked as non-robot environment/static objects
 * and are explicitly EXCLUDED from HierarchyPanel and robot JSON definition exports.
 *
 * This script provides static functions that can be called directly from outside:
 * - ModelImportScript.importModel()
 * - ModelImportScript.importModel(String filePath)
 * - ModelImportScript.importModel(File file)
 * - ModelImportScript.importModel(SceneNode rootNode)
 * - ModelImportScript.importModel(SceneNode rootNode, String filePath)
 * - ModelImportScript.importModel(SceneNode rootNode, String filePath, UpAxis upAxis)
 */
public class ModelImportScript {

    public static final float DEFAULT_TARGET_SIZE = 2.0f;

    /**
     * Directly imports a 3D model (.stl or .obj) via file dialog into the current active scene.
     * The imported model is NOT added to HierarchyPanel.
     *
     * @return the created SceneNode, or null if cancelled/failed
     */
    public static SceneNode importModel() {
        return importModel((SceneNode) null, (String) null, UpAxis.Y_UP);
    }

    /**
     * Directly imports a 3D model (.stl or .obj) from the given computer file path into the current active scene.
     * The imported model is NOT added to HierarchyPanel.
     *
     * @param filePath absolute or relative path to the .stl or .obj file on the computer
     * @return the created SceneNode, or null if failed
     */
    public static SceneNode importModel(String filePath) {
        return importModel((SceneNode) null, filePath, UpAxis.Y_UP);
    }

    /**
     * Directly imports a 3D model from a File object into the current active scene.
     * The imported model is NOT added to HierarchyPanel.
     *
     * @param file the File pointing to a .stl or .obj file
     * @return the created SceneNode, or null if failed
     */
    public static SceneNode importModel(File file) {
        if (file == null) {
            return null;
        }
        return importModel((SceneNode) null, file.getAbsolutePath(), UpAxis.Y_UP);
    }

    /**
     * Directly imports a 3D model via file dialog and attaches it to the specified root node.
     * The imported model is NOT added to HierarchyPanel.
     *
     * @param root the parent scene node (e.g. world)
     * @return the created SceneNode, or null if cancelled/failed
     */
    public static SceneNode importModel(SceneNode root) {
        return importModel(root, (String) null, UpAxis.Y_UP);
    }

    /**
     * Directly imports a 3D model from the given computer file path and attaches it to the specified root node.
     * The imported model is NOT added to HierarchyPanel.
     *
     * @param root parent scene node (if null, Engine.getInstance().getRootNode() will be used if available)
     * @param filePath computer file path to .stl or .obj (if null, an open file dialog will prompt user)
     * @return the created SceneNode, or null if failed
     */
    public static SceneNode importModel(SceneNode root, String filePath) {
        return importModel(root, filePath, UpAxis.Y_UP);
    }

    /**
     * Directly imports a 3D model (.stl or .obj) with full configuration.
     * The imported model is marked as non-robot and is NOT added to HierarchyPanel.
     *
     * @param root parent scene node (if null, Engine.getInstance().getRootNode() will be used if available)
     * @param filePath computer file path to .stl or .obj (if null, opens file dialog)
     * @param upAxis coordinate up-axis (Y_UP or Z_UP)
     * @return the created SceneNode, or null if failed
     */
    public static SceneNode importModel(SceneNode root, String filePath, UpAxis upAxis) {
        return importModel(root, filePath, upAxis, DEFAULT_TARGET_SIZE, true);
    }

    /**
     * Complete model import implementation.
     *
     * @param root parent scene node
     * @param filePath file path to model (.stl or .obj), or null to prompt file dialog
     * @param upAxis coordinate orientation
     * @param targetSize normalized target bounding dimension
     * @param autoGround whether to adjust vertical position so the model rests on ground (y >= 0)
     * @return the created SceneNode
     */
    public static SceneNode importModel(SceneNode root, String filePath, UpAxis upAxis, float targetSize, boolean autoGround) {
        String selectedPath = filePath;

        // If no file path is provided, prompt user with file dialog
        if (selectedPath == null || selectedPath.trim().isEmpty()) {
            selectedPath = openFileDialog();
        }

        if (selectedPath == null || selectedPath.trim().isEmpty()) {
            System.out.println("[ModelImportScript] Model selection was cancelled or path is empty.");
            return null;
        }

        File file = new File(selectedPath);
        if (!file.exists() || !file.isFile()) {
            // Also check if accessible via classloader resource as fallback
            if (ModelImportScript.class.getResource(selectedPath.startsWith("/") ? selectedPath : "/" + selectedPath) == null) {
                System.err.println("[ModelImportScript] Model file not found: " + selectedPath);
                return null;
            }
        }

        try {
            Mesh mesh = loadMesh(selectedPath);
            if (mesh == null) {
                System.err.println("[ModelImportScript] Failed to load mesh from: " + selectedPath);
                return null;
            }
            mesh.setSourcePath(selectedPath);

            // Determine unique ID for the scene node
            String baseId = extractBaseId(file.getName());
            Set<String> existingIds = new HashSet<>();
            SceneNode targetRoot = resolveRootNode(root);
            if (targetRoot != null) {
                collectIds(targetRoot, existingIds);
            }

            String uniqueId = baseId;
            int counter = 1;
            while (existingIds.contains(uniqueId)) {
                uniqueId = baseId + "_" + counter++;
            }

            SceneNode newNode = new SceneNode(uniqueId, mesh);
            newNode.setSourceMeshPath(selectedPath);

            // IMPORTANT: Mark as non-robot part so it is excluded from HierarchyPanel and robot exports
            newNode.setRobotPart(false);
            newNode.setExcludedFromHierarchy(true);

            // Scale normalization
            float scaleFactor = 1.0f;
            if (mesh.getMinBound() != null && mesh.getMaxBound() != null) {
                float sizeX = mesh.getMaxBound().x - mesh.getMinBound().x;
                float sizeY = mesh.getMaxBound().y - mesh.getMinBound().y;
                float sizeZ = mesh.getMaxBound().z - mesh.getMinBound().z;
                float longestSide = Math.max(sizeX, Math.max(sizeY, sizeZ));
                if (longestSide > 0.0001f && targetSize > 0.0f) {
                    scaleFactor = targetSize / longestSide;
                }
                newNode.getLocalScale().set(scaleFactor, scaleFactor, scaleFactor);
            }

            // Up-axis alignment
            if (upAxis == UpAxis.Z_UP) {
                newNode.getLocalRotation().rotateX((float) Math.toRadians(-90));
            }

            // Ground placement: ensure lowest point rests on ground plane (y = 0)
            if (autoGround && mesh.getMinBound() != null) {
                float minY = mesh.getMinBound().y * scaleFactor;
                if (minY < 0.0f) {
                    newNode.getLocalPosition().y = -minY;
                }
            }

            // Attach to root node
            if (targetRoot != null) {
                targetRoot.addChild(newNode);
            }

            // If engine is active, select node in editor
            Engine engine = Engine.getInstance();
            if (engine != null && engine.getSelectionManager() != null) {
                engine.getSelectionManager().select(newNode);
            }

            System.out.printf("[ModelImportScript] Successfully imported model '%s' from '%s' (Excluded from HierarchyPanel)%n",
                    newNode.getId(), selectedPath);

            return newNode;
        } catch (Exception e) {
            System.err.println("[ModelImportScript] Error importing model from " + selectedPath + ": " + e.getMessage());
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Batch import multiple model files at once into the active scene.
     */
    public static List<SceneNode> importModels(String... filePaths) {
        List<SceneNode> nodes = new ArrayList<>();
        if (filePaths == null) return nodes;

        for (String path : filePaths) {
            SceneNode node = importModel(path);
            if (node != null) {
                nodes.add(node);
            }
        }
        return nodes;
    }

    /**
     * Alias for importModel() with descriptive naming.
     */
    public static SceneNode importEnvironmentModel() {
        return importModel();
    }

    /**
     * Alias for importModel(filePath) with descriptive naming.
     */
    public static SceneNode importEnvironmentModel(String filePath) {
        return importModel(filePath);
    }

    /**
     * Alias for importModel() with object naming.
     */
    public static SceneNode importObject() {
        return importModel();
    }

    /**
     * Alias for importModel(filePath) with object naming.
     */
    public static SceneNode importObject(String filePath) {
        return importModel(filePath);
    }

    private static String openFileDialog() {
        try (MemoryStack stack = stackPush()) {
            PointerBuffer filters = stack.mallocPointer(2);
            filters.put(stack.UTF8("*.obj"));
            filters.put(stack.UTF8("*.stl"));
            filters.flip();

            return TinyFileDialogs.tinyfd_openFileDialog(
                    "Select 3D Model (.obj, .stl)",
                    "",
                    filters,
                    "3D Model Files (*.obj, *.stl)",
                    false
            );
        } catch (Throwable t) {
            System.err.println("[ModelImportScript] File dialog failed: " + t.getMessage());
            return null;
        }
    }

    private static Mesh loadMesh(String path) throws Exception {
        String lower = path.toLowerCase();
        if (lower.endsWith(".stl")) {
            return StlLoader.load(path);
        } else if (lower.endsWith(".obj")) {
            return ObjLoader.load(path);
        } else {
            // Attempt OBJ loader first, fall back to STL loader
            try {
                return ObjLoader.load(path);
            } catch (Exception ignored) {
                return StlLoader.load(path);
            }
        }
    }

    private static SceneNode resolveRootNode(SceneNode providedRoot) {
        if (providedRoot != null) {
            return providedRoot;
        }
        Engine engine = Engine.getInstance();
        if (engine != null) {
            return engine.getRootNode();
        }
        return null;
    }

    private static String extractBaseId(String fileName) {
        if (fileName == null || fileName.isEmpty()) {
            return "model_object";
        }
        int dotIdx = fileName.lastIndexOf('.');
        String baseId = dotIdx > 0 ? fileName.substring(0, dotIdx) : fileName;
        return baseId.trim().isEmpty() ? "model_object" : baseId;
    }

    private static void collectIds(SceneNode node, Set<String> ids) {
        if (node == null) return;
        ids.add(node.getId());
        for (SceneNode child : node.getChildren()) {
            collectIds(child, ids);
        }
    }
}
