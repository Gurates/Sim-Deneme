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

public class ModelImportScript {

    public static final float DEFAULT_TARGET_SIZE = 2.0f;

    public static SceneNode importModel() {
        return importModel((SceneNode) null, (String) null, UpAxis.Y_UP);
    }

    public static SceneNode importModel(String filePath) {
        return importModel((SceneNode) null, filePath, UpAxis.Y_UP);
    }

    public static SceneNode importModel(File file) {
        if (file == null) {
            return null;
        }
        return importModel((SceneNode) null, file.getAbsolutePath(), UpAxis.Y_UP);
    }

    public static SceneNode importModel(SceneNode root) {
        return importModel(root, (String) null, UpAxis.Y_UP);
    }

    public static SceneNode importModel(SceneNode root, String filePath) {
        return importModel(root, filePath, UpAxis.Y_UP);
    }

    public static SceneNode importModel(SceneNode root, String filePath, UpAxis upAxis) {
        return importModel(root, filePath, upAxis, DEFAULT_TARGET_SIZE, true);
    }

    public static SceneNode importModel(SceneNode root, String filePath, UpAxis upAxis, float targetSize, boolean autoGround) {
        String selectedPath = filePath;
        if (selectedPath == null || selectedPath.trim().isEmpty()) {
            selectedPath = openFileDialog();
        }

        if (selectedPath == null || selectedPath.trim().isEmpty()) {
            System.out.println("[ModelImportScript] Model selection was cancelled or path is empty.");
            return null;
        }

        File file = new File(selectedPath);
        if (!file.exists() || !file.isFile()) {
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

            newNode.setRobotPart(false);
            newNode.setExcludedFromHierarchy(true);

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

            if (upAxis == UpAxis.Z_UP) {
                newNode.getLocalRotation().rotateX((float) Math.toRadians(-90));
            }

            if (autoGround && mesh.getMinBound() != null) {
                float minY = mesh.getMinBound().y * scaleFactor;
                if (minY < 0.0f) {
                    newNode.getLocalPosition().y = -minY;
                }
            }

            if (targetRoot != null) {
                targetRoot.addChild(newNode);
            }

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

    public static SceneNode importEnvironmentModel() {
        return importModel();
    }

    public static SceneNode importEnvironmentModel(String filePath) {
        return importModel(filePath);
    }

    public static SceneNode importObject() {
        return importModel();
    }

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
