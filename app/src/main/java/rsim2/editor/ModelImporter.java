package rsim2.editor;

import org.joml.Vector3f;
import org.lwjgl.PointerBuffer;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.util.tinyfd.TinyFileDialogs;
import rsim2.graphics.Mesh;
import rsim2.io.ObjLoader;
import rsim2.io.StlLoader;
import rsim2.scene.SceneNode;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static org.lwjgl.system.MemoryStack.stackPush;

public class ModelImporter {
    private static final float TARGET_SIZE = 2.0f;

    public static SceneNode importModel(SceneNode root) {
        String selectedPath;

        try (MemoryStack stack = stackPush()) {
            PointerBuffer filters = stack.mallocPointer(2);
            filters.put(stack.UTF8("*.obj"));
            filters.put(stack.UTF8("*.stl"));
            filters.flip();

            selectedPath = TinyFileDialogs.tinyfd_openFileDialog(
                    "Select 3D Model",
                    "",
                    filters,
                    "3D Model Files (*.obj, *.stl)",
                    false
            );
        }

        if (selectedPath == null || selectedPath.trim().isEmpty()) {
            return null;
        }

        try {
            Mesh mesh;
            String lower = selectedPath.toLowerCase();
            if (lower.endsWith(".stl")) {
                mesh = StlLoader.load(selectedPath);
            } else {
                mesh = ObjLoader.load(selectedPath);
            }

            File file = new File(selectedPath);
            String fileName = file.getName();
            int dotIdx = fileName.lastIndexOf('.');
            String baseId = dotIdx > 0 ? fileName.substring(0, dotIdx) : fileName;

            Set<String> existingIds = new HashSet<>();
            collectIds(root, existingIds);

            String uniqueId = baseId;
            int counter = 1;
            while (existingIds.contains(uniqueId)) {
                uniqueId = baseId + "_" + counter++;
            }

            SceneNode newNode = new SceneNode(uniqueId, mesh);
            newNode.setSourceMeshPath(selectedPath);

            if (mesh != null && mesh.getMinBound() != null && mesh.getMaxBound() != null) {
                float sizeX = mesh.getMaxBound().x - mesh.getMinBound().x;
                float sizeY = mesh.getMaxBound().y - mesh.getMinBound().y;
                float sizeZ = mesh.getMaxBound().z - mesh.getMinBound().z;
                float longestSide = Math.max(sizeX, Math.max(sizeY, sizeZ));
                float scaleFactor = 1.0f;
                if (longestSide > 0.0001f) {
                    scaleFactor = TARGET_SIZE / longestSide;
                }
                newNode.getLocalScale().set(scaleFactor, scaleFactor, scaleFactor);
            }

            if (root != null) {
                root.addChild(newNode);
            }

            return newNode;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public static List<SceneNode> importModelsGroup(SceneNode root) {
        String selectedPaths;

        try (MemoryStack stack = stackPush()) {
            PointerBuffer filters = stack.mallocPointer(2);
            filters.put(stack.UTF8("*.obj"));
            filters.put(stack.UTF8("*.stl"));
            filters.flip();

            selectedPaths = TinyFileDialogs.tinyfd_openFileDialog(
                    "Select 3D Model Group",
                    "",
                    filters,
                    "3D Model Files (*.obj, *.stl)",
                    true
            );
        }

        if (selectedPaths == null || selectedPaths.trim().isEmpty()) {
            return Collections.emptyList();
        }

        String[] filePaths = selectedPaths.split("\\|");
        List<SceneNode> resultNodes = new ArrayList<>();
        Set<String> existingIds = new HashSet<>();
        collectIds(root, existingIds);

        Vector3f groupMin = new Vector3f(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
        Vector3f groupMax = new Vector3f(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE);

        for (String filePath : filePaths) {
            filePath = filePath.trim();
            if (filePath.isEmpty()) continue;

            try {
                Mesh mesh;
                String lower = filePath.toLowerCase();
                if (lower.endsWith(".stl")) {
                    mesh = StlLoader.load(filePath);
                } else {
                    mesh = ObjLoader.load(filePath);
                }

                File file = new File(filePath);
                String fileName = file.getName();
                int dotIdx = fileName.lastIndexOf('.');
                String baseId = dotIdx > 0 ? fileName.substring(0, dotIdx) : fileName;

                String uniqueId = baseId;
                int counter = 1;
                while (existingIds.contains(uniqueId)) {
                    uniqueId = baseId + "_" + counter++;
                }
                existingIds.add(uniqueId);

                SceneNode newNode = new SceneNode(uniqueId, mesh);
                newNode.setSourceMeshPath(filePath);

                if (mesh != null && mesh.getMinBound() != null && mesh.getMaxBound() != null) {
                    groupMin.min(mesh.getMinBound());
                    groupMax.max(mesh.getMaxBound());
                }

                resultNodes.add(newNode);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        if (resultNodes.isEmpty()) {
            return Collections.emptyList();
        }

        float sizeX = groupMax.x - groupMin.x;
        float sizeY = groupMax.y - groupMin.y;
        float sizeZ = groupMax.z - groupMin.z;
        float longestSide = Math.max(sizeX, Math.max(sizeY, sizeZ));
        float groupScaleFactor = 1.0f;
        if (longestSide > 0.0001f) {
            groupScaleFactor = TARGET_SIZE / longestSide;
        }

        for (SceneNode node : resultNodes) {
            node.getLocalScale().set(groupScaleFactor, groupScaleFactor, groupScaleFactor);
            if (root != null) {
                root.addChild(node);
            }
        }

        return resultNodes;
    }

    private static void collectIds(SceneNode node, Set<String> ids) {
        if (node == null) return;
        ids.add(node.getId());
        for (SceneNode child : node.getChildren()) {
            collectIds(child, ids);
        }
    }
}
