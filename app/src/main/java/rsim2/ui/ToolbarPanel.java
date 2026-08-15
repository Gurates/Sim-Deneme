package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiWindowFlags;
import org.lwjgl.PointerBuffer;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.util.tinyfd.TinyFileDialogs;
import rsim2.core.Engine;
import rsim2.editor.ModelImporter;
import rsim2.editor.SelectionManager;
import rsim2.scene.SceneNode;

import java.util.List;

import static org.lwjgl.system.MemoryStack.stackPush;

public class ToolbarPanel {
    private final Engine engine;
    private SceneNode rootNode;
    private final SelectionManager selectionManager;

    public ToolbarPanel(Engine engine, SceneNode rootNode, SelectionManager selectionManager) {
        this.engine = engine;
        this.rootNode = rootNode;
        this.selectionManager = selectionManager;
    }

    public void setRootNode(SceneNode rootNode) {
        this.rootNode = rootNode;
    }

    public void render() {
        float displayWidth = ImGui.getIO().getDisplaySizeX();

        ImGui.setNextWindowPos(0.0f, 0.0f, ImGuiCond.Always);
        ImGui.setNextWindowSize(displayWidth, 40.0f, ImGuiCond.Always);

        int flags = ImGuiWindowFlags.NoMove
                | ImGuiWindowFlags.NoResize
                | ImGuiWindowFlags.NoCollapse
                | ImGuiWindowFlags.NoTitleBar
                | ImGuiWindowFlags.NoScrollbar;

        ImGui.begin("Toolbar", flags);

        if (ImGui.button("Load Project", 110.0f, 24.0f)) {
            try (MemoryStack stack = stackPush()) {
                PointerBuffer filters = stack.mallocPointer(1);
                filters.put(stack.UTF8("*.json"));
                filters.flip();

                String path = TinyFileDialogs.tinyfd_openFileDialog("Load Project", "", filters, "JSON Files (*.json)", false);
                if (path != null && !path.trim().isEmpty()) {
                    engine.loadProject(path);
                }
            }
        }

        ImGui.sameLine();
        if (ImGui.button("Save Project", 110.0f, 24.0f)) {
            String currentPath = engine.getCurrentProjectPath();
            if (currentPath != null && !currentPath.trim().isEmpty()) {
                engine.saveProject(currentPath);
            } else {
                saveProjectAs();
            }
        }

        ImGui.sameLine();
        if (ImGui.button("Save Project As", 130.0f, 24.0f)) {
            saveProjectAs();
        }

        ImGui.sameLine();
        ImGui.textDisabled("|");
        ImGui.sameLine();

        if (ImGui.button("Import Model", 120.0f, 24.0f)) {
            SceneNode imported = ModelImporter.importModel(rootNode);
            if (imported != null && selectionManager != null) {
                selectionManager.select(imported);
            }
        }

        ImGui.sameLine();
        if (ImGui.button("Import Model Group", 150.0f, 24.0f)) {
            List<SceneNode> group = ModelImporter.importModelsGroup(rootNode);
            if (!group.isEmpty() && selectionManager != null) {
                selectionManager.select(group.get(0));
            }
        }

        ImGui.sameLine();
        ImGui.textDisabled("|");
        ImGui.sameLine();

        SceneNode selected = selectionManager != null ? selectionManager.getSelected() : null;
        boolean canDelete = selected != null && selected.getParent() != null;

        ImGui.beginDisabled(!canDelete);
        if (ImGui.button("Delete Object", 110.0f, 24.0f)) {
            if (selected != null && selected.getParent() != null) {
                selected.getParent().removeChild(selected);
                selectionManager.clearSelection();
                selected.cleanup();
            }
        }
        ImGui.endDisabled();

        long now = System.currentTimeMillis();
        if (now - engine.getLastAutoSaveTime() < 2000) {
            ImGui.sameLine();
            ImGui.textDisabled("|");
            ImGui.sameLine();
            ImGui.textColored(0.4f, 0.85f, 0.4f, 1.0f, "Kaydedildi");
        }

        ImGui.end();
    }

    private void saveProjectAs() {
        try (MemoryStack stack = stackPush()) {
            PointerBuffer filters = stack.mallocPointer(1);
            filters.put(stack.UTF8("*.json"));
            filters.flip();

            String defaultName = engine.getCurrentProjectPath() != null ? engine.getCurrentProjectPath() : "robot.json";
            String path = TinyFileDialogs.tinyfd_saveFileDialog("Save Project As", defaultName, filters, "JSON Files (*.json)");
            if (path != null && !path.trim().isEmpty()) {
                engine.saveProject(path);
            }
        }
    }
}
