package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImInt;
import org.lwjgl.PointerBuffer;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.util.tinyfd.TinyFileDialogs;
import rsim2.core.Engine;
import rsim2.editor.ModelImporter;
import rsim2.editor.SelectionManager;
import rsim2.editor.UpAxis;
import rsim2.editor.commands.CommandHistory;
import rsim2.editor.commands.ImportModelCommand;
import rsim2.scene.SceneNode;

import java.util.List;

import static org.lwjgl.system.MemoryStack.stackPush;

public class ToolbarPanel {
    private final Engine engine;
    private SceneNode rootNode;
    private final SelectionManager selectionManager;
    private CommandHistory commandHistory;

    private final ImInt selectedUpAxisIdx = new ImInt(0); // 0 = Y-up, 1 = Z-up
    private int importTypePending = 0; // 0 = Single, 1 = Group, 2 = Multi-Part Link

    public ToolbarPanel(Engine engine, SceneNode rootNode, SelectionManager selectionManager,
            CommandHistory commandHistory) {
        this.engine = engine;
        this.rootNode = rootNode;
        this.selectionManager = selectionManager;
        this.commandHistory = commandHistory;
    }

    public void setRootNode(SceneNode rootNode) {
        this.rootNode = rootNode;
    }

    public void setCommandHistory(CommandHistory commandHistory) {
        this.commandHistory = commandHistory;
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

        // Project File Operations
        if (ImGui.button("Load Project", 95.0f, 24.0f)) {
            try (MemoryStack stack = stackPush()) {
                PointerBuffer filters = stack.mallocPointer(1);
                filters.put(stack.UTF8("*.json"));
                filters.flip();

                String path = TinyFileDialogs.tinyfd_openFileDialog("Load Project", "", filters, "JSON Files (*.json)",
                        false);
                if (path != null && !path.trim().isEmpty()) {
                    engine.loadProject(path);
                }
            }
        }

        ImGui.sameLine();
        if (ImGui.button("Save Project", 95.0f, 24.0f)) {
            String currentPath = engine.getCurrentProjectPath();
            if (currentPath != null && !currentPath.trim().isEmpty()) {
                engine.saveProject(currentPath);
            } else {
                saveProjectAs();
            }
        }

        ImGui.sameLine();
        if (ImGui.button("Save As", 65.0f, 24.0f)) {
            saveProjectAs();
        }

        ImGui.sameLine();
        ImGui.textDisabled("|");
        ImGui.sameLine();

        // Undo / Redo Buttons
        boolean canUndo = commandHistory != null && commandHistory.canUndo();
        ImGui.beginDisabled(!canUndo);
        if (ImGui.button("Undo", 55.0f, 24.0f)) {
            if (commandHistory != null) {
                commandHistory.undo();
            }
        }
        ImGui.endDisabled();

        ImGui.sameLine();
        boolean canRedo = commandHistory != null && commandHistory.canRedo();
        ImGui.beginDisabled(!canRedo);
        if (ImGui.button("Redo", 55.0f, 24.0f)) {
            if (commandHistory != null) {
                commandHistory.redo();
            }
        }
        ImGui.endDisabled();

        ImGui.sameLine();
        ImGui.textDisabled("|");
        ImGui.sameLine();

        // Import URDF Button
        ImGui.pushStyleColor(imgui.flag.ImGuiCol.Button, 0.15f, 0.55f, 0.45f, 1.0f);
        ImGui.pushStyleColor(imgui.flag.ImGuiCol.ButtonHovered, 0.20f, 0.68f, 0.55f, 1.0f);
        if (ImGui.button("Import URDF", 95.0f, 24.0f)) {
            try (MemoryStack stack = stackPush()) {
                PointerBuffer filters = stack.mallocPointer(1);
                filters.put(stack.UTF8("*.urdf"));
                filters.flip();

                String path = TinyFileDialogs.tinyfd_openFileDialog("Select URDF Robot File", "", filters,
                        "URDF Files (*.urdf)", false);
                if (path != null && !path.trim().isEmpty()) {
                    engine.loadUrdf(path);
                }
            }
        }
        ImGui.popStyleColor(2);
        if (ImGui.isItemHovered()) {
            ImGui.setTooltip(
                    "Import URDF Robot (.urdf): Tek tıkla tüm linkleri, eklemleri, eksenleri ve limitleri otomatik kurar.");
        }

        ImGui.sameLine();
        ImGui.textDisabled("|");
        ImGui.sameLine();

        // Import Buttons with Tooltips
        if (ImGui.button("Import Model", 95.0f, 24.0f)) {
            importTypePending = 0;
            ImGui.openPopup("Import 3D Model");
        }
        if (ImGui.isItemHovered()) {
            ImGui.setTooltip("Import Model: 1 dosya = 1 link");
        }

        ImGui.sameLine();
        if (ImGui.button("Import Group", 95.0f, 24.0f)) {
            importTypePending = 1;
            ImGui.openPopup("Import 3D Model");
        }
        if (ImGui.isItemHovered()) {
            ImGui.setTooltip("Import Model Group: Multiple files = Multiple SEPARATE links (offsets preserved)");
        }

        ImGui.sameLine();
        if (ImGui.button("Multi-Part Link", 105.0f, 24.0f)) {
            importTypePending = 2;
            ImGui.openPopup("Import 3D Model");
        }
        if (ImGui.isItemHovered()) {
            ImGui.setTooltip(
                    "Import Multi-Part Link: Çoklu dosya = TEK link (alt-parçalar birleşir, birlikte hareket eder)");
        }

        // Import Up-Axis Modal Popup
        if (ImGui.beginPopupModal("Import 3D Model", ImGuiWindowFlags.AlwaysAutoResize)) {
            ImGui.text("Select Model Coordinate System (Up Axis):");
            ImGui.spacing();

            ImGui.radioButton("Y-up (Unity, standard 3D meshes)", selectedUpAxisIdx, 0);
            ImGui.radioButton("Z-up (SolidWorks, Fusion 360, CAD, Robotics)", selectedUpAxisIdx, 1);

            ImGui.spacing();
            ImGui.textDisabled("Note: If your model was exported from SolidWorks or Fusion 360, select Z-up.");
            ImGui.spacing();
            ImGui.separator();
            ImGui.spacing();

            if (ImGui.button("Select File(s)...", 140.0f, 26.0f)) {
                UpAxis upAxis = (selectedUpAxisIdx.get() == 1) ? UpAxis.Z_UP : UpAxis.Y_UP;
                ImGui.closeCurrentPopup();

                if (importTypePending == 1) {
                    List<SceneNode> group = ModelImporter.importModelsGroup(null, upAxis);
                    if (!group.isEmpty()) {
                        if (commandHistory != null) {
                            commandHistory.executeAndRecord(new ImportModelCommand(group, rootNode, selectionManager));
                        } else {
                            for (SceneNode node : group) {
                                rootNode.addChild(node);
                            }
                            if (selectionManager != null) {
                                selectionManager.select(group.get(0));
                            }
                        }
                    }
                } else if (importTypePending == 2) {
                    SceneNode multiLink = ModelImporter.importMultiMeshLink(null, upAxis);
                    if (multiLink != null) {
                        if (commandHistory != null) {
                            commandHistory
                                    .executeAndRecord(new ImportModelCommand(multiLink, rootNode, selectionManager));
                        } else {
                            rootNode.addChild(multiLink);
                            if (selectionManager != null) {
                                selectionManager.select(multiLink);
                            }
                        }
                    }
                } else {
                    SceneNode imported = ModelImporter.importModel(null, upAxis);
                    if (imported != null) {
                        if (commandHistory != null) {
                            commandHistory
                                    .executeAndRecord(new ImportModelCommand(imported, rootNode, selectionManager));
                        } else {
                            rootNode.addChild(imported);
                            if (selectionManager != null) {
                                selectionManager.select(imported);
                            }
                        }
                    }
                }
            }

            ImGui.sameLine();
            if (ImGui.button("Cancel", 80.0f, 26.0f)) {
                ImGui.closeCurrentPopup();
            }

            ImGui.endPopup();
        }

        ImGui.sameLine();
        ImGui.textDisabled("|");
        ImGui.sameLine();

        SceneNode selected = selectionManager != null ? selectionManager.getSelected() : null;
        boolean canDelete = selected != null && selected.getParent() != null;

        ImGui.beginDisabled(!canDelete);
        if (ImGui.button("Delete", 65.0f, 24.0f)) {
            if (selected != null) {
                engine.deleteNode(selected);
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
            String path = TinyFileDialogs.tinyfd_saveFileDialog("Save Project As", defaultName, filters,
                    "JSON Files (*.json)");
            if (path != null && !path.trim().isEmpty()) {
                engine.saveProject(path);
            }
        }
    }
}
