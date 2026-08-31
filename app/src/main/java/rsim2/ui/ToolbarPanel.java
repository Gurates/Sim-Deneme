package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiSliderFlags;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
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
import rsim2.motion.MotionPlayer;
import rsim2.motion.MotionSequence;
import rsim2.scene.SceneNode;

import static org.lwjgl.system.MemoryStack.stackPush;

public class ToolbarPanel {
    private final Engine engine;
    private SceneNode rootNode;
    private final SelectionManager selectionManager;
    private CommandHistory commandHistory;

    private final ImInt selectedUpAxisIdx = new ImInt(0);
    private int importTypePending = 0;
    private final AIPanel aiPanel;
    private BridgePanel bridgePanel;
    private CollisionPanel collisionPanel;

    public ToolbarPanel(Engine engine, SceneNode rootNode, SelectionManager selectionManager,
            CommandHistory commandHistory, AIPanel aiPanel) {
        this(engine, rootNode, selectionManager, commandHistory, aiPanel, null, null);
    }

    public ToolbarPanel(Engine engine, SceneNode rootNode, SelectionManager selectionManager,
            CommandHistory commandHistory, AIPanel aiPanel, BridgePanel bridgePanel) {
        this(engine, rootNode, selectionManager, commandHistory, aiPanel, bridgePanel, null);
    }

    public ToolbarPanel(Engine engine, SceneNode rootNode, SelectionManager selectionManager,
            CommandHistory commandHistory, AIPanel aiPanel, BridgePanel bridgePanel, CollisionPanel collisionPanel) {
        this.engine = engine;
        this.rootNode = rootNode;
        this.selectionManager = selectionManager;
        this.commandHistory = commandHistory;
        this.aiPanel = aiPanel;
        this.bridgePanel = bridgePanel;
        this.collisionPanel = collisionPanel;
    }

    public void setCollisionPanel(CollisionPanel collisionPanel) {
        this.collisionPanel = collisionPanel;
    }

    public void setBridgePanel(BridgePanel bridgePanel) {
        this.bridgePanel = bridgePanel;
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

        if (ImGui.button("Load Project", 90.0f, 24.0f)) {
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
        if (ImGui.button("Save Project", 90.0f, 24.0f)) {
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

        ImGui.pushStyleColor(ImGuiCol.Button, 0.15f, 0.55f, 0.45f, 1.0f);
        ImGui.pushStyleColor(ImGuiCol.ButtonHovered, 0.20f, 0.68f, 0.55f, 1.0f);
        if (ImGui.button("Import URDF", 90.0f, 24.0f)) {
            try (MemoryStack stack = stackPush()) {
                PointerBuffer filters = stack.mallocPointer(2);
                filters.put(stack.UTF8("*.urdf"));
                filters.put(stack.UTF8("*.xml"));
                filters.flip();

                String path = TinyFileDialogs.tinyfd_openFileDialog("Select URDF / XML Robot File", "", filters,
                        "URDF / XML Robot Files (*.urdf, *.xml)", false);
                if (path != null && !path.trim().isEmpty()) {
                    engine.loadUrdf(path);
                }
            }
        }
        ImGui.popStyleColor(2);

        ImGui.sameLine();
        if (ImGui.button("Import Model", 90.0f, 24.0f)) {
            ImGui.openPopup("Select Up Axis");
            importTypePending = 0;
        }

        if (ImGui.beginPopupModal("Select Up Axis", ImGuiWindowFlags.AlwaysAutoResize)) {
            ImGui.text("What is the Up Axis orientation of the imported model?");
            ImGui.spacing();

            String[] upAxisOptions = { "Y-Up (Blender / OpenGL / Unity)", "Z-Up (ROS / Gazebo / SolidWorks)" };
            ImGui.combo("Up Axis", selectedUpAxisIdx, upAxisOptions);

            ImGui.spacing();
            ImGui.separator();
            ImGui.spacing();

            if (ImGui.button("Import", 100.0f, 26.0f)) {
                UpAxis upAxis = (selectedUpAxisIdx.get() == 1) ? UpAxis.Z_UP : UpAxis.Y_UP;
                ImGui.closeCurrentPopup();

                if (importTypePending == 1) {
                    try (MemoryStack stack = stackPush()) {
                        PointerBuffer filters = stack.mallocPointer(2);
                        filters.put(stack.UTF8("*.urdf"));
                        filters.put(stack.UTF8("*.xml"));
                        filters.flip();

                        String path = TinyFileDialogs.tinyfd_openFileDialog("Select URDF / XML Robot File", "", filters,
                                "URDF / XML Robot Files (*.urdf, *.xml)", false);
                        if (path != null && !path.trim().isEmpty()) {
                            engine.loadUrdf(path);
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
        if (ImGui.button("Delete", 60.0f, 24.0f)) {
            if (selected != null) {
                engine.deleteNode(selected);
            }
        }
        ImGui.endDisabled();

        MotionPlayer motionPlayer = engine.getMotionPlayer();
        boolean hasSequence = (motionPlayer != null && motionPlayer.hasSequence());

        float centerControlsWidth = 330.0f;
        float centerStartX = (displayWidth - centerControlsWidth) * 0.5f;

        if (ImGui.getCursorPosX() < centerStartX) {
            ImGui.sameLine(centerStartX);
        } else {
            ImGui.sameLine();
        }

        ImGui.textDisabled("|");
        ImGui.sameLine();

        boolean isPlaying = (motionPlayer != null && motionPlayer.isPlaying());
        if (isPlaying) {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.85f, 0.50f, 0.15f, 1.0f);
            ImGui.pushStyleColor(ImGuiCol.ButtonHovered, 0.95f, 0.60f, 0.25f, 1.0f);
            if (ImGui.button("|| Pause", 68.0f, 24.0f)) {
                if (motionPlayer != null) {
                    motionPlayer.pause();
                }
            }
            ImGui.popStyleColor(2);
        } else {
            if (hasSequence) {
                ImGui.pushStyleColor(ImGuiCol.Button, 0.18f, 0.68f, 0.35f, 1.0f);
                ImGui.pushStyleColor(ImGuiCol.ButtonHovered, 0.22f, 0.78f, 0.42f, 1.0f);
            }
            if (ImGui.button("> Play", 68.0f, 24.0f)) {
                if (motionPlayer != null) {
                    motionPlayer.play();
                }
            }
            if (hasSequence) {
                ImGui.popStyleColor(2);
            }
        }
        if (ImGui.isItemHovered()) {
            ImGui.setTooltip(hasSequence ? "Play / Pause Motion" : "Generate a motion in AI panel to play it here.");
        }

        ImGui.sameLine();
        if (ImGui.button("[] Stop", 60.0f, 24.0f)) {
            if (motionPlayer != null) {
                motionPlayer.stop();
                motionPlayer.seek(0.0f, engine.getJoints());
            }
        }
        if (ImGui.isItemHovered()) {
            ImGui.setTooltip("Stop and Reset Motion to 0.00s");
        }

        ImGui.sameLine();
        boolean loopVal = (motionPlayer != null && motionPlayer.isLooping());
        ImBoolean loopBool = new ImBoolean(loopVal);
        if (ImGui.checkbox("Loop", loopBool)) {
            if (motionPlayer != null) {
                motionPlayer.setLooping(loopBool.get());
            }
        }

        if (hasSequence && motionPlayer.getSequence() != null) {
            MotionSequence seq = motionPlayer.getSequence();
            ImGui.sameLine();
            float duration = Math.max(0.01f, seq.getDurationSeconds());
            float curTime = motionPlayer.getCurrentTime();
            ImFloat timeVal = new ImFloat(curTime);

            ImGui.setNextItemWidth(90.0f);
            if (ImGui.sliderFloat("##topTimeline", timeVal.getData(), 0.0f, duration, "%.1fs", ImGuiSliderFlags.None)) {
                motionPlayer.seek(timeVal.get(), engine.getJoints());
            }
            if (ImGui.isItemHovered()) {
                ImGui.setTooltip(String.format("Motion: %s (%.2fs / %.2fs)", seq.getName(), curTime, duration));
            }
        }

        ImGui.sameLine(displayWidth - 335.0f);

        long now = System.currentTimeMillis();
        if (now - engine.getLastAutoSaveTime() < 2000) {
            ImGui.textColored(0.4f, 0.85f, 0.4f, 1.0f, "Saved");
            ImGui.sameLine();
            ImGui.textDisabled("|");
            ImGui.sameLine();
        }

        boolean collisionActive = (collisionPanel != null && collisionPanel.isVisible());
        boolean hasCollisions = (engine.getCollisionWorld() != null && engine.getCollisionWorld().getLastResult().hasCollision());
        int colCount = hasCollisions ? engine.getCollisionWorld().getLastResult().getCollisionCount() : 0;

        if (hasCollisions) {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.85f, 0.22f, 0.22f, 1.0f);
            ImGui.pushStyleColor(ImGuiCol.ButtonHovered, 0.95f, 0.35f, 0.35f, 1.0f);
        } else if (collisionActive) {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.3f, 0.6f, 0.75f, 1.0f);
            ImGui.pushStyleColor(ImGuiCol.ButtonHovered, 0.4f, 0.7f, 0.85f, 1.0f);
        }
        String colBtnLabel = hasCollisions ? "Col (" + colCount + ")" : "Collision";
        if (ImGui.button(colBtnLabel, 80.0f, 24.0f)) {
            if (collisionPanel != null) {
                collisionPanel.toggleVisible();
            }
        }
        if (hasCollisions || collisionActive) {
            ImGui.popStyleColor(2);
        }
        if (ImGui.isItemHovered()) {
            ImGui.setTooltip(hasCollisions ? "Collision Alert! " + colCount + " contact(s) detected. Click to view."
                                           : "Collision Detection System: Self-collision & ground contact");
        }

        ImGui.sameLine();

        boolean bridgeActive = (bridgePanel != null && bridgePanel.isVisible());
        if (bridgeActive) {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.65f, 0.55f, 1.0f);
        }
        if (ImGui.button("Sim2Real", 70.0f, 24.0f)) {
            if (bridgePanel != null) {
                bridgePanel.toggleVisible();
            }
        }
        if (bridgeActive) {
            ImGui.popStyleColor();
        }
        if (ImGui.isItemHovered()) {
            ImGui.setTooltip("Sim-to-Real Hardware Bridge: Live ESP32/robot sync & telemetry");
        }

        ImGui.sameLine();

        boolean aiActive = (aiPanel != null && aiPanel.isVisible());
        if (aiActive) {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.55f, 0.85f, 1.0f);
        }
        if (ImGui.button("AI", 45.0f, 24.0f)) {
            if (aiPanel != null) {
                aiPanel.toggleVisible();
            }
        }
        if (aiActive) {
            ImGui.popStyleColor();
        }
        if (ImGui.isItemHovered()) {
            ImGui.setTooltip("AI Assistant: Robot control & motion planning");
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