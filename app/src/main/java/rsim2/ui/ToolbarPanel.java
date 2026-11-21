package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiHoveredFlags;
import imgui.flag.ImGuiSliderFlags;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.lwjgl.PointerBuffer;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.util.tinyfd.TinyFileDialogs;
import java.nio.file.Path;
import java.util.Locale;
import rsim2.core.Engine;
import rsim2.editor.EnvironmentModelImporter;
import rsim2.editor.ModelImporter;
import rsim2.editor.SelectionManager;
import rsim2.editor.UpAxis;
import rsim2.editor.commands.CommandHistory;
import rsim2.editor.commands.ImportModelCommand;
import rsim2.io.MotionSequenceIO;
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
    private final McpPanel mcpPanel;
    private String motionStatus = "";
    private final SceneObjectHierarchy sceneObjectHierarchy = new SceneObjectHierarchy();

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
        this.mcpPanel = new McpPanel(engine.getMcpBridge());
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
        motionStatus = "";
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

        if (ImGui.button("Load Project", 100.0f, 24.0f)) {
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
        if (ImGui.button("Save Project", 100.0f, 24.0f)) {
            String currentPath = engine.getCurrentProjectPath();
            if (currentPath != null && !currentPath.trim().isEmpty()) {
                engine.saveProject(currentPath);
            } else {
                saveProjectAs();
            }
        }

        ImGui.sameLine();
        ImGui.pushStyleColor(ImGuiCol.Button, 0.15f, 0.55f, 0.45f, 1.0f);
        ImGui.pushStyleColor(ImGuiCol.ButtonHovered, 0.20f, 0.68f, 0.55f, 1.0f);
        if (ImGui.button("Import Project", 120.0f, 24.0f)) {
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

        if (ImGui.button("Import Object", 120.0f, 24.0f)) {
            SceneNode model = EnvironmentModelImporter.importModel();
        }

        ImGui.sameLine();






        MotionPlayer motionPlayer = engine.getMotionPlayer();
        boolean hasSequence = (motionPlayer != null && motionPlayer.hasSequence());

        float centerControlsWidth = 330.0f;
        float centerStartX = (displayWidth - centerControlsWidth) * 0.5f;

        if (ImGui.getCursorPosX() < centerStartX) {
            ImGui.sameLine(centerStartX);
        } else {
            ImGui.sameLine();
        }


        boolean isPlaying = (motionPlayer != null && motionPlayer.isPlaying());
        String playbackBlockReason = isPlaying ? null : engine.motionPlaybackBlockReason();
        if (isPlaying) {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.85f, 0.50f, 0.15f, 1.0f);
            ImGui.pushStyleColor(ImGuiCol.ButtonHovered, 0.95f, 0.60f, 0.25f, 1.0f);
            if (ImGui.button("|| Pause", 68.0f, 24.0f)) {
                engine.pauseMotion();
            }
            ImGui.popStyleColor(2);
        } else {
            if (hasSequence) {
                ImGui.pushStyleColor(ImGuiCol.Button, 0.18f, 0.68f, 0.35f, 1.0f);
                ImGui.pushStyleColor(ImGuiCol.ButtonHovered, 0.22f, 0.78f, 0.42f, 1.0f);
            }
            ImGui.beginDisabled(playbackBlockReason != null);
            if (ImGui.button("> Play", 68.0f, 24.0f)) {
                try {
                    engine.playMotion();
                    motionStatus = "";
                } catch (IllegalArgumentException error) {
                    motionStatus = error.getMessage();
                    ImGui.openPopup("Motion files");
                }
            }
            ImGui.endDisabled();
            if (hasSequence) {
                ImGui.popStyleColor(2);
            }
        }
        if (ImGui.isItemHovered(ImGuiHoveredFlags.AllowWhenDisabled)) {
            ImGui.setTooltip(isPlaying ? "Pause Motion" : engine.motionPlaybackHint());
        }

        ImGui.sameLine();
        if (ImGui.button("[] Stop", 60.0f, 24.0f)) {
            if (motionPlayer != null) {
                motionPlayer.bindJoints(engine.getJoints());
                motionPlayer.stop();
            }
        }
        if (ImGui.isItemHovered()) {
            ImGui.setTooltip("Stop motion at the current pose");
        }

        ImGui.sameLine();
        boolean loopVal = (motionPlayer != null && motionPlayer.isLooping());
        ImBoolean loopBool = new ImBoolean(loopVal);
        boolean interactionMotion = hasSequence && (motionPlayer.getSequence().isIncomplete()
                || !motionPlayer.getSequence().getEvents().isEmpty());
        ImGui.beginDisabled(interactionMotion || engine.hasActiveMotionTask());
        if (ImGui.checkbox("##loopPlayback", loopBool)) {
            if (motionPlayer != null) {
                motionPlayer.setLooping(loopBool.get());
            }
        }
        ImGui.endDisabled();

        ImGui.sameLine(displayWidth - 620.0f);
        renderMotionFiles(motionPlayer);


        if (hasSequence && motionPlayer.getSequence() != null) {
            MotionSequence seq = motionPlayer.getSequence();
            ImGui.sameLine();
            float duration = Math.max(0.01f, seq.getDurationSeconds());
            float curTime = motionPlayer.getCurrentTime();
            ImFloat timeVal = new ImFloat(curTime);

            ImGui.setNextItemWidth(90.0f);
            ImGui.beginDisabled(interactionMotion || engine.hasActiveMotionTask());
            if (ImGui.sliderFloat("##topTimeline", timeVal.getData(), 0.0f, duration, "%.1fs", ImGuiSliderFlags.None)) {
                motionPlayer.seek(timeVal.get(), engine.getJoints());
            }
            ImGui.endDisabled();
            if (ImGui.isItemHovered()) {
                ImGui.setTooltip(String.format("Motion: %s (%.2fs / %.2fs)", seq.getName(), curTime, duration));
            }
        }


        ImGui.sameLine(displayWidth - 400.0f);

        if (ImGui.button("MCP", 72.0f, 24.0f)) {
            mcpPanel.toggleVisible();
        }
        if (ImGui.isItemHovered()) {
            ImGui.setTooltip("MCP connection setup and server test");
        }

        ImGui.sameLine();
        if (ImGui.button("Scene Objects", 96.0f, 24.0f)) {
            sceneObjectHierarchy.toggleVisible();
        }

        ImGui.sameLine();

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
            ImGui.setTooltip("Sim-to-Real Hardware Bridge");
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
        mcpPanel.render();
        sceneObjectHierarchy.render(rootNode, selectionManager);
    }

    private void renderMotionFiles(MotionPlayer player) {
        if (ImGui.button("Motion", 72.0f, 24.0f)) ImGui.openPopup("Motion files");
        if (!ImGui.beginPopup("Motion files")) return;
        MotionSequence sequence = player.getSequence();
        if (sequence != null) {
            ImGui.textUnformatted(String.format("%s (%.2fs)", sequence.getName(), sequence.getDurationSeconds()));
            if (sequence.isIncomplete()) ImGui.textDisabled("Only completed interactions can be saved.");
        }

        ImGui.beginDisabled(sequence == null || !sequence.hasKeyframes() || sequence.isIncomplete() || engine.hasActiveMotionTask());
        if (ImGui.button("Save Motion")) chooseMotionFile(player, true);
        ImGui.endDisabled();

        ImGui.sameLine();
        ImGui.beginDisabled(player.isPlaying() || engine.hasActiveMotionTask() || player.isReplayingEvents());
        if (ImGui.button("Load Motion")) chooseMotionFile(player, false);
        ImGui.endDisabled();

        ImGui.sameLine();
        String resetBlockReason = engine.robotPoseResetBlockReason();
        ImGui.beginDisabled(resetBlockReason != null);
        if (ImGui.button("Reset Pose") && engine.resetRobotPose()) {
            motionStatus = "Robot reset to its initial pose.";
        }
        ImGui.endDisabled();
        if (ImGui.isItemHovered(ImGuiHoveredFlags.AllowWhenDisabled)) {
            ImGui.setTooltip(resetBlockReason != null ? resetBlockReason : "Return the robot to its scene-load pose");
        }

        if (!motionStatus.isEmpty()) ImGui.textWrapped(motionStatus);
        ImGui.endPopup();
    }

    private void chooseMotionFile(MotionPlayer player, boolean save) {
        try (MemoryStack stack = stackPush()) {
            PointerBuffer filters = stack.mallocPointer(1);
            filters.put(stack.UTF8("*.motion.json")).flip();
            String description = "Motion Files (*.motion.json)";
            String path = save
                    ? TinyFileDialogs.tinyfd_saveFileDialog("Save Motion", "robot.motion.json", filters, description)
                    : TinyFileDialogs.tinyfd_openFileDialog("Load Motion", "", filters, description, false);
            if (path == null || path.isBlank()) return;
            if (save) {
                if (!path.toLowerCase(Locale.ROOT).endsWith(".motion.json")) path += ".motion.json";
                MotionSequenceIO.save(player.getSequence(), Path.of(path));
                motionStatus = "Saved " + Path.of(path).getFileName();
            } else {
                MotionSequence sequence = MotionSequenceIO.load(Path.of(path));
                player.stop();
                if (aiPanel != null) aiPanel.onExternalControl();
                player.loadSequence(sequence);
                motionStatus = "Loaded " + sequence.getName();
            }
        } catch (Exception error) {
            motionStatus = "Motion file could not be " + (save ? "saved: " : "loaded: ") + error.getMessage();
        }
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
