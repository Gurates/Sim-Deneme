package rsim2.core;

import imgui.ImGui;
import org.joml.Quaternionf;
import org.joml.Vector3f;
import org.lwjgl.BufferUtils;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.glfw.GLFWImage;
import org.lwjgl.glfw.GLFWNativeWin32;
import org.lwjgl.glfw.GLFWVidMode;
import org.lwjgl.opengl.GL;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.system.windows.User32;
import rsim2.camera.Camera;
import rsim2.collision.CollisionResult;
import rsim2.collision.CollisionWorld;
import rsim2.collision.MotionCollisionState;
import rsim2.data.RobotDefinitionDTO;
import rsim2.editor.Picker;
import rsim2.editor.SelectionManager;
import rsim2.editor.TranslateGizmo;
import rsim2.editor.commands.CommandHistory;
import rsim2.editor.commands.DeleteNodeCommand;
import rsim2.graphics.Renderer;
import rsim2.input.Input;
import rsim2.io.RobotJsonIO;
import rsim2.io.UrdfLoader;
import rsim2.motion.MotionPlayer;
import rsim2.motion.MotionSequence;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;
import rsim2.ui.*;

import javax.imageio.ImageIO;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static org.lwjgl.glfw.Callbacks.glfwFreeCallbacks;
import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.system.MemoryStack.stackPush;
import static org.lwjgl.system.MemoryUtil.NULL;

public class Engine {
    private static volatile Engine instance;

    public static Engine getInstance() {
        return instance;
    }

    public static void setInstance(Engine inst) {
        instance = inst;
    }

    private long window;
    private int width = 1280;
    private int height = 720;
    private String title = "RSim";

    private Input input;
    private Renderer renderer;
    private Camera camera;
    private TranslateGizmo translateGizmo;
    private CommandHistory commandHistory;

    private ImGuiLayer imguiLayer;
    private SelectionManager selectionManager;
    private ToolbarPanel toolbarPanel;
    private HierarchyPanel hierarchyPanel;
    private AllJointsPanel allJointsPanel;
    private InspectorPanel inspectorPanel;
    private AIPanel aiPanel;
    private BridgePanel bridgePanel;
    private CollisionPanel collisionPanel;
    private CollisionWorld collisionWorld;
    private MotionPlayer motionPlayer;
    private final rsim2.mcp.McpBridge mcpBridge = new rsim2.mcp.McpBridge();
    private final rsim2.mcp.McpRobotController mcpRobot = new rsim2.mcp.McpRobotController();

    public rsim2.mcp.McpBridge getMcpBridge() { return mcpBridge; }
    private MotionCollisionState motionCollisionState = new MotionCollisionState();

    private SceneNode rootNode;
    private List<Joint> joints = new ArrayList<>();
    private record NodePose(SceneNode parent, Vector3f position, Quaternionf rotation) {}
    private final Map<SceneNode, NodePose> loadedNodePoses = new IdentityHashMap<>();
    private final Map<Joint, Float> loadedJointAngles = new IdentityHashMap<>();
    private String currentProjectPath = null;

    private long lastAutoSaveTime = 0;
    private String lastAutoSaveMessage = "";

    private int fpsLimit = 60;

    public void run() {
        System.out.println("Starting Engine...");
        init();
        try {
            loop();
        } finally {
            rsim2.bridge.BridgeManager.getInstance().getFirmataBridge().close();
        }

        if (commandHistory != null) {
            commandHistory.clear();
        }

        cleanup();
    }

    private void init() {
        instance = this;
        rsim2.App.initWindowsAppId();
        GLFWErrorCallback.createPrint(System.err).set();

        if (!glfwInit()) {
            throw new IllegalStateException("Unable to initialize GLFW");
        }

        glfwDefaultWindowHints();
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);

        window = glfwCreateWindow(width, height, title, NULL, NULL);
        if (window == NULL) {
            throw new RuntimeException("Failed to create the GLFW window");
        }

        input = new Input();
        input.init(window);

        glfwSetKeyCallback(window, (win, key, scancode, action, mods) -> {
            input.invokeKey(key, action);
        });

        glfwSetFramebufferSizeCallback(window, (win, w, h) -> {
            this.width = w;
            this.height = h;
            glViewport(0, 0, w, h);
            if (camera != null) {
                camera.setAspectRatio((float) w / h);
            }
        });

        try (MemoryStack stack = stackPush()) {
            IntBuffer pWidth = stack.mallocInt(1);
            IntBuffer pHeight = stack.mallocInt(1);

            glfwGetWindowSize(window, pWidth, pHeight);
            GLFWVidMode vidmode = glfwGetVideoMode(glfwGetPrimaryMonitor());

            if (vidmode != null) {
                glfwSetWindowPos(
                        window,
                        (vidmode.width() - pWidth.get(0)) / 2,
                        (vidmode.height() - pHeight.get(0)) / 2);
            }
        }

        glfwMakeContextCurrent(window);
        glfwSwapInterval(0);
        glfwShowWindow(window);
        setWindowIcon();

        GL.createCapabilities();
        glClearColor(0.12f, 0.12f, 0.14f, 1.0f);

        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        imguiLayer = new ImGuiLayer();
        imguiLayer.init(window);

        camera = new Camera((float) width / height);
        renderer = new Renderer();
        translateGizmo = new TranslateGizmo();
        commandHistory = new CommandHistory(this::autoSaveProject);
        translateGizmo.setCommandHistory(commandHistory);

        try {
            renderer.init();
            translateGizmo.init();
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to initialize Renderer/Gizmo", e);
        }

        setupScene();
    }

    private void setupScene() {
        selectionManager = new SelectionManager();
        rootNode = new SceneNode("world");
        joints = new ArrayList<>();
        rememberLoadedPose(null);
        currentProjectPath = null;
        motionPlayer = new MotionPlayer();
        collisionWorld = new CollisionWorld();

        collisionPanel = new CollisionPanel(this, rootNode, joints, selectionManager, collisionWorld);
        aiPanel = new AIPanel(this, rootNode, selectionManager, joints, motionPlayer, collisionWorld);
        bridgePanel = new BridgePanel(motionPlayer, joints);
        rsim2.bridge.BridgeManager.getInstance().registerJoints(joints);
        toolbarPanel = new ToolbarPanel(this, rootNode, selectionManager, commandHistory, aiPanel, bridgePanel, collisionPanel);
        hierarchyPanel = new HierarchyPanel(rootNode, selectionManager);
        allJointsPanel = new AllJointsPanel(this, rootNode, joints, commandHistory);
        inspectorPanel = new InspectorPanel(this, rootNode, selectionManager, joints, commandHistory);
    }

    public void loadProject(String filePath) {
        try {
            System.out.println("Loading project from: " + filePath);
            RobotDefinitionDTO dto = RobotJsonIO.load(filePath);

            File jsonFile = new File(filePath);
            String jsonDir = jsonFile.getParentFile() != null ? jsonFile.getParentFile().getAbsolutePath() : "";

            RobotJsonIO.SceneGraphResult res = RobotJsonIO.toSceneGraph(dto, jsonDir);

            if (this.rootNode != null) {
                this.rootNode.cleanup();
            }

            this.rootNode = res.rootNode != null ? res.rootNode : new SceneNode("world");
            this.joints = res.joints != null ? res.joints : new ArrayList<>();
            rememberLoadedPose(res.rootNode);
            this.currentProjectPath = filePath;

            if (commandHistory != null) {
                commandHistory.clear();
            }

            if (selectionManager != null) {
                selectionManager.clearSelection();
            }
            if (hierarchyPanel != null) {
                hierarchyPanel.setRootNode(this.rootNode);
            }
            if (toolbarPanel != null) {
                toolbarPanel.setRootNode(this.rootNode);
                toolbarPanel.setCommandHistory(this.commandHistory);
            }
            if (allJointsPanel != null) {
                allJointsPanel.setRootNode(this.rootNode);
                allJointsPanel.setJoints(this.joints);
                allJointsPanel.setCommandHistory(this.commandHistory);
            }
            if (inspectorPanel != null) {
                inspectorPanel.setRootNode(this.rootNode);
                inspectorPanel.setJoints(this.joints);
                inspectorPanel.setCommandHistory(this.commandHistory);
            }
            if (collisionPanel != null) {
                collisionPanel.setRootNode(this.rootNode);
                collisionPanel.setJoints(this.joints);
            }
            if (bridgePanel != null) {
                bridgePanel.setJoints(this.joints);
            }
            rsim2.bridge.BridgeManager.getInstance().registerJoints(this.joints);
            if (collisionWorld != null) {
                collisionWorld.getFilter().clearIgnoredPairs();
                collisionWorld.getFilter().computeAutoAcm(this.rootNode, this.joints, collisionWorld);
            }
            if (motionPlayer != null) {
                motionPlayer.stop();
                motionPlayer.loadSequence(null);
            }
            if (aiPanel != null) {
                aiPanel.setRootNode(this.rootNode);
                aiPanel.setJoints(this.joints);
                aiPanel.setMotionPlayer(this.motionPlayer);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void loadUrdf(String filePath) {
        try {
            System.out.println("Loading URDF/XML robot from: " + filePath);
            UrdfLoader.UrdfResult res = UrdfLoader.load(filePath, true);

            if (this.rootNode != null) {
                this.rootNode.cleanup();
            }

            this.rootNode = new SceneNode("world");
            if (res.rootNode != null) {
                this.rootNode.addChild(res.rootNode);
                this.rootNode.setMetersPerSceneUnit(res.rootNode.getMetersPerSceneUnit());
            }
            this.joints = res.joints != null ? res.joints : new ArrayList<>();
            rememberLoadedPose(res.rootNode);

            File urdfFile = new File(filePath);
            String jsonPath = new File(urdfFile.getParentFile(), "robot.json").getAbsolutePath();
            this.currentProjectPath = jsonPath;

            if (commandHistory != null) {
                commandHistory.clear();
            }

            if (selectionManager != null) {
                selectionManager.clearSelection();
                if (res.rootNode != null) {
                    selectionManager.select(res.rootNode);
                }
            }
            if (hierarchyPanel != null) {
                hierarchyPanel.setRootNode(this.rootNode);
            }
            if (toolbarPanel != null) {
                toolbarPanel.setRootNode(this.rootNode);
                toolbarPanel.setCommandHistory(this.commandHistory);
            }
            if (allJointsPanel != null) {
                allJointsPanel.setRootNode(this.rootNode);
                allJointsPanel.setJoints(this.joints);
                allJointsPanel.setCommandHistory(this.commandHistory);
            }
            if (inspectorPanel != null) {
                inspectorPanel.setRootNode(this.rootNode);
                inspectorPanel.setJoints(this.joints);
                inspectorPanel.setCommandHistory(this.commandHistory);
            }
            if (collisionPanel != null) {
                collisionPanel.setRootNode(this.rootNode);
                collisionPanel.setJoints(this.joints);
            }
            if (bridgePanel != null) {
                bridgePanel.setJoints(this.joints);
            }
            rsim2.bridge.BridgeManager.getInstance().registerJoints(this.joints);
            if (collisionWorld != null) {
                collisionWorld.getFilter().clearIgnoredPairs();
                if (res.disabledCollisionPairs != null) {
                    collisionWorld.getFilter().loadDisabledPairs(res.disabledCollisionPairs);
                }
                collisionWorld.getFilter().computeAutoAcm(this.rootNode, this.joints, collisionWorld);
            }
            if (motionPlayer != null) {
                motionPlayer.stop();
                motionPlayer.loadSequence(null);
            }
            if (aiPanel != null) {
                aiPanel.setRootNode(this.rootNode);
                aiPanel.setJoints(this.joints);
                aiPanel.setMotionPlayer(this.motionPlayer);
            }

            saveProject(jsonPath);
            System.out.println("Robot imported successfully and auto-saved to: " + jsonPath);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void saveProject(String filePath) {
        try {
            System.out.println("Saving project to: " + filePath);
            File jsonFile = new File(filePath);
            String jsonDir = jsonFile.getParentFile() != null ? jsonFile.getParentFile().getAbsolutePath() : "";

            RobotDefinitionDTO dto = RobotJsonIO.fromSceneGraph(rootNode, joints, jsonDir);
            RobotJsonIO.save(dto, filePath);
            this.currentProjectPath = filePath;
            this.lastAutoSaveTime = System.currentTimeMillis();
            this.lastAutoSaveMessage = "Saved: " + new File(filePath).getName();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void autoSaveProject() {
        if (currentProjectPath != null && !currentProjectPath.trim().isEmpty()) {
            saveProject(currentProjectPath);
        }
    }

    public String getCurrentProjectPath() {
        return currentProjectPath;
    }

    public MotionPlayer getMotionPlayer() {
        return motionPlayer;
    }

    public boolean hasActiveMotionTask() {
        return mcpRobot.hasActiveTask() || (aiPanel != null && aiPanel.hasActiveInteraction());
    }

    private void rememberLoadedPose(SceneNode robotRoot) {
        loadedNodePoses.clear();
        loadedJointAngles.clear();
        Set<SceneNode> nodes = new HashSet<>();
        collectSubtree(robotRoot, nodes);
        for (SceneNode node : nodes) {
            if (node.isRobotPart()) {
                loadedNodePoses.put(node, new NodePose(node.getParent(), new Vector3f(node.getLocalPosition()),
                        new Quaternionf(node.getLocalRotation())));
            }
        }
        for (Joint joint : joints) {
            // The load-time motor target already accounts for models whose limits exclude zero.
            var motor = joint.getMotor();
            if (motor != null) loadedJointAngles.put(joint, motor.getTargetAngleRadians());
        }
    }

    public String robotPoseResetBlockReason() {
        if (loadedNodePoses.isEmpty()) return "Load a robot to set its initial pose.";
        if (hasActiveMotionTask()) return "Stop the active motion task before resetting the pose.";
        if (!canReplayInteractions()) return "Clear E-stop and disable Sim2Real sync before resetting the pose.";
        return null;
    }

    public boolean resetRobotPose() {
        if (robotPoseResetBlockReason() != null) return false;
        if (aiPanel != null) aiPanel.onExternalControl();
        if (motionPlayer != null) {
            motionPlayer.bindJoints(joints);
            motionPlayer.stop();
        }

        Set<SceneNode> nodes = new HashSet<>();
        collectSubtree(rootNode, nodes);
        for (SceneNode node : nodes) {
            NodePose pose = loadedNodePoses.get(node);
            if (pose != null && node.getParent() == pose.parent()) {
                node.getLocalPosition().set(pose.position());
                node.getLocalRotation().set(pose.rotation());
            }
        }
        for (Joint joint : joints) {
            Float angle = loadedJointAngles.get(joint);
            var motor = joint.getMotor();
            if (angle != null && motor != null) {
                motor.setTargetAngleRadians(angle);
                joint.setAngle(motor.getTargetAngleRadians());
                motor.setCurrentVelocity(0);
            }
        }
        motionCollisionState = new MotionCollisionState();
        motionCollisionState.beginStep(rootNode, joints);
        return true;
    }

    public String motionPlaybackBlockReason() {
        if (motionPlayer == null || !motionPlayer.hasSequence()) return "No motion is loaded.";
        boolean aiBusy = aiPanel != null && aiPanel.hasActiveInteraction();
        if (mcpRobot.hasActiveTask()) {
            if (!aiBusy && mcpRobot.canResumeTask(rootNode, joints, motionPlayer)) return null;
            if (!canReplayInteractions()) return "Clear E-stop and disable Sim2Real sync before resuming.";
            return "An MCP motion is active.";
        }
        MotionSequence sequence = motionPlayer.getSequence();
        if (sequence.isIncomplete() && !aiBusy && !canReplayInteractions()) {
            return "Clear E-stop and disable Sim2Real sync before previewing the motion.";
        }
        if (motionPlayer.restoresObjectsOnReplay() && !canReplayInteractions())
            return "Clear E-stop and disable Sim2Real sync before replaying interactions.";
        return null;
    }

    public String motionPlaybackHint() {
        String blockReason = motionPlaybackBlockReason();
        if (blockReason != null) return blockReason;
        if (motionPlayer.getSequence().isIncomplete() && !hasActiveMotionTask()) {
            String reason = mcpRobot.incompleteMotionReason(motionPlayer);
            String hint = "Play the joint motion in simulation. Grasp and release events are skipped.";
            return reason.isBlank() ? hint : hint + "\nLast attempt: " + reason;
        }
        return "Play / Pause Motion";
    }

    public void pauseMotion() {
        if (motionPlayer == null || mcpRobot.pauseTask(motionPlayer, joints)) return;
        motionPlayer.bindJoints(joints);
        motionPlayer.pause();
    }

    public void playMotion() {
        String reason = motionPlaybackBlockReason();
        if (reason != null) throw new IllegalArgumentException(reason);
        if (mcpRobot.hasActiveTask()) {
            mcpRobot.resumeTask(rootNode, joints, motionPlayer);
            return;
        }
        MotionSequence sequence = motionPlayer.getSequence();
        if (sequence.isIncomplete() && !hasActiveMotionTask()) {
            motionPlayer.playIncompletePreview();
            return;
        }
        if (motionPlayer.restoresObjectsOnReplay()) {
            boolean fromStart = motionPlayer.startsNewPlayback();
            motionPlayer.playInteraction(rootNode, aiPanel.getObjectInteractions());
            if (fromStart) {
                // Replay the recorded scene: objects are back in place, now put the robot at the first frame
                // instead of sweeping it there (with whatever it held) under collision checks.
                motionPlayer.rewindToStart(joints);
                motionCollisionState = new MotionCollisionState();
                motionCollisionState.beginStep(rootNode, joints);
            }
            return;
        }
        motionPlayer.play();
    }

    private boolean canReplayInteractions() {
        var bridge = rsim2.bridge.BridgeManager.getInstance();
        return !bridge.getSafetyFilter().isEmergencyStopActive() && !bridge.isLiveSyncEnabled() && !bridge.isDigitalTwinEnabled();
    }

    public BridgePanel getBridgePanel() {
        return bridgePanel;
    }

    public CollisionWorld getCollisionWorld() {
        return collisionWorld;
    }

    public CollisionPanel getCollisionPanel() {
        return collisionPanel;
    }

    public List<Joint> getJoints() {
        return joints;
    }

    public long getLastAutoSaveTime() {
        return lastAutoSaveTime;
    }

    public String getLastAutoSaveMessage() {
        return lastAutoSaveMessage;
    }

    public SceneNode getRootNode() {
        return rootNode;
    }

    public SelectionManager getSelectionManager() {
        return selectionManager;
    }

    public CommandHistory getCommandHistory() {
        return commandHistory;
    }

    public Renderer getRenderer() {
        return renderer;
    }

    public HierarchyPanel getHierarchyPanel() {
        return hierarchyPanel;
    }

    public void deleteNode(SceneNode node) {
        if (node == null || node.getParent() == null) {
            return;
        }

        if (commandHistory != null) {
            commandHistory.executeAndRecord(new DeleteNodeCommand(node, joints, selectionManager));
        } else {
            Set<SceneNode> nodesToRemove = new HashSet<>();
            collectSubtree(node, nodesToRemove);

            if (joints != null) {
                joints.removeIf(
                        j -> nodesToRemove.contains(j.getChildNode()) || nodesToRemove.contains(j.getParentNode()));
            }

            if (selectionManager != null && nodesToRemove.contains(selectionManager.getSelected())) {
                selectionManager.clearSelection();
            }

            SceneNode parent = node.getParent();
            if (parent != null) {
                parent.removeChild(node);
            }

            node.cleanup();
            autoSaveProject();
        }
    }

    private void collectSubtree(SceneNode current, Set<SceneNode> collected) {
        if (current == null)
            return;
        collected.add(current);
        for (SceneNode child : current.getChildren()) {
            collectSubtree(child, collected);
        }
    }

    private void loop() {
        double lastTime = glfwGetTime();

        while (!glfwWindowShouldClose(window)) {
            double now = glfwGetTime();
            float deltaTime = (float) (now - lastTime);
            lastTime = now;

            input.update();

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            motionCollisionState.beginStep(rootNode, joints);
            if (aiPanel != null) {
                aiPanel.update(deltaTime);
                mcpRobot.useInteractions(rootNode, aiPanel.getObjectInteractions());
            }
            if (!mcpBridge.isEnabled()) mcpRobot.disable(motionPlayer, joints);
            mcpRobot.afterStep(rootNode, joints, motionPlayer, null);
            mcpBridge.drain(request -> mcpRobot.execute(request.get("tool").getAsString(),
                    request.getAsJsonObject("arguments"), rootNode, joints, motionPlayer,
                    collisionWorld == null ? null : collisionWorld.getLastResult(),
                    collisionWorld == null ? null : collisionWorld.getFilter(),
                    selectionManager == null ? null : selectionManager.getSelected(),
                    motionPlayer.isReplayingEvents() || (aiPanel != null && aiPanel.hasActiveInteraction()),
                    () -> {
                        rsim2.mcp.McpBridge.ensureRequestActive();
                        if (aiPanel != null) aiPanel.onExternalControl();
                    }));

            if ((motionPlayer.isReplayingEvents() || motionPlayer.isPreviewingIncompleteMotion())
                    && !canReplayInteractions()) motionPlayer.cancelReplay();
            if (motionPlayer.isPlaying()) {
                motionPlayer.updateFromRenderFrame(mcpRobot.limitDelta(deltaTime, motionPlayer), joints);
            } else if (joints != null) {
                for (Joint joint : joints) {
                    if (joint != null && joint.getMotor() != null) {
                        joint.getMotor().update(deltaTime);
                    }
                }
            }

            String blocked = motionCollisionState.validateAndCommit(rootNode, joints);
            if (blocked != null) {
                motionPlayer.pause();
                motionPlayer.cancelReplay();
                if (aiPanel != null) aiPanel.onMotionBlocked(blocked);
            } else if (aiPanel != null && motionPlayer.isReplayingEvents()) {
                try {
                    motionPlayer.applyReplayEvents(rootNode, joints, aiPanel.getObjectInteractions());
                } catch (IllegalArgumentException error) {
                    motionPlayer.cancelReplay();
                    aiPanel.reportMotionStatus("Playback stopped: " + error.getMessage(), true);
                }
            }
            if (aiPanel != null) {
                for (String event : aiPanel.getObjectInteractions().updateGripping(rootNode, joints)) {
                    aiPanel.reportMotionStatus(event, false);
                    mcpRobot.noteGripEvent(event);
                }
            }
            mcpRobot.afterStep(rootNode, joints, motionPlayer, blocked);

            if (collisionWorld != null) {
                collisionWorld.update(rootNode, joints);
            }
            // Check the current frame's collision state before publishing the validated pose.
            var bridge = rsim2.bridge.BridgeManager.getInstance();
            bridge.getSafetyFilter().setCollisionStopActive(collisionWorld != null && collisionPanel != null
                    && collisionPanel.isStopOnCollision() && collisionWorld.getLastResult().hasCollision());
            bridge.streamCurrentJointStates(joints, motionPlayer != null ? motionPlayer.getCurrentTime() : 0.0f);

            SceneNode gizmoTarget = (selectionManager != null) ? selectionManager.getSelected() : null;
            if (gizmoTarget != null && gizmoTarget.getParent() == null) {
                gizmoTarget = null;
            }

            if (translateGizmo != null) {
                translateGizmo.setTargetNode(gizmoTarget);
            }

            imguiLayer.newFrame();

            boolean wantCaptureKeyboard = false;
            try {
                wantCaptureKeyboard = ImGui.getIO().getWantCaptureKeyboard();
            } catch (Throwable ignored) {
            }

            if (!wantCaptureKeyboard) {
                boolean isCtrlDown = input.isKeyDown(GLFW_KEY_LEFT_CONTROL)
                        || input.isKeyDown(GLFW_KEY_RIGHT_CONTROL);
                boolean isShiftDown = input.isKeyDown(GLFW_KEY_LEFT_SHIFT) || input.isKeyDown(GLFW_KEY_RIGHT_SHIFT);

                if (isCtrlDown && !isShiftDown && input.isKeyPressed(GLFW_KEY_Z)) {
                    if (commandHistory != null) {
                        commandHistory.undo();
                    }
                } else if ((isCtrlDown && input.isKeyPressed(GLFW_KEY_Y))
                        || (isCtrlDown && isShiftDown && input.isKeyPressed(GLFW_KEY_Z))) {
                    if (commandHistory != null) {
                        commandHistory.redo();
                    }
                }

                if (input.isKeyPressed(GLFW_KEY_DELETE)) {
                    if (selectionManager != null) {
                        SceneNode sel = selectionManager.getSelected();
                        if (sel != null && sel.getParent() != null) {
                            deleteNode(sel);
                        }
                    }
                }
            }

            boolean wantCaptureMouse = false;
            try {
                wantCaptureMouse = ImGui.getIO().getWantCaptureMouse();
            } catch (Throwable ignored) {
            }

            if (!wantCaptureMouse) {
                if (input.isMouseButtonClicked(GLFW_MOUSE_BUTTON_LEFT)) {
                    boolean pickedGizmo = false;
                    if (translateGizmo != null) {
                        pickedGizmo = translateGizmo.onMouseDown(input.getMouseX(), input.getMouseY(), camera,
                                width, height);
                    }

                    if (!pickedGizmo) {
                        SceneNode picked = Picker.pick(input.getMouseX(), input.getMouseY(), width, height, camera,
                                rootNode);
                        if (picked != null && selectionManager != null) {
                            selectionManager.select(picked);
                        }
                    }
                }

                if (input.isMouseButtonDown(GLFW_MOUSE_BUTTON_LEFT)) {
                    if (translateGizmo != null && translateGizmo.isDragging()) {
                        translateGizmo.onMouseDrag(input.getMouseX(), input.getMouseY(), camera, width, height);
                    }
                }
            }

            if (!input.isMouseButtonDown(GLFW_MOUSE_BUTTON_LEFT)) {
                if (translateGizmo != null && translateGizmo.isDragging()) {
                    translateGizmo.onMouseUp();
                }
            }

            if (translateGizmo == null || !translateGizmo.isDragging()) {
                camera.update(input, deltaTime);
            }

            Set<SceneNode> collidingNodes = (collisionWorld != null) ? collisionWorld.getLastResult().getCollidingNodes() : null;
            Map<SceneNode, rsim2.collision.OBB> debugOBBs = (collisionWorld != null && collisionWorld.isDebugWireframesEnabled())
                    ? collisionWorld.getWorldOBBs() : null;
            renderer.render(camera, rootNode, (selectionManager != null) ? selectionManager.getSelected() : null, collidingNodes, debugOBBs);

            if (translateGizmo != null) {
                translateGizmo.render(camera, width, height);
            }

            if (toolbarPanel != null) {
                toolbarPanel.render();
            }
            if (hierarchyPanel != null) {
                hierarchyPanel.render();
            }
            if (allJointsPanel != null) {
                allJointsPanel.render();
            }
            if (inspectorPanel != null) {
                inspectorPanel.render();
            }
            if (aiPanel != null) {
                aiPanel.render();
            }
            if (bridgePanel != null) {
                bridgePanel.render();
            }
            if (collisionPanel != null) {
                collisionPanel.render();
            }

            imguiLayer.render();

            glfwSwapBuffers(window);
            glfwPollEvents();

            if (fpsLimit > 0) {
                double targetFrameTime = 1.0 / fpsLimit;

                while (true) {
                    double elapsed = (System.nanoTime() - now) / 1_000_000_000.0;
                    double remaining = targetFrameTime - elapsed;
                    if (remaining <= 0) {
                        break;
                    }

                    if (remaining > 0.002) {
                        try {
                            Thread.sleep(1);
                        } catch (InterruptedException ignored) {
                        }
                    } else {
                        Thread.onSpinWait();
                    }
                }
            }
        }

        if (translateGizmo != null) {
            translateGizmo.cleanup();
        }
        if (renderer != null) {
            renderer.cleanup();
        }
        if (rootNode != null) {
            rootNode.cleanup();
        }
    }

    private void cleanup() {
        mcpBridge.close();
        imguiLayer.dispose();

        glfwFreeCallbacks(window);
        glfwDestroyWindow(window);

        glfwTerminate();
        GLFWErrorCallback callback = glfwSetErrorCallback(null);
        if (callback != null) {
            callback.free();
        }
        if (instance == this) {
            instance = null;
        }
    }

    private void setWindowIcon() {
        try {
            BufferedImage image = null;

            String[] resourcePaths = {
                "/icon/logo.jpg",
                "/icon/logo.png",
                "/icons/logo.jpg",
                "/icons/logo.png",
                "/logo.jpg",
                "/logo.png"
            };

            for (String resPath : resourcePaths) {
                InputStream is = getClass().getResourceAsStream(resPath);
                if (is != null) {
                    try (InputStream in = is) {
                        image = ImageIO.read(in);
                        if (image != null) break;
                    }
                }
            }

            if (image == null) {
                File[] fileCandidates = {
                    new File("icon/logo.jpg"),
                    new File("icon/logo.png"),
                    new File("app/src/main/resources/icon/logo.jpg"),
                    new File("app/src/main/resources/icon/logo.png"),
                    new File("../icon/logo.jpg"),
                    new File("logo.jpg"),
                    new File("logo.png")
                };

                for (File file : fileCandidates) {
                    if (file.exists() && file.isFile()) {
                        image = ImageIO.read(file);
                        if (image != null) break;
                    }
                }
            }

            if (image == null) {
                System.err.println("[Engine] Window icon could not be found.");
                return;
            }

            int[] iconSizes = {16, 24, 32, 48, 64, 128, 256};
            GLFWImage.Buffer iconBuffer = GLFWImage.malloc(iconSizes.length);

            for (int i = 0; i < iconSizes.length; i++) {
                int size = iconSizes[i];
                BufferedImage resized = resizeImage(image, size, size);
                ByteBuffer pixels = convertImageData(resized);
                iconBuffer.get(i).set(size, size, pixels);
            }

            glfwSetWindowIcon(window, iconBuffer);
            iconBuffer.free();

            String os = System.getProperty("os.name").toLowerCase();
            if (os.contains("win")) {
                try {
                    long hwnd = GLFWNativeWin32.glfwGetWin32Window(window);
                    if (hwnd != 0) {
                        long hIconBig = User32.DefWindowProc(hwnd, 0x007F, 1, 0);
                        long hIconSmall = User32.DefWindowProc(hwnd, 0x007F, 0, 0);
                        if (hIconBig != 0) {
                            User32.SetClassLongPtr(hwnd, -14, hIconBig);
                            User32.SendMessage(hwnd, 0x0080, 1L, hIconBig);
                        }
                        if (hIconSmall != 0) {
                            User32.SetClassLongPtr(hwnd, -34, hIconSmall);
                            User32.SendMessage(hwnd, 0x0080, 0L, hIconSmall);
                        }
                    }
                } catch (Throwable t) {
                    System.err.println("[Engine] Could not update Win32 class icons: " + t.getMessage());
                }
            }

            System.out.println("[Engine] Window icon set successfully.");
        } catch (Exception e) {
            System.err.println("[Engine] Failed to set window icon: " + e.getMessage());
            e.printStackTrace();
        }
    }

    private BufferedImage resizeImage(BufferedImage original, int targetWidth, int targetHeight) {
        BufferedImage resized = new BufferedImage(targetWidth, targetHeight, BufferedImage.TYPE_INT_ARGB);
        Graphics2D g2d = resized.createGraphics();
        g2d.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
        g2d.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2d.drawImage(original, 0, 0, targetWidth, targetHeight, null);
        g2d.dispose();
        return resized;
    }

    private ByteBuffer convertImageData(BufferedImage image) {
        int width = image.getWidth();
        int height = image.getHeight();
        int[] pixels = new int[width * height];
        image.getRGB(0, 0, width, height, pixels, 0, width);

        ByteBuffer buffer = BufferUtils.createByteBuffer(width * height * 4);
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int pixel = pixels[y * width + x];
                buffer.put((byte) ((pixel >> 16) & 0xFF));
                buffer.put((byte) ((pixel >> 8) & 0xFF));
                buffer.put((byte) (pixel & 0xFF));
                buffer.put((byte) ((pixel >> 24) & 0xFF));
            }
        }
        buffer.flip();
        return buffer;
    }
}
