package rsim2.core;

import imgui.ImGui;
import org.joml.Vector3f;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.glfw.GLFWVidMode;
import org.lwjgl.opengl.GL;
import org.lwjgl.system.MemoryStack;
import rsim2.camera.Camera;
import rsim2.collision.CollisionResult;
import rsim2.collision.CollisionWorld;
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
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;
import rsim2.ui.*;

import java.io.File;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static org.lwjgl.glfw.Callbacks.glfwFreeCallbacks;
import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.system.MemoryStack.stackPush;
import static org.lwjgl.system.MemoryUtil.NULL;

public class Engine {
    private long window;
    private int width = 1280;
    private int height = 720;
    private String title = "RSim2";

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

    private SceneNode rootNode;
    private List<Joint> joints = new ArrayList<>();
    private String currentProjectPath = null;

    private long lastAutoSaveTime = 0;
    private String lastAutoSaveMessage = "";

    private int fpsLimit = 60;

    public void run() {
        System.out.println("Starting Engine...");
        init();
        loop();

        if (commandHistory != null) {
            commandHistory.clear();
        }

        cleanup();
    }

    private void init() {
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
        currentProjectPath = null;
        motionPlayer = new MotionPlayer();
        collisionWorld = new CollisionWorld();

        collisionPanel = new CollisionPanel(this, rootNode, joints, selectionManager, collisionWorld);
        aiPanel = new AIPanel(this, rootNode, selectionManager, joints, motionPlayer);
        bridgePanel = new BridgePanel(motionPlayer);
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
            if (motionPlayer != null) {
                motionPlayer.stop();
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
            }
            this.joints = res.joints != null ? res.joints : new ArrayList<>();

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
            if (motionPlayer != null) {
                motionPlayer.stop();
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

            if (motionPlayer != null && motionPlayer.isPlaying()) {
                motionPlayer.update(deltaTime, joints);
                rsim2.bridge.BridgeManager.getInstance().streamCurrentJointStates(joints, motionPlayer.getCurrentTime());
            } else if (joints != null) {
                for (Joint joint : joints) {
                    if (joint != null && joint.getMotor() != null) {
                        joint.getMotor().update(deltaTime);
                    }
                }
                rsim2.bridge.BridgeManager.getInstance().streamCurrentJointStates(joints, 0.0f);
            }

            if (collisionWorld != null) {
                collisionWorld.update(rootNode, joints);
                if (collisionPanel != null && collisionPanel.isStopOnCollision()) {
                    rsim2.bridge.BridgeManager.getInstance().getSafetyFilter().setCollisionStopActive(
                            collisionWorld.getLastResult().hasCollision()
                    );
                }
            }

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
        imguiLayer.dispose();

        glfwFreeCallbacks(window);
        glfwDestroyWindow(window);

        glfwTerminate();
        GLFWErrorCallback callback = glfwSetErrorCallback(null);
        if (callback != null) {
            callback.free();
        }
    }
}
