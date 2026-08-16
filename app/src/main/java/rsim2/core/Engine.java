package rsim2.core;

import imgui.ImGui;
import org.joml.Vector3f;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.glfw.GLFWVidMode;
import org.lwjgl.opengl.GL;
import org.lwjgl.system.MemoryStack;
import rsim2.camera.Camera;
import rsim2.data.RobotDefinitionDTO;
import rsim2.editor.Picker;
import rsim2.editor.SelectionManager;
import rsim2.editor.TranslateGizmo;
import rsim2.graphics.Renderer;
import rsim2.input.Input;
import rsim2.io.RobotJsonIO;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;
import rsim2.ui.*;

import java.io.File;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
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

    private ImGuiLayer imguiLayer;
    private SelectionManager selectionManager;
    private ToolbarPanel toolbarPanel;
    private HierarchyPanel hierarchyPanel;
    private InspectorPanel inspectorPanel;
    private JointToolPanel jointToolPanel;

    private SceneNode rootNode;
    private List<Joint> joints = new ArrayList<>();
    private String currentProjectPath = null;
    private long lastAutoSaveTime = 0;
    private String lastAutoSaveMessage = "";

    public void run() {
        System.out.println("Starting Engine...");
        init();
        loop();

        if (imguiLayer != null) {
            imguiLayer.dispose();
        }

        glfwFreeCallbacks(window);
        glfwDestroyWindow(window);

        glfwTerminate();
        glfwSetErrorCallback(null).free();
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
            if (key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE) {
                glfwSetWindowShouldClose(win, true);
            }
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
                try {
                    glfwSetWindowPos(
                            window,
                            (vidmode.width() - pWidth.get(0)) / 2,
                            (vidmode.height() - pHeight.get(0)) / 2);
                } catch (Throwable ignored) {
                }
            }
        }

        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
        glfwShowWindow(window);

        GL.createCapabilities();

        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        imguiLayer = new ImGuiLayer();
        imguiLayer.init(window);

        camera = new Camera((float) width / height);
        renderer = new Renderer();
        translateGizmo = new TranslateGizmo();

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

        toolbarPanel = new ToolbarPanel(this, rootNode, selectionManager);
        hierarchyPanel = new HierarchyPanel(rootNode, selectionManager);
        inspectorPanel = new InspectorPanel(this, selectionManager, joints);
        jointToolPanel = new JointToolPanel(this, rootNode, joints, this::autoSaveProject);
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

            if (selectionManager != null) {
                selectionManager.clearSelection();
            }
            if (hierarchyPanel != null) {
                hierarchyPanel.setRootNode(this.rootNode);
            }
            if (toolbarPanel != null) {
                toolbarPanel.setRootNode(this.rootNode);
            }
            if (inspectorPanel != null) {
                inspectorPanel.setJoints(this.joints);
            }
            if (jointToolPanel != null) {
                jointToolPanel.setRootNode(this.rootNode);
                jointToolPanel.setJoints(this.joints);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void saveProject(String filePath) {
        try {
            System.out.println("Saving project to: " + filePath);
            File jsonFile = new File(filePath);
            String jsonDir = jsonFile.getParentFile() != null ? jsonFile.getParentFile().getAbsolutePath() : "";

            RobotDefinitionDTO dto = RobotJsonIO.fromSceneGraph(rootNode, joints, "RSimRobot", jsonDir);
            RobotJsonIO.save(dto, filePath);
            this.currentProjectPath = filePath;
            this.lastAutoSaveTime = System.currentTimeMillis();
            this.lastAutoSaveMessage = "Kaydedildi";
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void autoSaveProject() {
        if (currentProjectPath != null && !currentProjectPath.trim().isEmpty()) {
            try {
                System.out.println("Auto-saving project to: " + currentProjectPath);
                File jsonFile = new File(currentProjectPath);
                String jsonDir = jsonFile.getParentFile() != null ? jsonFile.getParentFile().getAbsolutePath() : "";

                RobotDefinitionDTO dto = RobotJsonIO.fromSceneGraph(rootNode, joints, "RSimRobot", jsonDir);
                RobotJsonIO.save(dto, currentProjectPath);
                this.lastAutoSaveTime = System.currentTimeMillis();
                this.lastAutoSaveMessage = "Otomatik kaydedildi";
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public void deleteNode(SceneNode node) {
        if (node == null || node.getParent() == null) {
            return;
        }

        Set<SceneNode> nodesToRemove = new HashSet<>();
        collectSubtree(node, nodesToRemove);

        if (joints != null) {
            joints.removeIf(j -> nodesToRemove.contains(j.getChildNode()) || nodesToRemove.contains(j.getParentNode()));
        }

        if (selectionManager != null && nodesToRemove.contains(selectionManager.getSelected())) {
            selectionManager.clearSelection();
        }

        node.getParent().removeChild(node);
        node.cleanup();
    }

    private void collectSubtree(SceneNode current, Set<SceneNode> collected) {
        if (current == null) return;
        collected.add(current);
        for (SceneNode child : current.getChildren()) {
            collectSubtree(child, collected);
        }
    }

    public String getCurrentProjectPath() {
        return currentProjectPath;
    }

    public void setCurrentProjectPath(String currentProjectPath) {
        this.currentProjectPath = currentProjectPath;
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

    public List<Joint> getJoints() {
        return joints;
    }

    private void loop() {
        glClearColor(0.85f, 0.92f, 0.98f, 1.0f);

        long lastTime = System.nanoTime();

        while (!glfwWindowShouldClose(window)) {
            long now = System.nanoTime();
            float deltaTime = (now - lastTime) / 1_000_000_000.0f;
            lastTime = now;

            if (joints != null) {
                for (Joint j : joints) {
                    if (j.getMotor() != null) {
                        j.getMotor().update(deltaTime);
                    }
                }
            }

            imguiLayer.newFrame();

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            input.update();

            SceneNode gizmoTarget = null;
            if (jointToolPanel != null && jointToolPanel.getActivePreviewChild() != null) {
                gizmoTarget = jointToolPanel.getActivePreviewChild();
            } else if (selectionManager != null) {
                gizmoTarget = selectionManager.getSelected();
            }
            if (translateGizmo != null) {
                translateGizmo.setTargetNode(gizmoTarget);
            }

            if (input.isKeyPressed(GLFW_KEY_DELETE)) {
                SceneNode selected = selectionManager != null ? selectionManager.getSelected() : null;
                if (selected != null) {
                    deleteNode(selected);
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
                        pickedGizmo = translateGizmo.onMouseDown(input.getMouseX(), input.getMouseY(), camera, width, height);
                    }

                    if (!pickedGizmo) {
                        SceneNode picked = Picker.pick(input.getMouseX(), input.getMouseY(), width, height, camera, rootNode);
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

            renderer.render(camera, rootNode);

            if (translateGizmo != null) {
                translateGizmo.render(camera, width, height);
            }

            if (toolbarPanel != null) {
                toolbarPanel.render();
            }
            if (hierarchyPanel != null) {
                hierarchyPanel.render();
            }
            if (inspectorPanel != null) {
                inspectorPanel.render();
            }
            if (jointToolPanel != null) {
                jointToolPanel.render();
            }
            imguiLayer.render();

            glfwSwapBuffers(window);
            glfwPollEvents();
        }

        if (translateGizmo != null) {
            translateGizmo.cleanup();
        }
        renderer.cleanup();
    }
}
