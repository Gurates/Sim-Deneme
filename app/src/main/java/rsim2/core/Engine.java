package rsim2.core;

import org.joml.Vector3f;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.glfw.GLFWVidMode;
import org.lwjgl.opengl.GL;
import org.lwjgl.system.MemoryStack;
import rsim2.camera.Camera;
import rsim2.graphics.Mesh;
import rsim2.graphics.Renderer;
import rsim2.input.Input;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;
import rsim2.ui.ImGuiLayer;
import rsim2.ui.InspectorPanel;

import java.nio.IntBuffer;

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

    private ImGuiLayer imguiLayer;
    private InspectorPanel inspectorPanel;

    private SceneNode rootNode;
    private Joint testJoint;
    private float totalTime = 0.0f;

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

        try {
            renderer.init();
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to initialize Renderer", e);
        }

        setupTestScene();
    }

    private void setupTestScene() {
        Mesh cubeMesh = renderer.getTestMesh();

        SceneNode baseNode = new SceneNode("base", cubeMesh);
        baseNode.getLocalPosition().set(0.0f, 0.5f, 0.0f);

        SceneNode armNode = new SceneNode("arm", cubeMesh);
        armNode.getLocalPosition().set(0.0f, 1.5f, 0.0f);

        baseNode.addChild(armNode);

        testJoint = new Joint("joint1", baseNode, armNode, new Vector3f(0.0f, 1.0f, 0.0f));
        rootNode = baseNode;

        inspectorPanel = new InspectorPanel(baseNode);
    }

    private void loop() {
        glClearColor(0.85f, 0.92f, 0.98f, 1.0f);

        long lastTime = System.nanoTime();

        while (!glfwWindowShouldClose(window)) {
            long now = System.nanoTime();
            float deltaTime = (now - lastTime) / 1_000_000_000.0f;
            lastTime = now;

            totalTime += deltaTime;

            if (testJoint != null) {
                float angle = (float) (Math.sin(totalTime * 2.0) * Math.PI / 2.0);
                testJoint.setAngle(angle);
            }

            imguiLayer.newFrame();

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            input.update();
            camera.update(input, deltaTime);

            renderer.render(camera, rootNode);

            if (inspectorPanel != null) {
                inspectorPanel.render();
            }
            imguiLayer.render();

            glfwSwapBuffers(window);
            glfwPollEvents();
        }

        renderer.cleanup();
    }
}
