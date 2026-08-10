package rsim2.input;

import static org.lwjgl.glfw.GLFW.*;

public class Input {
    private final boolean[] keys = new boolean[GLFW_KEY_LAST];
    private final boolean[] mouseButtons = new boolean[GLFW_MOUSE_BUTTON_LAST];
    
    private double mouseX;
    private double mouseY;
    private double lastMouseX;
    private double lastMouseY;
    private float deltaMouseX;
    private float deltaMouseY;

    public void init(long window) {
        glfwSetMouseButtonCallback(window, (windowHandle, button, action, mods) -> {
            if (button < GLFW_MOUSE_BUTTON_LAST) {
                mouseButtons[button] = action != GLFW_RELEASE;
            }
        });

        glfwSetCursorPosCallback(window, (windowHandle, xpos, ypos) -> {
            lastMouseX = mouseX;
            lastMouseY = mouseY;
            mouseX = xpos;
            mouseY = ypos;
        });
    }
    
    public void invokeKey(int key, int action) {
        if (key >= 0 && key < GLFW_KEY_LAST) {
            keys[key] = action != GLFW_RELEASE;
        }
    }

    public void update() {
        deltaMouseX = (float) (mouseX - lastMouseX);
        deltaMouseY = (float) (mouseY - lastMouseY);
        
        lastMouseX = mouseX;
        lastMouseY = mouseY;
    }

    public boolean isKeyDown(int key) {
        return keys[key];
    }

    public boolean isMouseButtonDown(int button) {
        return mouseButtons[button];
    }

    public float getDeltaMouseX() {
        return deltaMouseX;
    }

    public float getDeltaMouseY() {
        return deltaMouseY;
    }
}
