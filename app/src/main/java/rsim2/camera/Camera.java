package rsim2.camera;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import rsim2.input.Input;

import static org.lwjgl.glfw.GLFW.*;

public class Camera {
    private final Vector3f position;
    private final Vector3f rotation;
    
    private final Vector3f orbitTarget = new Vector3f(0, 0, 0);
    private float orbitDistance = 10.0f;

    private final Matrix4f viewMatrix;
    private final Matrix4f projectionMatrix;
    
    private float aspectRatio;
    private static final float FOV = (float) Math.toRadians(60.0f);
    private static final float Z_NEAR = 0.01f;
    private static final float Z_FAR = 1000.f;

    private float mouseSensitivity = 0.2f;
    private float moveSpeed = 5.0f;

    public Camera(float aspectRatio) {
        this.aspectRatio = aspectRatio;
        position = new Vector3f(0, 5, 10);
        rotation = new Vector3f((float) Math.toRadians(30), 0, 0);
        viewMatrix = new Matrix4f();
        projectionMatrix = new Matrix4f();
        updateProjectionMatrix();
    }

    public void setAspectRatio(float aspectRatio) {
        this.aspectRatio = aspectRatio;
        updateProjectionMatrix();
    }

    private void updateProjectionMatrix() {
        projectionMatrix.identity().perspective(FOV, aspectRatio, Z_NEAR, Z_FAR);
    }

    public void update(Input input, float deltaTime) {
        float dx = input.getDeltaMouseX() * mouseSensitivity;
        float dy = input.getDeltaMouseY() * mouseSensitivity;

        if (input.isMouseButtonDown(GLFW_MOUSE_BUTTON_RIGHT)) {
            rotation.y += Math.toRadians(dx);
            rotation.x += Math.toRadians(dy);
            
            rotation.x = Math.max((float) Math.toRadians(-89), Math.min((float) Math.toRadians(89), rotation.x));

            Vector3f forward = new Vector3f(
                (float) (Math.sin(rotation.y) * Math.cos(rotation.x)),
                (float) -Math.sin(rotation.x),
                (float) (-Math.cos(rotation.y) * Math.cos(rotation.x))
            ).normalize();

            Vector3f right = new Vector3f(
                (float) Math.sin(rotation.y - Math.PI / 2.0),
                0,
                (float) -Math.cos(rotation.y - Math.PI / 2.0)
            ).normalize();

            if (input.isKeyDown(GLFW_KEY_W)) position.add(forward.mul(moveSpeed * deltaTime));
            if (input.isKeyDown(GLFW_KEY_S)) position.sub(forward.mul(moveSpeed * deltaTime));
            if (input.isKeyDown(GLFW_KEY_A)) position.sub(right.mul(moveSpeed * deltaTime));
            if (input.isKeyDown(GLFW_KEY_D)) position.add(right.mul(moveSpeed * deltaTime));
            if (input.isKeyDown(GLFW_KEY_SPACE)) position.y += moveSpeed * deltaTime;
            if (input.isKeyDown(GLFW_KEY_LEFT_SHIFT)) position.y -= moveSpeed * deltaTime;
            
        } else if (input.isMouseButtonDown(GLFW_MOUSE_BUTTON_LEFT)) {
            rotation.y += Math.toRadians(dx);
            rotation.x += Math.toRadians(dy);
            
            rotation.x = Math.max((float) Math.toRadians(-89), Math.min((float) Math.toRadians(89), rotation.x));
            
            float horizontalDistance = (float) (orbitDistance * Math.cos(rotation.x));
            float verticalDistance = (float) (orbitDistance * Math.sin(rotation.x));
            
            float xOffset = (float) (horizontalDistance * Math.sin(-rotation.y));
            float zOffset = (float) (horizontalDistance * Math.cos(-rotation.y));
            
            position.set(orbitTarget.x + xOffset, orbitTarget.y + verticalDistance, orbitTarget.z + zOffset);
        }
    }

    public Matrix4f getViewMatrix() {
        viewMatrix.identity();
        viewMatrix.rotateX(rotation.x);
        viewMatrix.rotateY(rotation.y);
        viewMatrix.translate(-position.x, -position.y, -position.z);
        return viewMatrix;
    }

    public Matrix4f getProjectionMatrix() {
        return projectionMatrix;
    }

    public Vector3f getPosition() {
        return position;
    }
}
