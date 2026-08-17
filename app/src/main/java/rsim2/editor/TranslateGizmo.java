package rsim2.editor;

import org.joml.Matrix4f;
import org.joml.Vector2f;
import org.joml.Vector3f;
import org.lwjgl.BufferUtils;
import rsim2.camera.Camera;
import rsim2.graphics.ShaderProgram;
import rsim2.scene.SceneNode;

import java.nio.FloatBuffer;

import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.opengl.GL15.*;
import static org.lwjgl.opengl.GL20.*;
import static org.lwjgl.opengl.GL30.*;

public class TranslateGizmo {
    public static final float GIZMO_LENGTH = 1.5f;
    public static final float GIZMO_PICK_THRESHOLD_PX = 15.0f;
    public static final float GRID_SNAP_SIZE = 0.05f;

    private SceneNode targetNode;
    private boolean isDragging = false;
    private int activeAxis = -1;
    private final Vector2f dragStartMouse = new Vector2f();
    private float dragStartValue = 0.0f;

    private ShaderProgram gizmoShader;
    private int projLoc;
    private int viewLoc;
    private int modelLoc;
    private int colorLoc;

    private int vao;
    private int vbo;

    private int markerVao;
    private int markerVbo;

    private final Matrix4f modelMatrix = new Matrix4f();

    private final float[] lineVertices = new float[]{
            0.0f, 0.0f, 0.0f,
            GIZMO_LENGTH, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, GIZMO_LENGTH, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, GIZMO_LENGTH
    };

    public void init() throws Exception {
        gizmoShader = new ShaderProgram();
        gizmoShader.createVertexShader("/shaders/gizmo.vert");
        gizmoShader.createFragmentShader("/shaders/gizmo.frag");
        gizmoShader.link();

        projLoc = gizmoShader.getUniformLocation("projectionMatrix");
        viewLoc = gizmoShader.getUniformLocation("viewMatrix");
        modelLoc = gizmoShader.getUniformLocation("modelMatrix");
        colorLoc = gizmoShader.getUniformLocation("color");

        vao = glGenVertexArrays();
        glBindVertexArray(vao);

        FloatBuffer buffer = BufferUtils.createFloatBuffer(lineVertices.length);
        buffer.put(lineVertices).flip();

        vbo = glGenBuffers();
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, buffer, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, false, 3 * Float.BYTES, 0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        setupMarkerMesh();
    }

    private void setupMarkerMesh() {
        float s = 0.08f;
        float[] markerVerts = new float[]{
                -s, 0.0f, 0.0f,  s, 0.0f, 0.0f,
                0.0f, -s, 0.0f,  0.0f, s, 0.0f,
                0.0f, 0.0f, -s,  0.0f, 0.0f, s
        };
        markerVao = glGenVertexArrays();
        glBindVertexArray(markerVao);

        FloatBuffer buffer = BufferUtils.createFloatBuffer(markerVerts.length);
        buffer.put(markerVerts).flip();

        markerVbo = glGenBuffers();
        glBindBuffer(GL_ARRAY_BUFFER, markerVbo);
        glBufferData(GL_ARRAY_BUFFER, buffer, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, false, 3 * Float.BYTES, 0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    public void setTargetNode(SceneNode targetNode) {
        this.targetNode = targetNode;
        if (targetNode == null) {
            isDragging = false;
            activeAxis = -1;
        }
    }

    public SceneNode getTargetNode() {
        return targetNode;
    }

    public boolean isDragging() {
        return isDragging;
    }

    public int getActiveAxis() {
        return activeAxis;
    }

    public void render(Camera camera, int screenWidth, int screenHeight) {
        if (targetNode == null || gizmoShader == null) {
            return;
        }

        Vector3f worldPos = new Vector3f();
        targetNode.getWorldTransform().getTranslation(worldPos);

        modelMatrix.identity().translation(worldPos);

        glDisable(GL_DEPTH_TEST);
        glLineWidth(3.5f);

        gizmoShader.bind();

        FloatBuffer projBuf = BufferUtils.createFloatBuffer(16);
        camera.getProjectionMatrix().get(projBuf);
        glUniformMatrix4fv(projLoc, false, projBuf);

        FloatBuffer viewBuf = BufferUtils.createFloatBuffer(16);
        camera.getViewMatrix().get(viewBuf);
        glUniformMatrix4fv(viewLoc, false, viewBuf);

        FloatBuffer modelBuf = BufferUtils.createFloatBuffer(16);
        modelMatrix.get(modelBuf);
        glUniformMatrix4fv(modelLoc, false, modelBuf);

        glBindVertexArray(vao);

        if (activeAxis == 0) {
            glUniform4f(colorLoc, 1.0f, 1.0f, 0.2f, 1.0f);
        } else {
            glUniform4f(colorLoc, 1.0f, 0.2f, 0.2f, 1.0f);
        }
        glDrawArrays(GL_LINES, 0, 2);

        if (activeAxis == 1) {
            glUniform4f(colorLoc, 1.0f, 1.0f, 0.2f, 1.0f);
        } else {
            glUniform4f(colorLoc, 0.2f, 1.0f, 0.2f, 1.0f);
        }
        glDrawArrays(GL_LINES, 2, 2);

        if (activeAxis == 2) {
            glUniform4f(colorLoc, 1.0f, 1.0f, 0.2f, 1.0f);
        } else {
            glUniform4f(colorLoc, 0.2f, 0.4f, 1.0f, 1.0f);
        }
        glDrawArrays(GL_LINES, 4, 2);

        glBindVertexArray(0);
        gizmoShader.unbind();

        glLineWidth(1.0f);
        glEnable(GL_DEPTH_TEST);
    }

    public void renderPointMarker(Vector3f worldPos, Camera camera, float r, float g, float b) {
        if (worldPos == null || gizmoShader == null) {
            return;
        }

        modelMatrix.identity().translation(worldPos);

        glDisable(GL_DEPTH_TEST);
        glLineWidth(3.0f);

        gizmoShader.bind();

        FloatBuffer projBuf = BufferUtils.createFloatBuffer(16);
        camera.getProjectionMatrix().get(projBuf);
        glUniformMatrix4fv(projLoc, false, projBuf);

        FloatBuffer viewBuf = BufferUtils.createFloatBuffer(16);
        camera.getViewMatrix().get(viewBuf);
        glUniformMatrix4fv(viewLoc, false, viewBuf);

        FloatBuffer modelBuf = BufferUtils.createFloatBuffer(16);
        modelMatrix.get(modelBuf);
        glUniformMatrix4fv(modelLoc, false, modelBuf);

        glUniform4f(colorLoc, r, g, b, 1.0f);

        glBindVertexArray(markerVao);
        glDrawArrays(GL_LINES, 0, 6);
        glBindVertexArray(0);

        gizmoShader.unbind();

        glLineWidth(1.0f);
        glEnable(GL_DEPTH_TEST);
    }

    public int pickAxis(float mouseX, float mouseY, Camera camera, int screenWidth, int screenHeight) {
        if (targetNode == null) {
            return -1;
        }

        Vector3f origin = new Vector3f();
        targetNode.getWorldTransform().getTranslation(origin);

        Vector2f originScreen = projectToScreen(origin, camera, screenWidth, screenHeight);
        if (originScreen == null) {
            return -1;
        }

        Vector3f[] axes = new Vector3f[]{
                new Vector3f(1.0f, 0.0f, 0.0f),
                new Vector3f(0.0f, 1.0f, 0.0f),
                new Vector3f(0.0f, 0.0f, 1.0f)
        };

        int closestAxis = -1;
        float minDistance = GIZMO_PICK_THRESHOLD_PX;

        for (int i = 0; i < 3; i++) {
            Vector3f endWorld = new Vector3f(origin).add(new Vector3f(axes[i]).mul(GIZMO_LENGTH));
            Vector2f endScreen = projectToScreen(endWorld, camera, screenWidth, screenHeight);
            if (endScreen == null) {
                continue;
            }

            float dist = pointToSegmentDistance(mouseX, mouseY, originScreen.x, originScreen.y, endScreen.x, endScreen.y);
            if (dist < minDistance) {
                minDistance = dist;
                closestAxis = i;
            }
        }

        return closestAxis;
    }

    public boolean onMouseDown(float mouseX, float mouseY, Camera camera, int screenWidth, int screenHeight) {
        int picked = pickAxis(mouseX, mouseY, camera, screenWidth, screenHeight);
        if (picked != -1 && targetNode != null) {
            isDragging = true;
            activeAxis = picked;
            dragStartMouse.set(mouseX, mouseY);

            Vector3f pos = targetNode.getLocalPosition();
            if (activeAxis == 0) dragStartValue = pos.x;
            else if (activeAxis == 1) dragStartValue = pos.y;
            else if (activeAxis == 2) dragStartValue = pos.z;

            return true;
        }
        return false;
    }

    public void onMouseDrag(float mouseX, float mouseY, Camera camera, int screenWidth, int screenHeight) {
        if (!isDragging || targetNode == null || activeAxis < 0 || activeAxis > 2) {
            return;
        }

        Vector3f origin = new Vector3f();
        targetNode.getWorldTransform().getTranslation(origin);

        Vector3f axisDir = new Vector3f();
        if (activeAxis == 0) axisDir.set(1.0f, 0.0f, 0.0f);
        else if (activeAxis == 1) axisDir.set(0.0f, 1.0f, 0.0f);
        else if (activeAxis == 2) axisDir.set(0.0f, 0.0f, 1.0f);

        Vector2f p0 = projectToScreen(origin, camera, screenWidth, screenHeight);
        Vector2f p1 = projectToScreen(new Vector3f(origin).add(axisDir), camera, screenWidth, screenHeight);

        if (p0 == null || p1 == null) {
            return;
        }

        Vector2f screenDir = new Vector2f(p1.x - p0.x, p1.y - p0.y);
        float len = screenDir.length();
        if (len < 0.001f) {
            return;
        }
        screenDir.div(len);

        Vector2f mouseDelta = new Vector2f(mouseX - dragStartMouse.x, mouseY - dragStartMouse.y);
        float projectedPixels = mouseDelta.dot(screenDir);

        float worldDelta = projectedPixels / len;
        float rawValue = dragStartValue + worldDelta;
        float snappedValue = Math.round(rawValue / GRID_SNAP_SIZE) * GRID_SNAP_SIZE;

        Vector3f localPos = targetNode.getLocalPosition();
        if (activeAxis == 0) localPos.x = snappedValue;
        else if (activeAxis == 1) localPos.y = snappedValue;
        else if (activeAxis == 2) localPos.z = snappedValue;
    }

    public void onMouseUp() {
        isDragging = false;
        activeAxis = -1;
    }

    private Vector2f projectToScreen(Vector3f worldPos, Camera camera, int screenWidth, int screenHeight) {
        Vector3f viewSpacePos = new Vector3f();
        camera.getViewMatrix().transformPosition(new Vector3f(worldPos), viewSpacePos);
        if (viewSpacePos.z > -0.01f) {
            return null;
        }

        Matrix4f viewProj = new Matrix4f(camera.getProjectionMatrix()).mul(camera.getViewMatrix());
        int[] viewport = new int[]{0, 0, screenWidth, screenHeight};
        Vector3f winCoords = new Vector3f();
        viewProj.project(worldPos.x, worldPos.y, worldPos.z, viewport, winCoords);

        return new Vector2f(winCoords.x, screenHeight - winCoords.y);
    }

    private float pointToSegmentDistance(float px, float py, float x1, float y1, float x2, float y2) {
        float dx = x2 - x1;
        float dy = y2 - y1;
        float l2 = dx * dx + dy * dy;
        if (l2 == 0.0f) {
            return (float) Math.hypot(px - x1, py - y1);
        }
        float t = Math.max(0.0f, Math.min(1.0f, ((px - x1) * dx + (py - y1) * dy) / l2));
        float projX = x1 + t * dx;
        float projY = y1 + t * dy;
        return (float) Math.hypot(px - projX, py - projY);
    }

    public void cleanup() {
        if (gizmoShader != null) {
            gizmoShader.cleanup();
        }
        if (vao != 0) glDeleteVertexArrays(vao);
        if (vbo != 0) glDeleteBuffers(vbo);
        if (markerVao != 0) glDeleteVertexArrays(markerVao);
        if (markerVbo != 0) glDeleteBuffers(markerVbo);
    }
}
