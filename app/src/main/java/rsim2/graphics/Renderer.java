package rsim2.graphics;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.lwjgl.BufferUtils;
import rsim2.camera.Camera;
import rsim2.io.ObjLoader;
import rsim2.scene.SceneNode;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;

import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.opengl.GL15.*;
import static org.lwjgl.opengl.GL20.*;
import static org.lwjgl.opengl.GL30.*;

public class Renderer {
    private ShaderProgram gridShaderProgram;
    private int gridVao;
    private int gridVbo, gridEbo;

    private int gridProjectionLocation;
    private int gridViewLocation;
    private int gridModelLocation;

    private ShaderProgram modelShaderProgram;
    private int modelProjectionLocation;
    private int modelViewLocation;
    private int modelModelLocation;
    private int modelIsSelectedLocation;
    private int modelIsCollidingLocation;
    private int modelCameraPosLocation;

    private ShaderProgram wireframeShaderProgram;
    private int wireframeVao, wireframeVbo;
    private int wireframeProjectionLocation;
    private int wireframeViewLocation;
    private int wireframeModelLocation;
    private int wireframeColorLocation;

    private Mesh testMesh;
    
    private final Matrix4f modelMatrix = new Matrix4f();
    private final Matrix4f obbMatrix = new Matrix4f();
    
    private final float[] gridVertices = {
        -100.0f, 0.0f, -100.0f,
         100.0f, 0.0f, -100.0f,
         100.0f, 0.0f,  100.0f,
        -100.0f, 0.0f,  100.0f
    };

    private final int[] gridIndices = {
        0, 1, 2,
        2, 3, 0
    };

    private final float[] unitCubeWireframeVertices = {
        -1, -1, -1,   1, -1, -1,
         1, -1, -1,   1, -1,  1,
         1, -1,  1,  -1, -1,  1,
        -1, -1,  1,  -1, -1, -1,
        -1,  1, -1,   1,  1, -1,
         1,  1, -1,   1,  1,  1,
         1,  1,  1,  -1,  1,  1,
        -1,  1,  1,  -1,  1, -1,
        -1, -1, -1,  -1,  1, -1,
         1, -1, -1,   1,  1, -1,
         1, -1,  1,   1,  1,  1,
        -1, -1,  1,  -1,  1,  1
    };

    public void init() throws Exception {
        setupGrid();
        setupModelPipeline();
        setupWireframePipeline();
    }

    private void setupWireframePipeline() throws Exception {
        wireframeShaderProgram = new ShaderProgram();
        wireframeShaderProgram.createVertexShader("/shaders/gizmo.vert");
        wireframeShaderProgram.createFragmentShader("/shaders/gizmo.frag");
        wireframeShaderProgram.link();

        wireframeProjectionLocation = wireframeShaderProgram.getUniformLocation("projectionMatrix");
        wireframeViewLocation = wireframeShaderProgram.getUniformLocation("viewMatrix");
        wireframeModelLocation = wireframeShaderProgram.getUniformLocation("modelMatrix");
        wireframeColorLocation = wireframeShaderProgram.getUniformLocation("color");

        wireframeVao = glGenVertexArrays();
        glBindVertexArray(wireframeVao);

        FloatBuffer cubeBuf = BufferUtils.createFloatBuffer(unitCubeWireframeVertices.length);
        cubeBuf.put(unitCubeWireframeVertices).flip();

        wireframeVbo = glGenBuffers();
        glBindBuffer(GL_ARRAY_BUFFER, wireframeVbo);
        glBufferData(GL_ARRAY_BUFFER, cubeBuf, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, false, 3 * Float.BYTES, 0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    private void setupGrid() throws Exception {
        gridShaderProgram = new ShaderProgram();
        gridShaderProgram.createVertexShader("/shaders/default.vert");
        gridShaderProgram.createFragmentShader("/shaders/default.frag");
        gridShaderProgram.link();

        gridProjectionLocation = gridShaderProgram.getUniformLocation("projectionMatrix");
        gridViewLocation = gridShaderProgram.getUniformLocation("viewMatrix");
        gridModelLocation = gridShaderProgram.getUniformLocation("modelMatrix");

        gridVao = glGenVertexArrays();
        glBindVertexArray(gridVao);

        FloatBuffer verticesBuffer = BufferUtils.createFloatBuffer(gridVertices.length);
        verticesBuffer.put(gridVertices).flip();

        gridVbo = glGenBuffers();
        glBindBuffer(GL_ARRAY_BUFFER, gridVbo);
        glBufferData(GL_ARRAY_BUFFER, verticesBuffer, GL_STATIC_DRAW);

        IntBuffer indicesBuffer = BufferUtils.createIntBuffer(gridIndices.length);
        indicesBuffer.put(gridIndices).flip();

        gridEbo = glGenBuffers();
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gridEbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indicesBuffer, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, false, 3 * Float.BYTES, 0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    private void setupModelPipeline() throws Exception {
        modelShaderProgram = new ShaderProgram();
        modelShaderProgram.createVertexShader("/shaders/model.vert");
        modelShaderProgram.createFragmentShader("/shaders/model.frag");
        modelShaderProgram.link();

        modelProjectionLocation = modelShaderProgram.getUniformLocation("projectionMatrix");
        modelViewLocation = modelShaderProgram.getUniformLocation("viewMatrix");
        modelModelLocation = modelShaderProgram.getUniformLocation("modelMatrix");
        modelIsSelectedLocation = modelShaderProgram.getUniformLocation("isSelected");
        modelIsCollidingLocation = modelShaderProgram.getUniformLocation("isColliding");
        modelCameraPosLocation = modelShaderProgram.getUniformLocation("cameraPos");

        testMesh = ObjLoader.load("/models/test.obj");
    }

    public void render(Camera camera, SceneNode rootNode) {
        render(camera, rootNode, null, null, null);
    }

    public void render(Camera camera, SceneNode rootNode, SceneNode selectedNode) {
        render(camera, rootNode, selectedNode, null, null);
    }

    public void render(Camera camera, SceneNode rootNode, SceneNode selectedNode,
                       java.util.Set<SceneNode> collidingNodes,
                       java.util.Map<SceneNode, rsim2.collision.OBB> debugOBBs) {
        renderGrid(camera);

        if (rootNode != null) {
            modelShaderProgram.bind();

            FloatBuffer projectionBuffer = BufferUtils.createFloatBuffer(16);
            camera.getProjectionMatrix().get(projectionBuffer);
            glUniformMatrix4fv(modelProjectionLocation, false, projectionBuffer);

            FloatBuffer viewBuffer = BufferUtils.createFloatBuffer(16);
            camera.getViewMatrix().get(viewBuffer);
            glUniformMatrix4fv(modelViewLocation, false, viewBuffer);

            Vector3f camPos = camera.getPosition();
            if (modelCameraPosLocation >= 0 && camPos != null) {
                glUniform3f(modelCameraPosLocation, camPos.x, camPos.y, camPos.z);
            }

            renderNode(rootNode, selectedNode, collidingNodes);

            modelShaderProgram.unbind();
        }

        if (debugOBBs != null && !debugOBBs.isEmpty()) {
            renderDebugWireframes(camera, selectedNode, collidingNodes, debugOBBs);
        }
    }

    private void renderNode(SceneNode node, SceneNode selectedNode, java.util.Set<SceneNode> collidingNodes) {
        if (!node.getMeshes().isEmpty()) {
            FloatBuffer modelBuffer = BufferUtils.createFloatBuffer(16);
            node.getWorldTransform().get(modelBuffer);
            glUniformMatrix4fv(modelModelLocation, false, modelBuffer);

            boolean isSel = (node == selectedNode);
            if (modelIsSelectedLocation >= 0) {
                glUniform1i(modelIsSelectedLocation, isSel ? 1 : 0);
            }

            boolean isColliding = (collidingNodes != null && collidingNodes.contains(node));
            if (modelIsCollidingLocation >= 0) {
                glUniform1i(modelIsCollidingLocation, isColliding ? 1 : 0);
            }

            for (Mesh mesh : node.getMeshes()) {
                if (mesh != null) {
                    mesh.render();
                }
            }
        }

        for (SceneNode child : node.getChildren()) {
            renderNode(child, selectedNode, collidingNodes);
        }
    }

    private void renderDebugWireframes(Camera camera, SceneNode selectedNode,
                                       java.util.Set<SceneNode> collidingNodes,
                                       java.util.Map<SceneNode, rsim2.collision.OBB> debugOBBs) {
        if (wireframeShaderProgram == null || debugOBBs == null || debugOBBs.isEmpty()) return;

        wireframeShaderProgram.bind();

        FloatBuffer projectionBuffer = BufferUtils.createFloatBuffer(16);
        camera.getProjectionMatrix().get(projectionBuffer);
        glUniformMatrix4fv(wireframeProjectionLocation, false, projectionBuffer);

        FloatBuffer viewBuffer = BufferUtils.createFloatBuffer(16);
        camera.getViewMatrix().get(viewBuffer);
        glUniformMatrix4fv(wireframeViewLocation, false, viewBuffer);

        glBindVertexArray(wireframeVao);

        FloatBuffer matBuf = BufferUtils.createFloatBuffer(16);

        for (java.util.Map.Entry<SceneNode, rsim2.collision.OBB> entry : debugOBBs.entrySet()) {
            SceneNode node = entry.getKey();
            rsim2.collision.OBB obb = entry.getValue();
            if (obb == null) continue;

            boolean isColliding = (collidingNodes != null && collidingNodes.contains(node));
            boolean isSel = (node == selectedNode);

            if (isColliding) {
                glUniform4f(wireframeColorLocation, 1.0f, 0.2f, 0.2f, 0.95f);
            } else if (isSel) {
                glUniform4f(wireframeColorLocation, 0.2f, 0.85f, 1.0f, 0.90f);
            } else {
                glUniform4f(wireframeColorLocation, 0.25f, 0.85f, 0.35f, 0.65f);
            }

            Vector3f c = obb.getCenter();
            Vector3f[] u = obb.getAxes();
            Vector3f e = obb.getHalfExtents();

            obbMatrix.identity();
            obbMatrix.set(
                    u[0].x * e.x, u[0].y * e.x, u[0].z * e.x, 0.0f,
                    u[1].x * e.y, u[1].y * e.y, u[1].z * e.y, 0.0f,
                    u[2].x * e.z, u[2].y * e.z, u[2].z * e.z, 0.0f,
                    c.x, c.y, c.z, 1.0f
            );

            matBuf.clear();
            obbMatrix.get(matBuf);
            glUniformMatrix4fv(wireframeModelLocation, false, matBuf);

            glDrawArrays(GL_LINES, 0, 24);
        }

        glBindVertexArray(0);
        wireframeShaderProgram.unbind();
    }

    private void renderGrid(Camera camera) {
        gridShaderProgram.bind();

        FloatBuffer projectionBuffer = BufferUtils.createFloatBuffer(16);
        camera.getProjectionMatrix().get(projectionBuffer);
        glUniformMatrix4fv(gridProjectionLocation, false, projectionBuffer);

        FloatBuffer viewBuffer = BufferUtils.createFloatBuffer(16);
        camera.getViewMatrix().get(viewBuffer);
        glUniformMatrix4fv(gridViewLocation, false, viewBuffer);

        FloatBuffer modelBuffer = BufferUtils.createFloatBuffer(16);
        modelMatrix.identity();
        modelMatrix.get(modelBuffer);
        glUniformMatrix4fv(gridModelLocation, false, modelBuffer);

        glBindVertexArray(gridVao);
        glDrawElements(GL_TRIANGLES, gridIndices.length, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
        
        gridShaderProgram.unbind();
    }

    public Mesh getTestMesh() {
        return testMesh;
    }

    public void cleanup() {
        if (gridShaderProgram != null) {
            gridShaderProgram.cleanup();
        }
        if (modelShaderProgram != null) {
            modelShaderProgram.cleanup();
        }
        if (wireframeShaderProgram != null) {
            wireframeShaderProgram.cleanup();
        }
        if (testMesh != null) {
            testMesh.cleanup();
        }
        glBindVertexArray(0);
        if (gridVao != 0) glDeleteVertexArrays(gridVao);
        if (gridVbo != 0) glDeleteBuffers(gridVbo);
        if (gridEbo != 0) glDeleteBuffers(gridEbo);
        if (wireframeVao != 0) glDeleteVertexArrays(wireframeVao);
        if (wireframeVbo != 0) glDeleteBuffers(wireframeVbo);
    }
}
