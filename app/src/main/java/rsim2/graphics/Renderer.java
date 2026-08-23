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
    private int modelCameraPosLocation;

    private Mesh testMesh;
    
    private final Matrix4f modelMatrix = new Matrix4f();
    
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

    public void init() throws Exception {
        setupGrid();
        setupModelPipeline();
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
        modelCameraPosLocation = modelShaderProgram.getUniformLocation("cameraPos");

        testMesh = ObjLoader.load("/models/test.obj");
    }

    public void render(Camera camera, SceneNode rootNode) {
        render(camera, rootNode, null);
    }

    public void render(Camera camera, SceneNode rootNode, SceneNode selectedNode) {
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

            renderNode(rootNode, selectedNode);

            modelShaderProgram.unbind();
        }
    }

    private void renderNode(SceneNode node, SceneNode selectedNode) {
        if (!node.getMeshes().isEmpty()) {
            FloatBuffer modelBuffer = BufferUtils.createFloatBuffer(16);
            node.getWorldTransform().get(modelBuffer);
            glUniformMatrix4fv(modelModelLocation, false, modelBuffer);

            boolean isSel = (node == selectedNode);
            if (modelIsSelectedLocation >= 0) {
                glUniform1i(modelIsSelectedLocation, isSel ? 1 : 0);
            }

            for (Mesh mesh : node.getMeshes()) {
                if (mesh != null) {
                    mesh.render();
                }
            }
        }

        for (SceneNode child : node.getChildren()) {
            renderNode(child, selectedNode);
        }
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
        if (testMesh != null) {
            testMesh.cleanup();
        }
        glBindVertexArray(0);
        if (gridVao != 0) glDeleteVertexArrays(gridVao);
        if (gridVbo != 0) glDeleteBuffers(gridVbo);
        if (gridEbo != 0) glDeleteBuffers(gridEbo);
    }
}
