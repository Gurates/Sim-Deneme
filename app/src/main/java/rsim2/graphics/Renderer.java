package rsim2.graphics;

import org.joml.Matrix4f;
import org.lwjgl.BufferUtils;
import rsim2.camera.Camera;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;

import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.opengl.GL15.*;
import static org.lwjgl.opengl.GL20.*;
import static org.lwjgl.opengl.GL30.*;

public class Renderer {
    private ShaderProgram shaderProgram;
    private int vao;
    private int vbo, ebo;

    private int projectionMatrixLocation;
    private int viewMatrixLocation;
    private int modelMatrixLocation;
    
    private final Matrix4f modelMatrix = new Matrix4f();
    
    private final float[] vertices = {
        -100.0f, 0.0f, -100.0f,
         100.0f, 0.0f, -100.0f,
         100.0f, 0.0f,  100.0f,
        -100.0f, 0.0f,  100.0f
    };

    private final int[] indices = {
        0, 1, 2,
        2, 3, 0
    };

    public void init() throws Exception {
        setupShaders();
        setupMesh();
    }

    private void setupShaders() throws Exception {
        shaderProgram = new ShaderProgram();
        shaderProgram.createVertexShader("/shaders/default.vert");
        shaderProgram.createFragmentShader("/shaders/default.frag");
        shaderProgram.link();

        projectionMatrixLocation = shaderProgram.getUniformLocation("projectionMatrix");
        viewMatrixLocation = shaderProgram.getUniformLocation("viewMatrix");
        modelMatrixLocation = shaderProgram.getUniformLocation("modelMatrix");
    }

    private void setupMesh() {
        vao = glGenVertexArrays();
        glBindVertexArray(vao);

        FloatBuffer verticesBuffer = BufferUtils.createFloatBuffer(vertices.length);
        verticesBuffer.put(vertices).flip();

        vbo = glGenBuffers();
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, verticesBuffer, GL_STATIC_DRAW);

        IntBuffer indicesBuffer = BufferUtils.createIntBuffer(indices.length);
        indicesBuffer.put(indices).flip();

        ebo = glGenBuffers();
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indicesBuffer, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, false, 3 * Float.BYTES, 0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    public void render(Camera camera) {
        shaderProgram.bind();

        FloatBuffer projectionBuffer = BufferUtils.createFloatBuffer(16);
        camera.getProjectionMatrix().get(projectionBuffer);
        glUniformMatrix4fv(projectionMatrixLocation, false, projectionBuffer);

        FloatBuffer viewBuffer = BufferUtils.createFloatBuffer(16);
        camera.getViewMatrix().get(viewBuffer);
        glUniformMatrix4fv(viewMatrixLocation, false, viewBuffer);

        FloatBuffer modelBuffer = BufferUtils.createFloatBuffer(16);
        modelMatrix.identity();
        modelMatrix.get(modelBuffer);
        glUniformMatrix4fv(modelMatrixLocation, false, modelBuffer);

        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, indices.length, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
        
        shaderProgram.unbind();
    }

    public void cleanup() {
        if (shaderProgram != null) {
            shaderProgram.cleanup();
        }
        glBindVertexArray(0);
        if (vao != 0) glDeleteVertexArrays(vao);
        if (vbo != 0) glDeleteBuffers(vbo);
        if (ebo != 0) glDeleteBuffers(ebo);
    }
}
