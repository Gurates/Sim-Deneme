package rsim2.graphics;

import org.lwjgl.BufferUtils;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;

import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.opengl.GL15.*;
import static org.lwjgl.opengl.GL20.*;
import static org.lwjgl.opengl.GL30.*;

public class Mesh {
    private final int vao;
    private final int vboPositions;
    private final int vboNormals;
    private final int ebo;
    private final int vertexCount;

    public Mesh(float[] vertices, float[] normals, int[] indices) {
        this.vertexCount = indices.length;

        vao = glGenVertexArrays();
        glBindVertexArray(vao);

        FloatBuffer posBuffer = BufferUtils.createFloatBuffer(vertices.length);
        posBuffer.put(vertices).flip();

        vboPositions = glGenBuffers();
        glBindBuffer(GL_ARRAY_BUFFER, vboPositions);
        glBufferData(GL_ARRAY_BUFFER, posBuffer, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, false, 0, 0);
        glEnableVertexAttribArray(0);

        if (normals != null && normals.length > 0) {
            FloatBuffer normBuffer = BufferUtils.createFloatBuffer(normals.length);
            normBuffer.put(normals).flip();

            vboNormals = glGenBuffers();
            glBindBuffer(GL_ARRAY_BUFFER, vboNormals);
            glBufferData(GL_ARRAY_BUFFER, normBuffer, GL_STATIC_DRAW);
            glVertexAttribPointer(1, 3, GL_FLOAT, false, 0, 0);
            glEnableVertexAttribArray(1);
        } else {
            vboNormals = 0;
        }

        IntBuffer idxBuffer = BufferUtils.createIntBuffer(indices.length);
        idxBuffer.put(indices).flip();

        ebo = glGenBuffers();
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, idxBuffer, GL_STATIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    public void render() {
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, vertexCount, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }

    public void cleanup() {
        glBindVertexArray(0);
        if (vao != 0) glDeleteVertexArrays(vao);
        if (vboPositions != 0) glDeleteBuffers(vboPositions);
        if (vboNormals != 0) glDeleteBuffers(vboNormals);
        if (ebo != 0) glDeleteBuffers(ebo);
    }
}
