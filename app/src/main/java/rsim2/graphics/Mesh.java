package rsim2.graphics;

import org.joml.Vector3f;
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
    private final Vector3f minBound;
    private final Vector3f maxBound;

    public Mesh(float[] vertices, float[] normals, int[] indices) {
        this.vertexCount = indices.length;

        minBound = new Vector3f(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
        maxBound = new Vector3f(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE);

        for (int i = 0; i < vertices.length; i += 3) {
            float x = vertices[i];
            float y = vertices[i + 1];
            float z = vertices[i + 2];
            minBound.min(new Vector3f(x, y, z));
            maxBound.max(new Vector3f(x, y, z));
        }

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

    public Vector3f getMinBound() {
        return minBound;
    }

    public Vector3f getMaxBound() {
        return maxBound;
    }

    public Vector3f getBoundingBoxMin() {
        return minBound;
    }

    public Vector3f getBoundingBoxMax() {
        return maxBound;
    }

    public void cleanup() {
        glBindVertexArray(0);
        if (vao != 0) glDeleteVertexArrays(vao);
        if (vboPositions != 0) glDeleteBuffers(vboPositions);
        if (vboNormals != 0) glDeleteBuffers(vboNormals);
        if (ebo != 0) glDeleteBuffers(ebo);
    }
}
