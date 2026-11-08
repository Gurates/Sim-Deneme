package rsim2.graphics;

import org.joml.Vector3f;
import org.joml.Matrix4f;
import org.joml.Matrix3f;
import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL;

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

    private final float[] vertices;
    private final float[] normals;
    private final int[] indices;
    private String sourcePath;
    private final Matrix4f sourceTransform = new Matrix4f();

    public Mesh(float[] vertices, float[] normals, int[] indices) {
        this(vertices, normals, indices, null);
    }

    public Mesh(float[] vertices, float[] normals, int[] indices, String sourcePath) {
        this.vertices = vertices;
        this.normals = normals;
        this.indices = indices;
        this.sourcePath = sourcePath;
        this.vertexCount = indices != null ? indices.length : 0;

        minBound = new Vector3f(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
        maxBound = new Vector3f(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE);

        if (vertices != null) {
            for (int i = 0; i < vertices.length; i += 3) {
                float x = vertices[i];
                float y = vertices[i + 1];
                float z = vertices[i + 2];
                minBound.min(new Vector3f(x, y, z));
                maxBound.max(new Vector3f(x, y, z));
            }
        }

        int vaoId = 0, vboPosId = 0, vboNormId = 0, eboId = 0;
        try {
            if (GL.getCapabilities() != null) {
                vaoId = glGenVertexArrays();
                glBindVertexArray(vaoId);

                if (vertices != null && vertices.length > 0) {
                    FloatBuffer posBuffer = BufferUtils.createFloatBuffer(vertices.length);
                    posBuffer.put(vertices).flip();

                    vboPosId = glGenBuffers();
                    glBindBuffer(GL_ARRAY_BUFFER, vboPosId);
                    glBufferData(GL_ARRAY_BUFFER, posBuffer, GL_STATIC_DRAW);
                    glVertexAttribPointer(0, 3, GL_FLOAT, false, 0, 0);
                    glEnableVertexAttribArray(0);
                }

                if (normals != null && normals.length > 0) {
                    FloatBuffer normBuffer = BufferUtils.createFloatBuffer(normals.length);
                    normBuffer.put(normals).flip();

                    vboNormId = glGenBuffers();
                    glBindBuffer(GL_ARRAY_BUFFER, vboNormId);
                    glBufferData(GL_ARRAY_BUFFER, normBuffer, GL_STATIC_DRAW);
                    glVertexAttribPointer(1, 3, GL_FLOAT, false, 0, 0);
                    glEnableVertexAttribArray(1);
                }

                if (indices != null && indices.length > 0) {
                    IntBuffer idxBuffer = BufferUtils.createIntBuffer(indices.length);
                    idxBuffer.put(indices).flip();

                    eboId = glGenBuffers();
                    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eboId);
                    glBufferData(GL_ELEMENT_ARRAY_BUFFER, idxBuffer, GL_STATIC_DRAW);
                }

                glBindBuffer(GL_ARRAY_BUFFER, 0);
                glBindVertexArray(0);
            }
        } catch (Throwable ignored) {
        }

        this.vao = vaoId;
        this.vboPositions = vboPosId;
        this.vboNormals = vboNormId;
        this.ebo = eboId;
    }

    public static Mesh createScaled(Mesh original, Vector3f scale) {
        if (original == null) return null;
        if (scale == null || (scale.x == 1.0f && scale.y == 1.0f && scale.z == 1.0f)) {
            return original;
        }

        return createTransformed(original, new Matrix4f().scaling(scale));
    }

    /** Bakes a visual-local transform without changing the owning link/joint frame. */
    public static Mesh createTransformed(Mesh original, Matrix4f transform) {
        if (original == null) return null;
        float[] origVerts = original.getVertices();
        if (origVerts == null || origVerts.length == 0) {
            return original;
        }

        float[] scaledVerts = new float[origVerts.length];
        for (int i = 0; i < origVerts.length; i += 3) {
            Vector3f p = transform.transformPosition(new Vector3f(origVerts[i], origVerts[i + 1], origVerts[i + 2]));
            scaledVerts[i] = p.x;
            scaledVerts[i + 1] = p.y;
            scaledVerts[i + 2] = p.z;
        }
        float[] transformedNormals = original.getNormals() == null ? null : original.getNormals().clone();
        Matrix3f normalMatrix = transform.normal(new Matrix3f());
        if (transformedNormals != null) {
            for (int i = 0; i < transformedNormals.length; i += 3) {
                Vector3f n = normalMatrix.transform(new Vector3f(transformedNormals[i], transformedNormals[i + 1], transformedNormals[i + 2]));
                if (n.lengthSquared() > 0) n.normalize();
                transformedNormals[i] = n.x;
                transformedNormals[i + 1] = n.y;
                transformedNormals[i + 2] = n.z;
            }
        }
        Mesh scaled = new Mesh(scaledVerts, transformedNormals, original.getIndices(), original.getSourcePath());
        scaled.sourceTransform.set(transform).mul(original.sourceTransform);
        original.cleanup();
        return scaled;
    }

    public Matrix4f getSourceTransform() {
        return new Matrix4f(sourceTransform);
    }

    public void render() {
        try {
            if (vao != 0 && GL.getCapabilities() != null) {
                glBindVertexArray(vao);
                glDrawElements(GL_TRIANGLES, vertexCount, GL_UNSIGNED_INT, 0);
                glBindVertexArray(0);
            }
        } catch (Throwable ignored) {
        }
    }

    public float[] getVertices() {
        return vertices;
    }

    public float[] getNormals() {
        return normals;
    }

    public int[] getIndices() {
        return indices;
    }

    public String getSourcePath() {
        return sourcePath;
    }

    public void setSourcePath(String sourcePath) {
        this.sourcePath = sourcePath;
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
        try {
            if (GL.getCapabilities() != null) {
                glBindVertexArray(0);
                if (vao != 0) glDeleteVertexArrays(vao);
                if (vboPositions != 0) glDeleteBuffers(vboPositions);
                if (vboNormals != 0) glDeleteBuffers(vboNormals);
                if (ebo != 0) glDeleteBuffers(ebo);
            }
        } catch (Throwable ignored) {
        }
    }
}
