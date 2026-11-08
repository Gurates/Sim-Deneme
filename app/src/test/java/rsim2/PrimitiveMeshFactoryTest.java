package rsim2;

import org.junit.jupiter.api.Test;
import rsim2.graphics.Mesh;
import rsim2.io.PrimitiveMeshFactory;

import static org.junit.jupiter.api.Assertions.*;

class PrimitiveMeshFactoryTest {
    @Test
    void boxHasSourceHalfExtentsAndOutwardFaces() {
        Mesh mesh = PrimitiveMeshFactory.box(0.019f, 0.019f, 0.0005f);
        assertEquals(-0.019f, mesh.getMinBound().x, 1e-7);
        assertEquals(0.019f, mesh.getMaxBound().y, 1e-7);
        assertEquals(0.0005f, mesh.getMaxBound().z, 1e-7);
        assertEquals(36, mesh.getIndices().length);
        assertOutwardNormals(mesh);
    }

    @Test
    void cameraLensCylinderUsesZAxisAndHalfLength() {
        Mesh mesh = PrimitiveMeshFactory.cylinder(0.007f, 0.00475f);
        assertEquals(-0.007f, mesh.getMinBound().x, 1e-7);
        assertEquals(0.007f, mesh.getMaxBound().y, 1e-7);
        assertEquals(-0.00475f, mesh.getMinBound().z, 1e-7);
        assertEquals(0.00475f, mesh.getMaxBound().z, 1e-7);
        assertOutwardNormals(mesh);
    }

    private static void assertOutwardNormals(Mesh mesh) {
        float[] vertices = mesh.getVertices(), normals = mesh.getNormals();
        for (int i = 0; i < vertices.length; i += 3) {
            float dot = vertices[i]*normals[i] + vertices[i+1]*normals[i+1] + vertices[i+2]*normals[i+2];
            assertTrue(dot > 0, "Normals and winding must face outward");
            float length = normals[i]*normals[i]+normals[i+1]*normals[i+1]+normals[i+2]*normals[i+2];
            assertEquals(1f, length, 1e-5);
        }
    }
}
