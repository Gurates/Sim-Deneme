package rsim2.io;

import org.joml.Vector3f;
import rsim2.graphics.Mesh;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

public class StlLoader {

    public static Mesh load(String filePath) throws Exception {
        File file = new File(filePath);
        byte[] bytes;
        if (file.exists() && file.isFile()) {
            try (InputStream in = new FileInputStream(file)) {
                bytes = in.readAllBytes();
            }
        } else {
            try (InputStream in = ObjLoader.getInputStream(filePath)) {
                bytes = in.readAllBytes();
            }
        }

        if (isBinaryStl(bytes)) {
            return parseBinaryStl(bytes);
        } else {
            return parseAsciiStl(bytes);
        }
    }

    private static boolean isBinaryStl(byte[] bytes) {
        if (bytes.length < 84) {
            return false;
        }
        ByteBuffer buffer = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
        int triangleCount = buffer.getInt(80);
        long expectedSize = 84L + (long) triangleCount * 50L;
        return bytes.length == expectedSize;
    }

    private static Mesh parseBinaryStl(byte[] bytes) {
        ByteBuffer buffer = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
        buffer.position(80);
        int triangleCount = buffer.getInt();

        float[] vertices = new float[triangleCount * 3 * 3];
        float[] normals = new float[triangleCount * 3 * 3];
        int[] indices = new int[triangleCount * 3];

        int vIdx = 0;
        int nIdx = 0;
        int iIdx = 0;

        for (int i = 0; i < triangleCount; i++) {
            float nx = buffer.getFloat();
            float ny = buffer.getFloat();
            float nz = buffer.getFloat();

            float v1x = buffer.getFloat();
            float v1y = buffer.getFloat();
            float v1z = buffer.getFloat();

            float v2x = buffer.getFloat();
            float v2y = buffer.getFloat();
            float v2z = buffer.getFloat();

            float v3x = buffer.getFloat();
            float v3y = buffer.getFloat();
            float v3z = buffer.getFloat();

            buffer.getShort();

            if (nx == 0.0f && ny == 0.0f && nz == 0.0f) {
                Vector3f e1 = new Vector3f(v2x - v1x, v2y - v1y, v2z - v1z);
                Vector3f e2 = new Vector3f(v3x - v1x, v3y - v1y, v3z - v1z);
                Vector3f norm = new Vector3f(e1).cross(e2).normalize();
                if (!Float.isNaN(norm.x)) {
                    nx = norm.x;
                    ny = norm.y;
                    nz = norm.z;
                }
            }

            vertices[vIdx++] = v1x;
            vertices[vIdx++] = v1y;
            vertices[vIdx++] = v1z;

            vertices[vIdx++] = v2x;
            vertices[vIdx++] = v2y;
            vertices[vIdx++] = v2z;

            vertices[vIdx++] = v3x;
            vertices[vIdx++] = v3y;
            vertices[vIdx++] = v3z;

            for (int k = 0; k < 3; k++) {
                normals[nIdx++] = nx;
                normals[nIdx++] = ny;
                normals[nIdx++] = nz;
                indices[iIdx] = iIdx++;
            }
        }

        return new Mesh(vertices, normals, indices);
    }

    private static Mesh parseAsciiStl(byte[] bytes) throws Exception {
        List<Float> positionsList = new ArrayList<>();
        List<Float> normalsList = new ArrayList<>();

        BufferedReader reader = new BufferedReader(new InputStreamReader(new ByteArrayInputStream(bytes), StandardCharsets.UTF_8));
        String line;

        float currentNx = 0.0f, currentNy = 1.0f, currentNz = 0.0f;

        while ((line = reader.readLine()) != null) {
            line = line.trim();
            if (line.isEmpty()) continue;

            String[] tokens = line.split("\\s+");
            if (tokens[0].equalsIgnoreCase("facet") && tokens.length >= 4 && tokens[1].equalsIgnoreCase("normal")) {
                currentNx = Float.parseFloat(tokens[2]);
                currentNy = Float.parseFloat(tokens[3]);
                currentNz = Float.parseFloat(tokens[4]);
            } else if (tokens[0].equalsIgnoreCase("vertex") && tokens.length >= 4) {
                float vx = Float.parseFloat(tokens[1]);
                float vy = Float.parseFloat(tokens[2]);
                float vz = Float.parseFloat(tokens[3]);

                positionsList.add(vx);
                positionsList.add(vy);
                positionsList.add(vz);

                normalsList.add(currentNx);
                normalsList.add(currentNy);
                normalsList.add(currentNz);
            }
        }

        float[] vertices = new float[positionsList.size()];
        for (int i = 0; i < positionsList.size(); i++) {
            vertices[i] = positionsList.get(i);
        }

        float[] normals = new float[normalsList.size()];
        for (int i = 0; i < normalsList.size(); i++) {
            normals[i] = normalsList.get(i);
        }

        int vertexCount = vertices.length / 3;
        int[] indices = new int[vertexCount];
        for (int i = 0; i < vertexCount; i++) {
            indices[i] = i;
        }

        return new Mesh(vertices, normals, indices);
    }
}
