package rsim2.io;

import org.joml.Vector3f;
import rsim2.graphics.Mesh;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.*;

public class ObjLoader {

    public static InputStream getInputStream(String path) throws Exception {
        File file = new File(path);
        if (file.exists() && file.isFile()) {
            return new FileInputStream(file);
        }
        InputStream in = ObjLoader.class.getResourceAsStream(path);
        if (in != null) {
            return in;
        }
        if (!path.startsWith("/")) {
            in = ObjLoader.class.getResourceAsStream("/" + path);
            if (in != null) return in;
        }
        throw new FileNotFoundException("Could not find file or resource: " + path);
    }

    public static Mesh load(String resourcePath) throws Exception {
        List<Vector3f> rawPositions = new ArrayList<>();
        List<Vector3f> rawNormals = new ArrayList<>();

        List<Float> finalPositions = new ArrayList<>();
        List<Float> finalNormals = new ArrayList<>();
        List<Integer> finalIndices = new ArrayList<>();

        Map<String, Integer> vertexMap = new HashMap<>();

        try (InputStream in = getInputStream(resourcePath)) {
            BufferedReader reader = new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8));
            String line;

            while ((line = reader.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) {
                    continue;
                }

                String[] tokens = line.split("\\s+");
                if (tokens[0].equals("v")) {
                    float x = Float.parseFloat(tokens[1]);
                    float y = Float.parseFloat(tokens[2]);
                    float z = Float.parseFloat(tokens[3]);
                    rawPositions.add(new Vector3f(x, y, z));
                } else if (tokens[0].equals("vn")) {
                    float x = Float.parseFloat(tokens[1]);
                    float y = Float.parseFloat(tokens[2]);
                    float z = Float.parseFloat(tokens[3]);
                    rawNormals.add(new Vector3f(x, y, z));
                } else if (tokens[0].equals("f")) {
                    List<String> faceTokens = new ArrayList<>(Arrays.asList(tokens).subList(1, tokens.length));

                    if (faceTokens.size() == 3) {
                        processVertexToken(faceTokens.get(0), rawPositions, rawNormals, vertexMap, finalPositions, finalNormals, finalIndices);
                        processVertexToken(faceTokens.get(1), rawPositions, rawNormals, vertexMap, finalPositions, finalNormals, finalIndices);
                        processVertexToken(faceTokens.get(2), rawPositions, rawNormals, vertexMap, finalPositions, finalNormals, finalIndices);
                    } else if (faceTokens.size() == 4) {
                        processVertexToken(faceTokens.get(0), rawPositions, rawNormals, vertexMap, finalPositions, finalNormals, finalIndices);
                        processVertexToken(faceTokens.get(1), rawPositions, rawNormals, vertexMap, finalPositions, finalNormals, finalIndices);
                        processVertexToken(faceTokens.get(2), rawPositions, rawNormals, vertexMap, finalPositions, finalNormals, finalIndices);

                        processVertexToken(faceTokens.get(0), rawPositions, rawNormals, vertexMap, finalPositions, finalNormals, finalIndices);
                        processVertexToken(faceTokens.get(2), rawPositions, rawNormals, vertexMap, finalPositions, finalNormals, finalIndices);
                        processVertexToken(faceTokens.get(3), rawPositions, rawNormals, vertexMap, finalPositions, finalNormals, finalIndices);
                    }
                }
            }
        }

        float[] posArray = new float[finalPositions.size()];
        for (int i = 0; i < finalPositions.size(); i++) {
            posArray[i] = finalPositions.get(i);
        }

        float[] normArray = new float[finalNormals.size()];
        for (int i = 0; i < finalNormals.size(); i++) {
            normArray[i] = finalNormals.get(i);
        }

        int[] idxArray = new int[finalIndices.size()];
        for (int i = 0; i < finalIndices.size(); i++) {
            idxArray[i] = finalIndices.get(i);
        }

        return new Mesh(posArray, normArray, idxArray);
    }

    private static void processVertexToken(
            String token,
            List<Vector3f> rawPositions,
            List<Vector3f> rawNormals,
            Map<String, Integer> vertexMap,
            List<Float> finalPositions,
            List<Float> finalNormals,
            List<Integer> finalIndices
    ) {
        String[] parts = token.split("/");
        int vIdx = Integer.parseInt(parts[0]) - 1;
        int vnIdx = -1;
        if (parts.length > 2 && !parts[2].isEmpty()) {
            vnIdx = Integer.parseInt(parts[2]) - 1;
        }

        String key = vIdx + "/" + vnIdx;

        if (vertexMap.containsKey(key)) {
            finalIndices.add(vertexMap.get(key));
        } else {
            int newIdx = vertexMap.size();
            vertexMap.put(key, newIdx);

            Vector3f pos = rawPositions.get(vIdx);
            finalPositions.add(pos.x);
            finalPositions.add(pos.y);
            finalPositions.add(pos.z);

            if (vnIdx >= 0 && vnIdx < rawNormals.size()) {
                Vector3f norm = rawNormals.get(vnIdx);
                finalNormals.add(norm.x);
                finalNormals.add(norm.y);
                finalNormals.add(norm.z);
            } else {
                finalNormals.add(0.0f);
                finalNormals.add(1.0f);
                finalNormals.add(0.0f);
            }

            finalIndices.add(newIdx);
        }
    }
}
