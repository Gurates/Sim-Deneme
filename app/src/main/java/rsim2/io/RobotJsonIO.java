package rsim2.io;

import com.google.gson.*;
import org.joml.Vector3f;
import rsim2.data.*;
import rsim2.graphics.Mesh;
import rsim2.scene.Joint;
import rsim2.scene.JointType;
import rsim2.scene.SceneNode;

import java.io.*;
import java.lang.reflect.Type;
import java.nio.charset.StandardCharsets;
import java.util.*;

public class RobotJsonIO {

    public static class SceneGraphResult {
        public SceneNode rootNode;
        public List<Joint> joints = new ArrayList<>();
        public Map<String, SceneNode> nodesById = new HashMap<>();
    }

    private static Gson createGson() {
        return new GsonBuilder()
                .setPrettyPrinting()
                .registerTypeAdapter(float[].class, new FloatArrayAdapter())
                .create();
    }

    private static class FloatArrayAdapter implements JsonDeserializer<float[]>, JsonSerializer<float[]> {
        @Override
        public float[] deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context) throws JsonParseException {
            if (json.isJsonPrimitive() && json.getAsJsonPrimitive().isNumber()) {
                float val = json.getAsFloat();
                return new float[]{ val, val, val };
            } else if (json.isJsonArray()) {
                JsonArray arr = json.getAsJsonArray();
                float[] res = new float[arr.size()];
                for (int i = 0; i < arr.size(); i++) {
                    res[i] = arr.get(i).getAsFloat();
                }
                return res;
            }
            return new float[]{ 1.0f, 1.0f, 1.0f };
        }

        @Override
        public JsonElement serialize(float[] src, Type typeOfSrc, JsonSerializationContext context) {
            JsonArray arr = new JsonArray();
            if (src != null) {
                for (float f : src) {
                    arr.add(f);
                }
            }
            return arr;
        }
    }

    public static void save(RobotDefinitionDTO def, String filePath) throws Exception {
        Gson gson = createGson();
        String jsonStr = gson.toJson(def);
        try (Writer writer = new OutputStreamWriter(new FileOutputStream(filePath), StandardCharsets.UTF_8)) {
            writer.write(jsonStr);
        }
    }

    public static RobotDefinitionDTO load(String filePath) throws Exception {
        Gson gson = createGson();
        try (Reader reader = new InputStreamReader(new FileInputStream(filePath), StandardCharsets.UTF_8)) {
            return gson.fromJson(reader, RobotDefinitionDTO.class);
        }
    }

    public static RobotDefinitionDTO fromSceneGraph(SceneNode root, List<Joint> joints, String robotName) {
        return fromSceneGraph(root, joints, robotName, null);
    }

    public static RobotDefinitionDTO fromSceneGraph(SceneNode root, List<Joint> joints, String robotName, String jsonFileDirectory) {
        RobotDefinitionDTO dto = new RobotDefinitionDTO();
        dto.schemaVersion = "1.0";
        dto.robotName = robotName;
        dto.metadata = new MetadataDTO("RSim2", "Exported robot definition");

        List<SceneNode> allNodes = new ArrayList<>();
        collectNodes(root, allNodes);

        for (SceneNode node : allNodes) {
            LinkDTO link = new LinkDTO();
            link.id = node.getId();

            List<String> meshPaths = new ArrayList<>();
            for (Mesh m : node.getMeshes()) {
                String rawPath = m.getSourcePath();
                if (rawPath == null || rawPath.isEmpty()) {
                    rawPath = node.getSourceMeshPath();
                }
                if (rawPath == null || rawPath.isEmpty()) {
                    rawPath = "models/test.obj";
                }

                if (jsonFileDirectory != null && !jsonFileDirectory.isEmpty()) {
                    meshPaths.add(makeRelativePath(jsonFileDirectory, rawPath));
                } else {
                    meshPaths.add(rawPath);
                }
            }

            if (meshPaths.size() > 1) {
                link.meshes = meshPaths;
                link.mesh = meshPaths.get(0);
            } else if (meshPaths.size() == 1) {
                link.mesh = meshPaths.get(0);
                link.meshes = meshPaths;
            } else {
                link.mesh = "models/test.obj";
            }

            link.mass = 1.0f;
            Vector3f scale = node.getLocalScale();
            link.scale = new float[]{ scale.x, scale.y, scale.z };

            Vector3f euler = new Vector3f();
            node.getLocalRotation().getEulerAnglesXYZ(euler);
            link.rotation = new float[]{ (float) Math.toDegrees(euler.x), (float) Math.toDegrees(euler.y), (float) Math.toDegrees(euler.z) };

            dto.links.add(link);
        }

        if (joints != null) {
            for (Joint j : joints) {
                JointDTO jDto = new JointDTO();
                jDto.id = j.getId();
                jDto.type = j.getType() != null ? j.getType().name().toLowerCase() : "revolute";
                jDto.parent = j.getParentNode() != null ? j.getParentNode().getId() : "";
                jDto.child = j.getChildNode() != null ? j.getChildNode().getId() : "";

                Vector3f axis = j.getAxis();
                jDto.axis = new float[]{ axis.x, axis.y, axis.z };

                Vector3f childPos = j.getChildNode() != null ? j.getChildNode().getLocalPosition() : new Vector3f();
                jDto.originPosition = new float[]{ childPos.x, childPos.y, childPos.z };

                if (j.getType() == JointType.REVOLUTE) {
                    jDto.limits = new LimitsDTO(j.getMinLimit(), j.getMaxLimit(), j.getMaxSpeed());
                    jDto.motor = new MotorDTO(
                            j.getMotorType() != null ? j.getMotorType() : "SERVO_PWM",
                            100.0f,
                            j.getPin(),
                            j.isInverted(),
                            j.getZeroOffsetDeg(),
                            j.getAcceleration(),
                            j.getTargetSpeedRatio()
                    );
                } else {
                    jDto.limits = null;
                    jDto.motor = null;
                }

                dto.joints.add(jDto);
            }
        }

        return dto;
    }

    private static String makeRelativePath(String jsonDir, String targetPath) {
        try {
            File targetFile = new File(targetPath);
            if (!targetFile.isAbsolute()) {
                return targetPath.replace('\\', '/');
            }
            File jsonDirFile = new File(jsonDir);
            java.nio.file.Path base = jsonDirFile.toPath().toAbsolutePath();
            java.nio.file.Path target = targetFile.toPath().toAbsolutePath();
            if (base.getRoot() != null && base.getRoot().equals(target.getRoot())) {
                return base.relativize(target).toString().replace('\\', '/');
            }
        } catch (Exception ignored) {
        }
        return targetPath.replace('\\', '/');
    }

    private static void collectNodes(SceneNode node, List<SceneNode> list) {
        if (node == null) return;
        list.add(node);
        for (SceneNode child : node.getChildren()) {
            collectNodes(child, list);
        }
    }

    public static SceneGraphResult toSceneGraph(RobotDefinitionDTO def) throws Exception {
        return toSceneGraph(def, null);
    }

    public static SceneGraphResult toSceneGraph(RobotDefinitionDTO def, String jsonFileDirectory) throws Exception {
        SceneGraphResult result = new SceneGraphResult();

        if (def == null || def.links == null) {
            return result;
        }

        for (LinkDTO link : def.links) {
            List<String> rawPaths = new ArrayList<>();
            if (link.meshes != null && !link.meshes.isEmpty()) {
                rawPaths.addAll(link.meshes);
            } else if (link.mesh != null && !link.mesh.isEmpty()) {
                rawPaths.add(link.mesh);
            }

            if (rawPaths.isEmpty()) {
                rawPaths.add("models/test.obj");
            }

            List<Mesh> loadedMeshes = new ArrayList<>();
            for (String rawPath : rawPaths) {
                String resolved = resolveMeshPath(rawPath, jsonFileDirectory);
                Mesh mesh;
                String lower = resolved.toLowerCase();
                if (lower.endsWith(".stl")) {
                    mesh = StlLoader.load(resolved);
                } else {
                    mesh = ObjLoader.load(resolved);
                }
                if (mesh != null) {
                    mesh.setSourcePath(resolved);
                    loadedMeshes.add(mesh);
                }
            }

            SceneNode node = new SceneNode(link.id, loadedMeshes);
            if (!loadedMeshes.isEmpty() && loadedMeshes.get(0).getSourcePath() != null) {
                node.setSourceMeshPath(loadedMeshes.get(0).getSourcePath());
            }

            if (link.scale != null && link.scale.length >= 3) {
                node.getLocalScale().set(link.scale[0], link.scale[1], link.scale[2]);
            }

            if (link.rotation != null && link.rotation.length >= 3) {
                node.getLocalRotation().rotationXYZ(
                        (float) Math.toRadians(link.rotation[0]),
                        (float) Math.toRadians(link.rotation[1]),
                        (float) Math.toRadians(link.rotation[2])
                );
            }

            result.nodesById.put(link.id, node);
        }

        if (def.joints != null) {
            for (JointDTO jDto : def.joints) {
                SceneNode parentNode = result.nodesById.get(jDto.parent);
                SceneNode childNode = result.nodesById.get(jDto.child);

                if (childNode != null && jDto.originPosition != null && jDto.originPosition.length >= 3) {
                    childNode.getLocalPosition().set(jDto.originPosition[0], jDto.originPosition[1], jDto.originPosition[2]);
                }

                if (parentNode != null && childNode != null) {
                    parentNode.addChild(childNode);
                }

                JointType type = JointType.fromString(jDto.type);
                Vector3f axis = (jDto.axis != null && jDto.axis.length >= 3) 
                        ? new Vector3f(jDto.axis[0], jDto.axis[1], jDto.axis[2]) 
                        : new Vector3f(0, 1, 0);

                Joint joint = new Joint(jDto.id, parentNode, childNode, type, axis);
                if (type == JointType.REVOLUTE && jDto.limits != null) {
                    joint.setLimits(jDto.limits.min, jDto.limits.max, jDto.limits.maxSpeed > 0 ? jDto.limits.maxSpeed : 2.0f);
                }
                if (jDto.motor != null) {
                    joint.setPin(jDto.motor.pin);
                    joint.setInverted(jDto.motor.inverted);
                    joint.setZeroOffsetDeg(jDto.motor.zeroOffsetDeg);
                    if (jDto.motor.acceleration > 0) {
                        joint.setAcceleration(jDto.motor.acceleration);
                    }
                    if (jDto.motor.speedRatio > 0) {
                        joint.setTargetSpeedRatio(jDto.motor.speedRatio);
                    }
                    if (jDto.motor.type != null && !jDto.motor.type.isEmpty()) {
                        joint.setMotorType(jDto.motor.type);
                    }
                }
                result.joints.add(joint);
            }
        }

        for (SceneNode node : result.nodesById.values()) {
            if (node.getParent() == null) {
                result.rootNode = node;
                break;
            }
        }

        return result;
    }

    private static String resolveMeshPath(String meshPath, String jsonFileDirectory) {
        if (meshPath == null || meshPath.isEmpty()) {
            return "models/test.obj";
        }

        File directFile = new File(meshPath);
        if (directFile.isAbsolute() && directFile.exists()) {
            return directFile.getAbsolutePath();
        }

        if (jsonFileDirectory != null && !jsonFileDirectory.isEmpty()) {
            File relFile = new File(jsonFileDirectory, meshPath);
            if (relFile.exists()) {
                return relFile.getAbsolutePath();
            }
        }

        return meshPath;
    }
}
