package rsim2.io;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import org.joml.Vector3f;
import rsim2.data.*;
import rsim2.graphics.Mesh;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.*;

public class RobotJsonIO {

    public static class SceneGraphResult {
        public SceneNode rootNode;
        public List<Joint> joints = new ArrayList<>();
        public Map<String, SceneNode> nodesById = new HashMap<>();
    }

    public static void save(RobotDefinitionDTO def, String filePath) throws Exception {
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        String jsonStr = gson.toJson(def);
        try (Writer writer = new OutputStreamWriter(new FileOutputStream(filePath), StandardCharsets.UTF_8)) {
            writer.write(jsonStr);
        }
    }

    public static RobotDefinitionDTO load(String filePath) throws Exception {
        Gson gson = new Gson();
        try (Reader reader = new InputStreamReader(new FileInputStream(filePath), StandardCharsets.UTF_8)) {
            return gson.fromJson(reader, RobotDefinitionDTO.class);
        }
    }

    public static RobotDefinitionDTO fromSceneGraph(SceneNode root, List<Joint> joints, String robotName) {
        RobotDefinitionDTO dto = new RobotDefinitionDTO();
        dto.schemaVersion = "1.0";
        dto.robotName = robotName;
        dto.metadata = new MetadataDTO("RSim2", "Exported robot definition");

        List<SceneNode> allNodes = new ArrayList<>();
        collectNodes(root, allNodes);

        for (SceneNode node : allNodes) {
            LinkDTO link = new LinkDTO();
            link.id = node.getId();
            link.mesh = node.getSourceMeshPath() != null ? node.getSourceMeshPath() : "models/test.obj";
            link.mass = 1.0f;
            dto.links.add(link);
        }

        if (joints != null) {
            for (Joint j : joints) {
                JointDTO jDto = new JointDTO();
                jDto.id = j.getId();
                jDto.type = "revolute";
                jDto.parent = j.getParentNode() != null ? j.getParentNode().getId() : "";
                jDto.child = j.getChildNode() != null ? j.getChildNode().getId() : "";
                
                Vector3f axis = j.getAxis();
                jDto.axis = new float[]{ axis.x, axis.y, axis.z };

                Vector3f childPos = j.getChildNode() != null ? j.getChildNode().getLocalPosition() : new Vector3f();
                jDto.originPosition = new float[]{ childPos.x, childPos.y, childPos.z };

                jDto.limits = new LimitsDTO(-3.14f, 3.14f, 10.0f);
                jDto.motor = new MotorDTO("servo", 100.0f);

                dto.joints.add(jDto);
            }
        }

        return dto;
    }

    private static void collectNodes(SceneNode node, List<SceneNode> list) {
        if (node == null) return;
        list.add(node);
        for (SceneNode child : node.getChildren()) {
            collectNodes(child, list);
        }
    }

    public static SceneGraphResult toSceneGraph(RobotDefinitionDTO def) throws Exception {
        SceneGraphResult result = new SceneGraphResult();

        if (def == null || def.links == null) {
            return result;
        }

        for (LinkDTO link : def.links) {
            String resourcePath = link.mesh;
            if (!resourcePath.startsWith("/")) {
                resourcePath = "/" + resourcePath;
            }

            Mesh mesh = ObjLoader.load(resourcePath);
            SceneNode node = new SceneNode(link.id, mesh);
            node.setSourceMeshPath(link.mesh);
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

                Vector3f axis = (jDto.axis != null && jDto.axis.length >= 3) 
                        ? new Vector3f(jDto.axis[0], jDto.axis[1], jDto.axis[2]) 
                        : new Vector3f(0, 1, 0);

                Joint joint = new Joint(jDto.id, parentNode, childNode, axis);
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
}
