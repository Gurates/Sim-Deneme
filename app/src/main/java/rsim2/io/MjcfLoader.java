package rsim2.io;

import org.joml.Quaternionf;
import org.joml.Vector3f;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NamedNodeMap;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import rsim2.graphics.Mesh;
import rsim2.scene.Joint;
import rsim2.scene.JointType;
import rsim2.scene.SceneNode;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.util.*;

public class MjcfLoader {

    public static class MeshAssetInfo {
        public String filePath;
        public Vector3f scale = new Vector3f(1.0f, 1.0f, 1.0f);
    }

    public static UrdfLoader.UrdfResult load(String mjcfFilePath, boolean rotateZUpToYUp) throws Exception {
        File file = new File(mjcfFilePath);
        if (!file.exists() || !file.isFile()) {
            throw new IllegalArgumentException("MuJoCo XML file not found: " + mjcfFilePath);
        }

        File baseDir = file.getParentFile();

        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setNamespaceAware(false);
        DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();

        Document doc;
        try (InputStream is = new FileInputStream(file)) {
            doc = dBuilder.parse(is);
        }
        doc.getDocumentElement().normalize();

        Element rootElem = doc.getDocumentElement();
        String modelName = rootElem.getAttribute("model");
        if (modelName == null || modelName.trim().isEmpty()) {
            modelName = "MuJoCo_Robot";
        }

        UrdfLoader.UrdfResult result = new UrdfLoader.UrdfResult();
        result.robotName = modelName;

        boolean isDegree = false;
        String customMeshDir = null;

        NodeList compilerNodes = rootElem.getElementsByTagName("compiler");
        if (compilerNodes.getLength() > 0) {
            Element compElem = (Element) compilerNodes.item(0);
            String angleAttr = compElem.getAttribute("angle");
            if ("degree".equalsIgnoreCase(angleAttr)) {
                isDegree = true;
            }
            String meshdirAttr = compElem.getAttribute("meshdir");
            if (meshdirAttr != null && !meshdirAttr.trim().isEmpty()) {
                customMeshDir = meshdirAttr.trim();
            }
        }

        Map<String, Map<String, Map<String, String>>> classDefaults = new HashMap<>();
        Map<String, Map<String, String>> globalDefaults = new HashMap<>();

        NodeList rootChildren = rootElem.getChildNodes();
        for (int i = 0; i < rootChildren.getLength(); i++) {
            Node n = rootChildren.item(i);
            if (n.getNodeType() == Node.ELEMENT_NODE && "default".equalsIgnoreCase(n.getNodeName())) {
                parseDefaultTree((Element) n, null, classDefaults, globalDefaults);
            }
        }

        Map<String, MeshAssetInfo> assetMeshes = new HashMap<>();
        NodeList assetNodes = rootElem.getElementsByTagName("asset");
        for (int a = 0; a < assetNodes.getLength(); a++) {
            Element assetElem = (Element) assetNodes.item(a);
            NodeList meshNodes = assetElem.getElementsByTagName("mesh");
            for (int m = 0; m < meshNodes.getLength(); m++) {
                Element meshElem = (Element) meshNodes.item(m);
                String fileAttr = resolveAttribute(meshElem, "mesh", "file", null, classDefaults, globalDefaults);
                if (fileAttr == null || fileAttr.isEmpty()) {
                    fileAttr = meshElem.getAttribute("file");
                }
                String nameAttr = meshElem.getAttribute("name");

                String scaleAttr = resolveAttribute(meshElem, "mesh", "scale", null, classDefaults, globalDefaults);

                if (fileAttr != null && !fileAttr.trim().isEmpty()) {
                    MeshAssetInfo info = new MeshAssetInfo();
                    info.filePath = fileAttr.trim();

                    if (scaleAttr != null && !scaleAttr.trim().isEmpty()) {
                        float[] s = parseFloats(scaleAttr, 3, 1.0f);
                        info.scale.set(s[0], s[1], s[2]);
                    } else {
                        info.scale.set(1.0f, 1.0f, 1.0f);
                    }

                    if (nameAttr != null && !nameAttr.trim().isEmpty()) {
                        assetMeshes.put(nameAttr.trim(), info);
                    }

                    File f = new File(fileAttr.trim());
                    String fileName = f.getName();
                    assetMeshes.put(fileName, info);

                    int dotIdx = fileName.lastIndexOf('.');
                    if (dotIdx > 0) {
                        String nameNoExt = fileName.substring(0, dotIdx);
                        assetMeshes.put(nameNoExt, info);
                    }
                }
            }
        }

        NodeList worldbodyNodes = rootElem.getElementsByTagName("worldbody");
        if (worldbodyNodes.getLength() == 0) {
            throw new IllegalArgumentException("No <worldbody> found in MuJoCo XML");
        }

        Element worldbodyElem = (Element) worldbodyNodes.item(0);
        String worldChildClass = worldbodyElem.getAttribute("childclass");
        if (worldChildClass != null && !worldChildClass.trim().isEmpty()) {
            worldChildClass = worldChildClass.trim();
        } else {
            worldChildClass = null;
        }

        NodeList childNodes = worldbodyElem.getChildNodes();
        List<SceneNode> topBodies = new ArrayList<>();

        for (int i = 0; i < childNodes.getLength(); i++) {
            Node n = childNodes.item(i);
            if (n.getNodeType() == Node.ELEMENT_NODE && "body".equalsIgnoreCase(n.getNodeName())) {
                SceneNode topNode = parseBody((Element) n, null, baseDir, customMeshDir, assetMeshes, isDegree,
                        worldChildClass, classDefaults, globalDefaults, result);
                if (topNode != null) {
                    topBodies.add(topNode);
                }
            }
        }

        if (!topBodies.isEmpty()) {
            result.rootNode = topBodies.get(0);
            if (topBodies.size() > 1) {
                SceneNode worldRoot = new SceneNode("world");
                for (SceneNode tb : topBodies) {
                    worldRoot.addChild(tb);
                }
                result.rootNode = worldRoot;
            }
        }

        if (rotateZUpToYUp && result.rootNode != null) {
            result.rootNode.getLocalRotation().rotateX((float) Math.toRadians(-90));
        }

        return result;
    }

    private static void parseDefaultTree(Element defaultElem, String parentClass,
                                        Map<String, Map<String, Map<String, String>>> classDefaults,
                                        Map<String, Map<String, String>> globalDefaults) {
        String currentClass = defaultElem.getAttribute("class");
        if (currentClass != null) {
            currentClass = currentClass.trim();
            if (currentClass.isEmpty()) {
                currentClass = null;
            }
        }

        String effectiveClass = (currentClass != null) ? currentClass : parentClass;

        if (effectiveClass != null) {
            Map<String, Map<String, String>> classMap = classDefaults.computeIfAbsent(effectiveClass, k -> new HashMap<>());

            if (parentClass != null && !effectiveClass.equals(parentClass)) {
                Map<String, Map<String, String>> parentMap = classDefaults.get(parentClass);
                if (parentMap != null) {
                    for (Map.Entry<String, Map<String, String>> tagEntry : parentMap.entrySet()) {
                        Map<String, String> currentTagMap = classMap.computeIfAbsent(tagEntry.getKey(), k -> new HashMap<>());
                        for (Map.Entry<String, String> attrEntry : tagEntry.getValue().entrySet()) {
                            currentTagMap.putIfAbsent(attrEntry.getKey(), attrEntry.getValue());
                        }
                    }
                }
            }

            NodeList children = defaultElem.getChildNodes();
            for (int i = 0; i < children.getLength(); i++) {
                Node node = children.item(i);
                if (node.getNodeType() == Node.ELEMENT_NODE) {
                    Element childElem = (Element) node;
                    String tagName = childElem.getNodeName().toLowerCase();
                    if ("default".equals(tagName)) {
                        parseDefaultTree(childElem, effectiveClass, classDefaults, globalDefaults);
                    } else {
                        Map<String, String> tagMap = classMap.computeIfAbsent(tagName, k -> new HashMap<>());
                        NamedNodeMap attrs = childElem.getAttributes();
                        for (int a = 0; a < attrs.getLength(); a++) {
                            Node attr = attrs.item(a);
                            tagMap.put(attr.getNodeName(), attr.getNodeValue());
                        }
                    }
                }
            }
        } else {
            NodeList children = defaultElem.getChildNodes();
            for (int i = 0; i < children.getLength(); i++) {
                Node node = children.item(i);
                if (node.getNodeType() == Node.ELEMENT_NODE) {
                    Element childElem = (Element) node;
                    String tagName = childElem.getNodeName().toLowerCase();
                    if ("default".equals(tagName)) {
                        parseDefaultTree(childElem, null, classDefaults, globalDefaults);
                    } else {
                        Map<String, String> tagMap = globalDefaults.computeIfAbsent(tagName, k -> new HashMap<>());
                        NamedNodeMap attrs = childElem.getAttributes();
                        for (int a = 0; a < attrs.getLength(); a++) {
                            Node attr = attrs.item(a);
                            tagMap.put(attr.getNodeName(), attr.getNodeValue());
                        }
                    }
                }
            }
        }
    }

    private static String resolveAttribute(Element elem, String tagName, String attrName, String activeClass,
                                           Map<String, Map<String, Map<String, String>>> classDefaults,
                                           Map<String, Map<String, String>> globalDefaults) {
        if (elem != null && elem.hasAttribute(attrName)) {
            String val = elem.getAttribute(attrName);
            if (val != null && !val.trim().isEmpty()) {
                return val.trim();
            }
        }

        String elemClass = (elem != null && elem.hasAttribute("class")) ? elem.getAttribute("class").trim() : null;
        if (elemClass != null && !elemClass.isEmpty()) {
            Map<String, Map<String, String>> cMap = classDefaults.get(elemClass);
            if (cMap != null) {
                Map<String, String> tagMap = cMap.get(tagName.toLowerCase());
                if (tagMap != null && tagMap.containsKey(attrName)) {
                    return tagMap.get(attrName);
                }
            }
        }

        if (activeClass != null && !activeClass.isEmpty()) {
            Map<String, Map<String, String>> cMap = classDefaults.get(activeClass);
            if (cMap != null) {
                Map<String, String> tagMap = cMap.get(tagName.toLowerCase());
                if (tagMap != null && tagMap.containsKey(attrName)) {
                    return tagMap.get(attrName);
                }
            }
        }

        if (globalDefaults != null) {
            Map<String, String> tagMap = globalDefaults.get(tagName.toLowerCase());
            if (tagMap != null && tagMap.containsKey(attrName)) {
                return tagMap.get(attrName);
            }
        }

        return null;
    }

    private static SceneNode parseBody(Element bodyElem, SceneNode parentNode, File baseDir, String customMeshDir,
                                       Map<String, MeshAssetInfo> assetMeshes, boolean isDegree,
                                       String parentChildClass,
                                       Map<String, Map<String, Map<String, String>>> classDefaults,
                                       Map<String, Map<String, String>> globalDefaults,
                                       UrdfLoader.UrdfResult result) {
        String bodyName = bodyElem.getAttribute("name");
        if (bodyName == null || bodyName.trim().isEmpty()) {
            bodyName = "body_" + UUID.randomUUID().toString().substring(0, 8);
        }

        String bodyChildClass = bodyElem.getAttribute("childclass");
        if (bodyChildClass == null || bodyChildClass.trim().isEmpty()) {
            bodyChildClass = parentChildClass;
        } else {
            bodyChildClass = bodyChildClass.trim();
        }

        Vector3f pos = new Vector3f(0.0f, 0.0f, 0.0f);
        String posAttr = bodyElem.getAttribute("pos");
        if (posAttr != null && !posAttr.trim().isEmpty()) {
            float[] p = parseFloats(posAttr, 3, 0.0f);
            pos.set(p[0], p[1], p[2]);
        }

        Quaternionf rot = new Quaternionf();
        String quatAttr = bodyElem.getAttribute("quat");
        if (quatAttr != null && !quatAttr.trim().isEmpty()) {
            float[] q = parseFloats(quatAttr, 4, 0.0f);
            rot.set(q[1], q[2], q[3], q[0]);
            rot.normalize();
        } else {
            String eulerAttr = bodyElem.getAttribute("euler");
            if (eulerAttr != null && !eulerAttr.trim().isEmpty()) {
                float[] e = parseFloats(eulerAttr, 3, 0.0f);
                if (isDegree) {
                    rot.rotationXYZ((float) Math.toRadians(e[0]), (float) Math.toRadians(e[1]), (float) Math.toRadians(e[2]));
                } else {
                    rot.rotationXYZ(e[0], e[1], e[2]);
                }
            }
        }

        List<Mesh> bodyMeshes = new ArrayList<>();
        NodeList geomNodes = bodyElem.getChildNodes();
        for (int i = 0; i < geomNodes.getLength(); i++) {
            Node gNode = geomNodes.item(i);
            if (gNode.getNodeType() != Node.ELEMENT_NODE || !"geom".equalsIgnoreCase(gNode.getNodeName())) {
                continue;
            }
            Element geomElem = (Element) gNode;

            String geomClass = geomElem.getAttribute("class");
            if (geomClass == null || geomClass.trim().isEmpty()) {
                geomClass = bodyChildClass;
            } else {
                geomClass = geomClass.trim();
            }

            String geomMeshName = resolveAttribute(geomElem, "geom", "mesh", geomClass, classDefaults, globalDefaults);
            if (geomMeshName == null || geomMeshName.isEmpty()) {
                geomMeshName = geomElem.getAttribute("mesh");
            }

            if (geomMeshName != null && !geomMeshName.trim().isEmpty()) {
                MeshAssetInfo assetInfo = assetMeshes.get(geomMeshName.trim());
                String meshFileToLoad = assetInfo != null ? assetInfo.filePath : geomMeshName.trim();
                Vector3f meshScale = assetInfo != null ? assetInfo.scale : new Vector3f(1.0f, 1.0f, 1.0f);

                File resolved = resolveMjcfMesh(meshFileToLoad, baseDir, customMeshDir);
                if (resolved != null && resolved.exists()) {
                    try {
                        String path = resolved.getAbsolutePath();
                        Mesh m;
                        String lower = path.toLowerCase();
                        if (lower.endsWith(".stl")) {
                            m = StlLoader.load(path);
                        } else {
                            m = ObjLoader.load(path);
                        }
                        if (m != null) {
                            if (meshScale != null && (meshScale.x != 1.0f || meshScale.y != 1.0f || meshScale.z != 1.0f)) {
                                m = Mesh.createScaled(m, meshScale);
                            }
                            m.setSourcePath(path);
                            bodyMeshes.add(m);
                        }
                    } catch (Exception ex) {
                        System.err.println("Warning: failed to load MuJoCo mesh: " + resolved.getAbsolutePath() + " (" + ex.getMessage() + ")");
                    }
                } else {
                    System.err.println("Warning: could not resolve MuJoCo mesh file for: " + geomMeshName);
                }
            }
        }

        SceneNode bodyNode = new SceneNode(bodyName, bodyMeshes);
        bodyNode.getLocalPosition().set(pos);
        bodyNode.getLocalRotation().set(rot);

        if (!bodyMeshes.isEmpty() && bodyMeshes.get(0).getSourcePath() != null) {
            bodyNode.setSourceMeshPath(bodyMeshes.get(0).getSourcePath());
        }

        result.linksById.put(bodyName, bodyNode);

        if (parentNode != null) {
            NodeList bChildren = bodyElem.getChildNodes();
            for (int i = 0; i < bChildren.getLength(); i++) {
                Node jNode = bChildren.item(i);
                if (jNode.getNodeType() != Node.ELEMENT_NODE || !"joint".equalsIgnoreCase(jNode.getNodeName())) {
                    continue;
                }
                Element jointElem = (Element) jNode;
                String jointName = jointElem.getAttribute("name");
                if (jointName == null || jointName.trim().isEmpty()) {
                    jointName = parentNode.getId() + "_" + bodyName + "_joint";
                }

                String jointClass = jointElem.getAttribute("class");
                if (jointClass == null || jointClass.trim().isEmpty()) {
                    jointClass = bodyChildClass;
                } else {
                    jointClass = jointClass.trim();
                }

                Vector3f axis = new Vector3f(0.0f, 0.0f, 1.0f);
                String axisAttr = resolveAttribute(jointElem, "joint", "axis", jointClass, classDefaults, globalDefaults);
                if (axisAttr == null || axisAttr.isEmpty()) {
                    axisAttr = jointElem.getAttribute("axis");
                }
                if (axisAttr != null && !axisAttr.trim().isEmpty()) {
                    float[] ax = parseFloats(axisAttr, 3, 0.0f);
                    axis.set(ax[0], ax[1], ax[2]);
                }

                float lowerLimit = -(float) Math.PI;
                float upperLimit = (float) Math.PI;
                String rangeAttr = resolveAttribute(jointElem, "joint", "range", jointClass, classDefaults, globalDefaults);
                if (rangeAttr == null || rangeAttr.isEmpty()) {
                    rangeAttr = jointElem.getAttribute("range");
                }
                if (rangeAttr != null && !rangeAttr.trim().isEmpty()) {
                    float[] r = parseFloats(rangeAttr, 2, 0.0f);
                    if (isDegree) {
                        lowerLimit = (float) Math.toRadians(r[0]);
                        upperLimit = (float) Math.toRadians(r[1]);
                    } else {
                        lowerLimit = r[0];
                        upperLimit = r[1];
                    }
                }

                Joint joint = new Joint(jointName, parentNode, bodyNode, JointType.REVOLUTE, axis);
                joint.setLimits(lowerLimit, upperLimit, 2.5f);
                result.joints.add(joint);
                break;
            }

            parentNode.addChild(bodyNode);
        }

        NodeList bodyChildren = bodyElem.getChildNodes();
        for (int i = 0; i < bodyChildren.getLength(); i++) {
            Node n = bodyChildren.item(i);
            if (n.getNodeType() == Node.ELEMENT_NODE && "body".equalsIgnoreCase(n.getNodeName())) {
                parseBody((Element) n, bodyNode, baseDir, customMeshDir, assetMeshes, isDegree,
                        bodyChildClass, classDefaults, globalDefaults, result);
            }
        }

        return bodyNode;
    }

    private static File resolveMjcfMesh(String meshPath, File baseDir, String customMeshDir) {
        if (meshPath == null || meshPath.trim().isEmpty()) return null;
        String clean = meshPath.trim();

        File direct = new File(clean);
        if (direct.isAbsolute() && direct.exists()) return direct;

        File rel = new File(baseDir, clean);
        if (rel.exists()) return rel;

        if (customMeshDir != null && !customMeshDir.isEmpty()) {
            File inCustom = new File(baseDir, customMeshDir + "/" + clean);
            if (inCustom.exists()) return inCustom;

            File inCustomDirect = new File(baseDir, customMeshDir + "/" + new File(clean).getName());
            if (inCustomDirect.exists()) return inCustomDirect;
        }

        File inModels = new File(baseDir, "models/" + new File(clean).getName());
        if (inModels.exists()) return inModels;

        File inAssets = new File(baseDir, "assets/" + new File(clean).getName());
        if (inAssets.exists()) return inAssets;

        File inMeshes = new File(baseDir, "meshes/" + new File(clean).getName());
        if (inMeshes.exists()) return inMeshes;

        File recursive = findFileRecursively(baseDir, new File(clean).getName());
        if (recursive != null) return recursive;

        String name = new File(clean).getName();
        if (!name.contains(".")) {
            File stl = findFileRecursively(baseDir, name + ".stl");
            if (stl != null) return stl;
            File obj = findFileRecursively(baseDir, name + ".obj");
            if (obj != null) return obj;
        }

        return rel;
    }

    private static File findFileRecursively(File dir, String targetFileName) {
        if (dir == null || !dir.exists() || !dir.isDirectory()) return null;

        File[] files = dir.listFiles();
        if (files == null) return null;

        for (File f : files) {
            if (f.isFile() && f.getName().equalsIgnoreCase(targetFileName)) {
                return f;
            }
        }

        for (File f : files) {
            if (f.isDirectory()) {
                File found = findFileRecursively(f, targetFileName);
                if (found != null) return found;
            }
        }

        return null;
    }

    private static float[] parseFloats(String str, int count, float defaultVal) {
        float[] res = new float[count];
        Arrays.fill(res, defaultVal);
        if (str == null) return res;

        String[] parts = str.trim().split("\\s+");
        for (int i = 0; i < Math.min(parts.length, count); i++) {
            try {
                res[i] = Float.parseFloat(parts[i].trim());
            } catch (Exception ignored) {
            }
        }
        return res;
    }
}
