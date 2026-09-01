package rsim2.io;

import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import rsim2.collision.*;
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

public class UrdfLoader {

    public static class UrdfResult {
        public String robotName;
        public SceneNode rootNode;
        public List<Joint> joints = new ArrayList<>();
        public Map<String, SceneNode> linksById = new HashMap<>();
        public List<String[]> disabledCollisionPairs = new ArrayList<>();
    }

    public static UrdfResult load(String urdfFilePath) throws Exception {
        return load(urdfFilePath, true);
    }

    public static UrdfResult load(String urdfFilePath, boolean rotateZUpToYUp) throws Exception {
        File file = new File(urdfFilePath);
        if (!file.exists() || !file.isFile()) {
            throw new IllegalArgumentException("URDF file not found: " + urdfFilePath);
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
        if ("mujoco".equalsIgnoreCase(rootElem.getTagName())) {
            return MjcfLoader.load(urdfFilePath, rotateZUpToYUp);
        }

        String robotName = rootElem.getAttribute("name");
        if (robotName == null || robotName.trim().isEmpty()) {
            robotName = "URDF_Robot";
        }

        UrdfResult result = new UrdfResult();
        result.robotName = robotName;

        NodeList linkNodes = rootElem.getElementsByTagName("link");
        for (int i = 0; i < linkNodes.getLength(); i++) {
            Node lNode = linkNodes.item(i);
            if (lNode.getNodeType() != Node.ELEMENT_NODE)
                continue;
            Element linkElem = (Element) lNode;

            String linkName = linkElem.getAttribute("name");
            if (linkName == null || linkName.trim().isEmpty()) {
                linkName = "link_" + i;
            }

            List<Mesh> meshes = new ArrayList<>();
            Vector3f linkScale = new Vector3f(1.0f, 1.0f, 1.0f);

            NodeList visualNodes = linkElem.getElementsByTagName("visual");
            for (int v = 0; v < visualNodes.getLength(); v++) {
                Node vNode = visualNodes.item(v);
                if (vNode.getNodeType() != Node.ELEMENT_NODE)
                    continue;
                Element visualElem = (Element) vNode;

                NodeList geomNodes = visualElem.getElementsByTagName("geometry");
                if (geomNodes.getLength() > 0) {
                    Element geomElem = (Element) geomNodes.item(0);

                    NodeList meshNodes = geomElem.getElementsByTagName("mesh");
                    if (meshNodes.getLength() > 0) {
                        Element meshElem = (Element) meshNodes.item(0);
                        String filename = meshElem.getAttribute("filename");
                        String scaleStr = meshElem.getAttribute("scale");

                        if (scaleStr != null && !scaleStr.trim().isEmpty()) {
                            float[] s = parseFloats(scaleStr, 3);
                            linkScale.set(s[0], s[1], s[2]);
                        }

                        if (filename != null && !filename.trim().isEmpty()) {
                            File resolvedFile = resolveMeshFile(filename, baseDir);
                            if (resolvedFile != null && resolvedFile.exists()) {
                                try {
                                    String path = resolvedFile.getAbsolutePath();
                                    Mesh m;
                                    String lower = path.toLowerCase();
                                    if (lower.endsWith(".stl")) {
                                        m = StlLoader.load(path);
                                    } else {
                                        m = ObjLoader.load(path);
                                    }
                                    if (m != null) {
                                        m.setSourcePath(path);
                                        meshes.add(m);
                                    }
                                } catch (Exception ex) {
                                    System.err.println("Warning: failed to load mesh: " + resolvedFile.getAbsolutePath()
                                            + " (" + ex.getMessage() + ")");
                                }
                            } else {
                                System.err.println("Warning: could not resolve mesh file for: " + filename);
                            }
                        }
                    }
                }
            }

            SceneNode sceneNode = new SceneNode(linkName, meshes);
            sceneNode.getLocalScale().set(linkScale);

            if (!meshes.isEmpty() && meshes.get(0).getSourcePath() != null) {
                sceneNode.setSourceMeshPath(meshes.get(0).getSourcePath());
            }

            CompoundShape compoundShape = parseCollisionShapes(linkElem, baseDir, meshes);
            if (compoundShape != null && !compoundShape.isEmpty()) {
                sceneNode.setCollisionShape(compoundShape);
            }

            result.linksById.put(linkName, sceneNode);
        }

        NodeList disableNodes = rootElem.getElementsByTagName("disable_collisions");
        for (int i = 0; i < disableNodes.getLength(); i++) {
            Node dNode = disableNodes.item(i);
            if (dNode.getNodeType() != Node.ELEMENT_NODE) continue;
            Element disElem = (Element) dNode;
            String link1 = disElem.getAttribute("link1");
            String link2 = disElem.getAttribute("link2");
            if (link1 != null && link2 != null && !link1.trim().isEmpty() && !link2.trim().isEmpty()) {
                result.disabledCollisionPairs.add(new String[]{link1.trim(), link2.trim()});
            }
        }

        NodeList jointNodes = rootElem.getElementsByTagName("joint");
        for (int i = 0; i < jointNodes.getLength(); i++) {
            Node jNode = jointNodes.item(i);
            if (jNode.getNodeType() != Node.ELEMENT_NODE)
                continue;
            Element jointElem = (Element) jNode;

            String jointName = jointElem.getAttribute("name");
            String jointTypeStr = jointElem.getAttribute("type");

            if (jointName == null || jointName.trim().isEmpty()) {
                jointName = "joint_" + i;
            }

            JointType type = JointType.REVOLUTE;
            if ("fixed".equalsIgnoreCase(jointTypeStr)) {
                type = JointType.FIXED;
            }

            String parentName = "";
            NodeList parentNodes = jointElem.getElementsByTagName("parent");
            if (parentNodes.getLength() > 0) {
                parentName = ((Element) parentNodes.item(0)).getAttribute("link");
            }

            String childName = "";
            NodeList childNodes = jointElem.getElementsByTagName("child");
            if (childNodes.getLength() > 0) {
                childName = ((Element) childNodes.item(0)).getAttribute("link");
            }

            SceneNode parentNode = result.linksById.get(parentName);
            SceneNode childNode = result.linksById.get(childName);

            Vector3f originPos = new Vector3f(0.0f, 0.0f, 0.0f);
            Vector3f originRpy = new Vector3f(0.0f, 0.0f, 0.0f);

            NodeList originNodes = jointElem.getElementsByTagName("origin");
            if (originNodes.getLength() > 0) {
                Element originElem = (Element) originNodes.item(0);
                String xyzStr = originElem.getAttribute("xyz");
                if (xyzStr != null && !xyzStr.trim().isEmpty()) {
                    float[] xyz = parseFloats(xyzStr, 3);
                    originPos.set(xyz[0], xyz[1], xyz[2]);
                }

                String rpyStr = originElem.getAttribute("rpy");
                if (rpyStr != null && !rpyStr.trim().isEmpty()) {
                    float[] rpy = parseFloats(rpyStr, 3);
                    originRpy.set(rpy[0], rpy[1], rpy[2]);
                }
            }

            Vector3f axis = new Vector3f(0.0f, 0.0f, 1.0f);
            NodeList axisNodes = jointElem.getElementsByTagName("axis");
            if (axisNodes.getLength() > 0) {
                Element axisElem = (Element) axisNodes.item(0);
                String axisStr = axisElem.getAttribute("xyz");
                if (axisStr != null && !axisStr.trim().isEmpty()) {
                    float[] ax = parseFloats(axisStr, 3);
                    axis.set(ax[0], ax[1], ax[2]);
                }
            }

            float lowerLimit = -(float) Math.PI;
            float upperLimit = (float) Math.PI;
            float maxSpeed = 2.0f;

            NodeList limitNodes = jointElem.getElementsByTagName("limit");
            if (limitNodes.getLength() > 0) {
                Element limitElem = (Element) limitNodes.item(0);
                String lowerStr = limitElem.getAttribute("lower");
                String upperStr = limitElem.getAttribute("upper");
                String velStr = limitElem.getAttribute("velocity");

                if (lowerStr != null && !lowerStr.trim().isEmpty()) {
                    try {
                        lowerLimit = Float.parseFloat(lowerStr.trim());
                    } catch (Exception ignored) {
                    }
                }
                if (upperStr != null && !upperStr.trim().isEmpty()) {
                    try {
                        upperLimit = Float.parseFloat(upperStr.trim());
                    } catch (Exception ignored) {
                    }
                }
                if (velStr != null && !velStr.trim().isEmpty()) {
                    try {
                        maxSpeed = Float.parseFloat(velStr.trim());
                    } catch (Exception ignored) {
                    }
                }
            }

            if (childNode != null) {
                childNode.getLocalPosition().set(originPos);
                childNode.getLocalRotation().rotationXYZ(originRpy.x, originRpy.y, originRpy.z);

                if (parentNode != null) {
                    parentNode.addChild(childNode);
                }

                Joint joint = new Joint(jointName, parentNode, childNode, type, axis);
                if (type == JointType.REVOLUTE) {
                    joint.setLimits(lowerLimit, upperLimit, maxSpeed > 0 ? maxSpeed : 2.0f);
                }
                result.joints.add(joint);
            }
        }

        for (SceneNode node : result.linksById.values()) {
            if (node.getParent() == null) {
                result.rootNode = node;
                break;
            }
        }

        if (result.rootNode == null && !result.linksById.isEmpty()) {
            result.rootNode = result.linksById.values().iterator().next();
        }

        if (rotateZUpToYUp && result.rootNode != null) {
            result.rootNode.getLocalRotation().rotateX((float) Math.toRadians(-90));
        }

        return result;
    }

    private static float[] parseFloats(String str, int count) {
        float[] res = new float[count];
        Arrays.fill(res, 1.0f);
        if (str == null)
            return res;

        String[] parts = str.trim().split("\\s+");
        for (int i = 0; i < Math.min(parts.length, count); i++) {
            try {
                res[i] = Float.parseFloat(parts[i].trim());
            } catch (Exception ignored) {
            }
        }
        return res;
    }

    private static float parseSingleFloat(String str, float defaultValue) {
        if (str == null || str.trim().isEmpty()) return defaultValue;
        try {
            return Float.parseFloat(str.trim());
        } catch (Exception e) {
            return defaultValue;
        }
    }

    private static CompoundShape parseCollisionShapes(Element linkElem, File baseDir, List<Mesh> fallbackVisualMeshes) {
        CompoundShape compound = new CompoundShape();
        if (linkElem == null) return compound;

        NodeList collisionNodes = linkElem.getElementsByTagName("collision");
        for (int c = 0; c < collisionNodes.getLength(); c++) {
            Node cNode = collisionNodes.item(c);
            if (cNode.getNodeType() != Node.ELEMENT_NODE) continue;
            Element colElem = (Element) cNode;

            Matrix4f localTransform = new Matrix4f();
            NodeList originNodes = colElem.getElementsByTagName("origin");
            if (originNodes.getLength() > 0) {
                Element originElem = (Element) originNodes.item(0);
                String xyzStr = originElem.getAttribute("xyz");
                String rpyStr = originElem.getAttribute("rpy");
                Vector3f pos = new Vector3f(0, 0, 0);
                if (xyzStr != null && !xyzStr.trim().isEmpty()) {
                    float[] xyz = parseFloats(xyzStr, 3);
                    pos.set(xyz[0], xyz[1], xyz[2]);
                }
                if (rpyStr != null && !rpyStr.trim().isEmpty()) {
                    float[] rpy = parseFloats(rpyStr, 3);
                    Quaternionf rot = new Quaternionf().rotationXYZ(rpy[0], rpy[1], rpy[2]);
                    localTransform.translationRotate(pos, rot);
                } else {
                    localTransform.translation(pos);
                }
            }

            NodeList geomNodes = colElem.getElementsByTagName("geometry");
            if (geomNodes.getLength() > 0) {
                Element geomElem = (Element) geomNodes.item(0);

                NodeList boxNodes = geomElem.getElementsByTagName("box");
                if (boxNodes.getLength() > 0) {
                    Element boxElem = (Element) boxNodes.item(0);
                    String sizeStr = boxElem.getAttribute("size");
                    if (sizeStr != null && !sizeStr.trim().isEmpty()) {
                        float[] sz = parseFloats(sizeStr, 3);
                        float hx = sz[0] * 0.5f, hy = sz[1] * 0.5f, hz = sz[2] * 0.5f;
                        compound.addShape(new BoxShape(new Vector3f(-hx, -hy, -hz), new Vector3f(hx, hy, hz)), localTransform);
                    }
                }

                NodeList cylNodes = geomElem.getElementsByTagName("cylinder");
                if (cylNodes.getLength() > 0) {
                    Element cylElem = (Element) cylNodes.item(0);
                    float radius = parseSingleFloat(cylElem.getAttribute("radius"), 0.05f);
                    float length = parseSingleFloat(cylElem.getAttribute("length"), 0.1f);
                    compound.addShape(new CylinderShape(radius, length), localTransform);
                }

                NodeList sphNodes = geomElem.getElementsByTagName("sphere");
                if (sphNodes.getLength() > 0) {
                    Element sphElem = (Element) sphNodes.item(0);
                    float radius = parseSingleFloat(sphElem.getAttribute("radius"), 0.05f);
                    compound.addShape(new SphereShape(radius), localTransform);
                }

                NodeList capNodes = geomElem.getElementsByTagName("capsule");
                if (capNodes.getLength() > 0) {
                    Element capElem = (Element) capNodes.item(0);
                    float radius = parseSingleFloat(capElem.getAttribute("radius"), 0.05f);
                    float length = parseSingleFloat(capElem.getAttribute("length"), 0.1f);
                    compound.addShape(new CapsuleShape(radius, length), localTransform);
                }

                NodeList meshNodes = geomElem.getElementsByTagName("mesh");
                if (meshNodes.getLength() > 0) {
                    Element meshElem = (Element) meshNodes.item(0);
                    String filename = meshElem.getAttribute("filename");
                    String scaleStr = meshElem.getAttribute("scale");
                    Vector3f scale = new Vector3f(1, 1, 1);
                    if (scaleStr != null && !scaleStr.trim().isEmpty()) {
                        float[] s = parseFloats(scaleStr, 3);
                        scale.set(s[0], s[1], s[2]);
                    }
                    if (filename != null && !filename.trim().isEmpty()) {
                        File resolved = resolveMeshFile(filename, baseDir);
                        if (resolved != null && resolved.exists()) {
                            try {
                                String path = resolved.getAbsolutePath();
                                Mesh m = path.toLowerCase().endsWith(".stl") ? StlLoader.load(path) : ObjLoader.load(path);
                                if (m != null) {
                                    if (scale.x != 1 || scale.y != 1 || scale.z != 1) {
                                        m = Mesh.createScaled(m, scale);
                                    }
                                    compound.addShape(new ConvexMeshShape(m), localTransform);
                                }
                            } catch (Exception ex) {
                                System.err.println("Warning: could not load collision mesh: " + filename);
                            }
                        }
                    }
                }
            }
        }

        if (compound.isEmpty() && fallbackVisualMeshes != null && !fallbackVisualMeshes.isEmpty()) {
            for (Mesh vMesh : fallbackVisualMeshes) {
                compound.addShape(new ConvexMeshShape(vMesh));
            }
        }

        return compound;
    }

    private static File resolveMeshFile(String filename, File baseDir) {
        if (filename == null || filename.trim().isEmpty()) {
            return null;
        }

        String cleaned = filename.trim();
        if (cleaned.startsWith("file://")) {
            cleaned = cleaned.substring(7);
        }

        File direct = new File(cleaned);
        if (direct.isAbsolute() && direct.exists()) {
            return direct;
        }

        if (cleaned.startsWith("package://")) {
            String pathWithoutPackagePrefix = cleaned.substring(10);
            int slashIdx = pathWithoutPackagePrefix.indexOf('/');
            String subPath = (slashIdx >= 0) ? pathWithoutPackagePrefix.substring(slashIdx + 1)
                    : pathWithoutPackagePrefix;

            File try1 = new File(baseDir, subPath);
            if (try1.exists())
                return try1;

            File try2 = new File(baseDir, pathWithoutPackagePrefix);
            if (try2.exists())
                return try2;

            if (baseDir.getParentFile() != null) {
                File try3 = new File(baseDir.getParentFile(), subPath);
                if (try3.exists())
                    return try3;

                File try4 = new File(baseDir.getParentFile(), pathWithoutPackagePrefix);
                if (try4.exists())
                    return try4;
            }

            File found = findFileRecursively(baseDir, new File(cleaned).getName());
            if (found != null)
                return found;
        }

        File rel = new File(baseDir, cleaned);
        if (rel.exists())
            return rel;

        File inMeshes = new File(baseDir, "meshes/" + new File(cleaned).getName());
        if (inMeshes.exists())
            return inMeshes;

        File inVisual = new File(baseDir, "meshes/visual/" + new File(cleaned).getName());
        if (inVisual.exists())
            return inVisual;

        File found = findFileRecursively(baseDir, new File(cleaned).getName());
        if (found != null)
            return found;

        String lowerName = new File(cleaned).getName().toLowerCase();
        if (lowerName.endsWith(".dae")) {
            String baseNoExt = lowerName.substring(0, lowerName.length() - 4);
            File stlMatch = findFileRecursively(baseDir, baseNoExt + ".stl");
            if (stlMatch != null)
                return stlMatch;

            File objMatch = findFileRecursively(baseDir, baseNoExt + ".obj");
            if (objMatch != null)
                return objMatch;
        }

        return rel;
    }

    private static File findFileRecursively(File dir, String targetFileName) {
        if (dir == null || !dir.exists() || !dir.isDirectory())
            return null;

        File[] files = dir.listFiles();
        if (files == null)
            return null;

        for (File f : files) {
            if (f.isFile() && f.getName().equalsIgnoreCase(targetFileName)) {
                return f;
            }
        }

        for (File f : files) {
            if (f.isDirectory()) {
                File found = findFileRecursively(f, targetFileName);
                if (found != null)
                    return found;
            }
        }

        return null;
    }
}
