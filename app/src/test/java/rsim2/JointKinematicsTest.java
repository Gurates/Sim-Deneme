package rsim2;

import org.joml.Quaternionf;
import org.joml.Vector3f;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import rsim2.io.UrdfLoader;
import rsim2.scene.Joint;
import rsim2.scene.JointType;
import rsim2.scene.SceneNode;

import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class JointKinematicsTest {
    private static final float EPSILON = 0.00001f;

    @TempDir
    Path tempDir;

    @Test
    void jointRotatesInOriginFrameAndReturnsToOrigin() {
        SceneNode child = new SceneNode("link");
        child.getLocalPosition().set(1, 2, 3);
        child.getLocalRotation().rotationXYZ(0.3f, -0.4f, 0.7f);
        Quaternionf origin = new Quaternionf(child.getLocalRotation());
        Joint joint = new Joint("joint", null, child, new Vector3f(0, 0, 1));

        joint.setAngle(0.6f);

        assertRotationEquals(origin.mul(new Quaternionf().rotationZ(0.6f), new Quaternionf()), child.getLocalRotation());
        assertVectorEquals(new Vector3f(1, 2, 3), child.getLocalPosition());

        joint.setAngle(-0.2f);
        joint.setAngle(0.6f);

        assertRotationEquals(origin.mul(new Quaternionf().rotationZ(0.6f), new Quaternionf()), child.getLocalRotation());

        joint.setAngle(0);

        assertRotationEquals(origin, child.getLocalRotation());
        assertEquals(0, joint.getCurrentAngleRadians(), EPSILON);
    }

    @Test
    void identityOriginKeepsExistingRotationBehavior() {
        SceneNode child = new SceneNode("link");
        Joint joint = new Joint("joint", null, child, new Vector3f(0, 2, 0));

        joint.setAngle(0.8f);

        assertRotationEquals(new Quaternionf().rotationY(0.8f), child.getLocalRotation());
    }

    @Test
    void fixedJointKeepsOriginTransform() {
        SceneNode child = new SceneNode("link");
        child.getLocalRotation().rotationXYZ(0.3f, 0.4f, 0.5f);
        Quaternionf origin = new Quaternionf(child.getLocalRotation());
        Joint joint = new Joint("joint", null, child, JointType.FIXED, new Vector3f(0, 0, 1));

        joint.setAngle(0.8f);

        assertRotationEquals(origin, child.getLocalRotation());
        assertEquals(0, joint.getCurrentAngleRadians(), EPSILON);
    }

    @Test
    void importedUrdfChainPreservesOriginAndWorldCoordinates() throws Exception {
        Path urdf = tempDir.resolve("robot.urdf");
        Files.writeString(urdf, """
                <robot name="origin_test">
                  <link name="base"/>
                  <link name="arm"/>
                  <link name="tip"/>
                  <joint name="shoulder" type="revolute">
                    <parent link="base"/>
                    <child link="arm"/>
                    <origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/>
                    <axis xyz="1 0 0"/>
                    <limit lower="-3.14" upper="3.14" velocity="2"/>
                  </joint>
                  <joint name="tool" type="fixed">
                    <parent link="arm"/>
                    <child link="tip"/>
                    <origin xyz="0 1 0" rpy="0 0 0"/>
                  </joint>
                </robot>
                """);
        UrdfLoader.UrdfResult robot = UrdfLoader.load(urdf.toString(), false);
        Joint shoulder = robot.joints.get(0);
        SceneNode tip = robot.linksById.get("tip");

        assertVectorEquals(new Vector3f(0, 0, 0), tip.getWorldTransform().getTranslation(new Vector3f()));

        shoulder.setAngle((float) (Math.PI / 2));

        assertVectorEquals(new Vector3f(1, 0, 1), tip.getWorldTransform().getTranslation(new Vector3f()));
        assertVectorEquals(new Vector3f(0, 1, 0), robot.linksById.get("arm").getWorldTransform().transformDirection(new Vector3f(1, 0, 0)));

        shoulder.setAngle(0);

        assertVectorEquals(new Vector3f(0, 0, 0), tip.getWorldTransform().getTranslation(new Vector3f()));
    }

    private static void assertRotationEquals(Quaternionf expected, Quaternionf actual) {
        assertVectorEquals(expected.transform(new Vector3f(1, 0, 0)), actual.transform(new Vector3f(1, 0, 0)));
        assertVectorEquals(expected.transform(new Vector3f(0, 1, 0)), actual.transform(new Vector3f(0, 1, 0)));
        assertVectorEquals(expected.transform(new Vector3f(0, 0, 1)), actual.transform(new Vector3f(0, 0, 1)));
    }

    private static void assertVectorEquals(Vector3f expected, Vector3f actual) {
        assertEquals(expected.x, actual.x, EPSILON);
        assertEquals(expected.y, actual.y, EPSILON);
        assertEquals(expected.z, actual.z, EPSILON);
    }
}
