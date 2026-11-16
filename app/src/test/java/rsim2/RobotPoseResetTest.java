package rsim2;

import org.joml.Quaternionf;
import org.joml.Vector3f;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import rsim2.core.Engine;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.*;

class RobotPoseResetTest {
    @TempDir Path tempDir;

    @Test void restoresImportedPoseAndHoldsItOnTheNextMotorUpdate() throws Exception {
        Path urdf = Files.writeString(tempDir.resolve("arm.urdf"), """
                <robot name="arm">
                    <link name="base"/>
                    <link name="tool"/>
                    <joint name="wrist" type="revolute">
                        <parent link="base"/>
                        <child link="tool"/>
                        <origin xyz="0 0 1"/>
                        <axis xyz="0 0 1"/>
                        <limit lower="0.3" upper="1.5" velocity="2"/>
                    </joint>
                </robot>
                """);
        Engine engine = new Engine();
        engine.loadUrdf(urdf.toString());
        Joint wrist = engine.getJoints().getFirst();
        SceneNode base = wrist.getParentNode();
        Quaternionf baseRotation = new Quaternionf(base.getLocalRotation());

        SceneNode object = new SceneNode("object");
        object.setRobotPart(false);
        object.getLocalPosition().set(0, 0, 4);
        engine.getRootNode().addChild(object);

        base.getLocalPosition().set(3, 2, 1);
        base.getLocalRotation().identity();
        wrist.getChildNode().getLocalPosition().zero();
        wrist.setAngle(0.9f);
        wrist.getMotor().setTargetAngleRadians(1.4f);
        wrist.getMotor().setCurrentVelocity(0.5f);

        assertTrue(engine.resetRobotPose());
        wrist.getMotor().update(0.016f);
        assertEquals(new Vector3f(), base.getLocalPosition());
        assertEquals(baseRotation, base.getLocalRotation());
        assertEquals(new Vector3f(0, 0, 1), wrist.getChildNode().getLocalPosition());
        assertEquals(0.3f, wrist.getCurrentAngleRadians());
        assertEquals(0.3f, wrist.getMotor().getTargetAngleRadians());
        assertEquals(0, wrist.getMotor().getCurrentVelocity());
        assertEquals(new Vector3f(0, 0, 4), object.getLocalPosition());
    }

    @Test void usesTheNewInitialPoseWhenAnotherProjectIsLoaded() throws Exception {
        Engine engine = new Engine();
        assertFalse(engine.resetRobotPose());
        String project = """
                {"links": [{"id": "base", "mesh": "primitive:box:1:1:1", "rotation": [0, %d, 0]}]}
                """;
        Path first = Files.writeString(tempDir.resolve("first.json"), project.formatted(30));
        Path second = Files.writeString(tempDir.resolve("second.json"), project.formatted(60));
        engine.loadProject(first.toString());
        engine.getRootNode().getLocalRotation().identity();
        engine.loadProject(second.toString());
        SceneNode secondBase = engine.getRootNode();
        Quaternionf loadedRotation = new Quaternionf(secondBase.getLocalRotation());
        secondBase.getLocalRotation().identity();

        assertTrue(engine.resetRobotPose());
        assertEquals(loadedRotation, secondBase.getLocalRotation());
    }
}
