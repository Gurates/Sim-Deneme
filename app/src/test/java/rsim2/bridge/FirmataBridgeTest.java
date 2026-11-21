package rsim2.bridge;

import org.firmata4j.Parser;
import org.firmata4j.transport.TransportInterface;
import org.joml.Vector3f;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import rsim2.scene.Joint;
import rsim2.scene.JointType;
import rsim2.scene.SceneNode;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

import static org.junit.jupiter.api.Assertions.*;

class FirmataBridgeTest {
    private final BridgeManager manager = BridgeManager.getInstance();
    private final TestBoard board = new TestBoard();
    private final FirmataBridge bridge = new FirmataBridge((port, onDisconnect) -> {
        board.onDisconnect = onDisconnect;
        return board;
    });

    @BeforeEach
    void connect() throws InterruptedException {
        manager.setActiveBridge(bridge);
        manager.resetEmergencyStop();
        manager.getSafetyFilter().clearAngleHistory();
        assertTrue(bridge.connect("TEST", FirmataBridge.BAUD_RATE));
        await(bridge::isConnected);
    }

    @AfterEach
    void disconnect() {
        bridge.close();
        manager.setActiveBridge(null);
        manager.registerJoints(null);
        manager.resetEmergencyStop();
        manager.getSafetyFilter().resetCollisionStop();
        manager.getSafetyFilter().setEnforceServo0To180(false);
        manager.getSafetyFilter().clearAngleHistory();
    }

    @Test
    void sendsCalibratedServoCommandsOnlyAfterLiveSyncStarts() throws InterruptedException {
        Joint shoulder = joint("shoulder", 9);
        shoulder.setAngle((float) Math.toRadians(30));
        shoulder.setInverted(true);
        shoulder.setZeroOffsetDeg(10);
        Joint wrist = joint("wrist", 17);
        wrist.setZeroOffsetDeg(-10);
        List<Joint> joints = List.of(shoulder, wrist, joint("unassigned", -1));

        manager.streamCurrentJointStates(joints, 0);
        board.expectIdle();
        manager.getSafetyFilter().setEnforceServo0To180(true);
        manager.setLiveSyncEnabled(true);
        manager.streamCurrentJointStates(joints, 0);

        board.expectMessage(0xF4, 9, 4);
        board.expectMessage(0xE9, 70, 0);
        board.expectMessage(0xF4, 17, 4);
        board.expectMessage(0xF0, 0x6F, 17, 80, 0, 0, 0, 0xF7);

        shoulder.setAngle((float) Math.toRadians(31));
        await(() -> {
            manager.streamCurrentJointStates(joints, 0.02f);
            return !board.commands.isEmpty();
        });
        board.expectMessage(0xE9, 69, 0);

        manager.triggerEmergencyStop();
        manager.resetEmergencyStop();
        manager.streamCurrentJointStates(joints, 0);
        assertFalse(manager.isLiveSyncEnabled());
        board.expectIdle();
    }

    @Test
    void validatesBoardPinsAndDisarmsWhenCollisionPreventsStreaming() throws InterruptedException {
        Joint shoulder = joint("shoulder", 9);
        Joint wrist = joint("wrist", 9);
        List<Joint> joints = List.of(shoulder, wrist);
        manager.registerJoints(joints);
        assertTrue(assertThrows(IllegalArgumentException.class, () -> manager.setLiveSyncEnabled(true))
                .getMessage().contains("assigned more than once"));
        wrist.setPin(8);
        assertTrue(assertThrows(IllegalArgumentException.class, () -> manager.setLiveSyncEnabled(true))
                .getMessage().contains("servo-capable"));
        wrist.setPin(-1);
        shoulder.setAngle((float) Math.toRadians(120));
        assertTrue(assertThrows(IllegalArgumentException.class, () -> manager.setLiveSyncEnabled(true))
                .getMessage().contains("servo degrees"));
        shoulder.setAngle(0);
        manager.setLiveSyncEnabled(true);

        manager.getSafetyFilter().triggerCollisionStop();
        manager.streamCurrentJointStates(joints, 0);
        assertFalse(manager.isLiveSyncEnabled());
        assertTrue(bridge.getStats().getLastError().contains("Collision detected"));
        board.expectIdle();

        manager.getSafetyFilter().resetCollisionStop();
        manager.setLiveSyncEnabled(true);
        shoulder.setPin(17);
        manager.streamCurrentJointStates(joints, 0);
        assertFalse(manager.isLiveSyncEnabled());
        assertTrue(bridge.getStats().getLastError().contains("Hardware mapping changed"));
        board.expectIdle();

        manager.setLiveSyncEnabled(true);
        board.onDisconnect.run();
        await(() -> bridge.getConnectionState() == FirmataBridge.ConnectionState.DISCONNECTED);
        assertFalse(manager.isLiveSyncEnabled());
        assertTrue(board.closed);
    }

    private static Joint joint(String name, int pin) {
        Joint joint = new Joint(name, new SceneNode(name + "Base"), new SceneNode(name + "Link"),
                JointType.REVOLUTE, new Vector3f(0, 0, 1), -(float) Math.PI, (float) Math.PI, 2);
        joint.setPin(pin);
        return joint;
    }

    private static void await(BooleanSupplier ready) throws InterruptedException {
        long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(3);
        while (!ready.getAsBoolean()) {
            if (System.nanoTime() >= deadline) fail("Timed out waiting for Firmata.");
            Thread.sleep(10);
        }
    }

    private static class TestBoard implements TransportInterface {
        private final LinkedBlockingQueue<int[]> commands = new LinkedBlockingQueue<>();
        private Parser parser;
        private Runnable onDisconnect;
        private boolean closed;

        @Override
        public void start() {}

        @Override
        public void stop() {
            closed = true;
        }

        @Override
        public void setParser(Parser parser) {
            this.parser = parser;
        }

        @Override
        public void write(byte[] bytes) {
            int[] message = new int[bytes.length];
            for (int i = 0; i < bytes.length; i++) message[i] = Byte.toUnsignedInt(bytes[i]);
            if (message[0] == 0xF0) {
                switch (message[1]) {
                    case 0x79 -> reply(0xF0, 0x79, 2, 5, 'T', 0, 0xF7);
                    case 0x6B -> capabilities();
                    case 0x6D -> reply(0xF0, 0x6E, message[2], 0, 0, 0xF7);
                    case 0x69 -> analogMapping();
                    case 0x6F -> commands.add(message);
                    default -> { }
                }
            } else if (message[0] == 0xF4 || (message[0] & 0xF0) == 0xE0) {
                commands.add(message);
            }
        }

        private void capabilities() {
            int[] message = new int[25];
            message[0] = 0xF0;
            message[1] = 0x6C;
            int index = 2;
            for (int pin = 0; pin < 18; pin++) {
                if (pin == 9 || pin == 17) {
                    message[index++] = 4;
                    message[index++] = 14;
                }
                message[index++] = 0x7F;
            }
            message[index] = 0xF7;
            reply(message);
        }

        private void analogMapping() {
            int[] message = new int[21];
            Arrays.fill(message, 0x7F);
            message[0] = 0xF0;
            message[1] = 0x6A;
            message[20] = 0xF7;
            reply(message);
        }

        private void reply(int... message) {
            byte[] bytes = new byte[message.length];
            for (int i = 0; i < bytes.length; i++) bytes[i] = (byte) message[i];
            parser.parse(bytes);
        }

        private void expectMessage(int... message) throws InterruptedException {
            assertArrayEquals(message, commands.poll(2, TimeUnit.SECONDS));
        }

        private void expectIdle() throws InterruptedException {
            assertNull(commands.poll(75, TimeUnit.MILLISECONDS));
        }
    }
}
