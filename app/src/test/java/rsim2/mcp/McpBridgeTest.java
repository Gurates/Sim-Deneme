package rsim2.mcp;

import com.google.gson.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.Path;
import java.util.concurrent.*;
import static org.junit.jupiter.api.Assertions.*;

class McpBridgeTest {
    @TempDir Path temporary;

    @Test void sameServerAndConfigReconnectAfterBridgeRestart() throws Exception {
        Path file = temporary.resolve("session.json");
        try (McpBridge bridge = new McpBridge(file)) {
            var server = new McpServer(file);
            server.handle("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"initialize\",\"params\":{\"protocolVersion\":\"2025-06-18\",\"capabilities\":{},\"clientInfo\":{}}}");
            server.handle("{\"jsonrpc\":\"2.0\",\"method\":\"notifications/initialized\"}");
            String request = "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"tools/call\",\"params\":{\"name\":\"get_connection_status\"}}";
            String config = McpConnectionSetup.detect(bridge).codexConfig();
            assertFalse(config.contains("--bridge-token"));
            assertFalse(server.handle(request).getAsJsonObject("result").getAsJsonObject("structuredContent").get("simulatorConnected").getAsBoolean());
            String oldToken = null;
            for (int iteration = 0; iteration < 2; iteration++) {
                bridge.start();
                assertEquals(config, McpConnectionSetup.detect(bridge).codexConfig());
                assertNotEquals(oldToken, bridge.token());
                oldToken = bridge.token();
                assertEquals(bridge.port(), McpSession.read(file).port());
                var client = CompletableFuture.supplyAsync(() -> server.handle(request));
                long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(2);
                while (!client.isDone() && System.nanoTime() < deadline) {
                    bridge.drain(payload -> JsonParser.parseString("{\"simulatorConnected\":true}").getAsJsonObject());
                    Thread.sleep(5);
                }
                assertTrue(client.get(1, TimeUnit.SECONDS).getAsJsonObject("result").getAsJsonObject("structuredContent").get("simulatorConnected").getAsBoolean());
                bridge.close();
                assertFalse(java.nio.file.Files.exists(file));
                assertFalse(server.handle(request).getAsJsonObject("result").getAsJsonObject("structuredContent").get("simulatorConnected").getAsBoolean());
            }
        }
    }

    @Test void secondInstanceCannotReplaceActiveSession() throws Exception {
        Path file = temporary.resolve("session.json");
        try (McpBridge first = new McpBridge(file); McpBridge second = new McpBridge(file)) {
            first.start();
            var session = McpSession.read(file);
            assertThrows(java.io.IOException.class, second::start);
            assertEquals(session, McpSession.read(file));
            assertTrue(first.isEnabled());
        }
    }
    @Test void stdioMoveReachesSimulationAndReturnsTaskWithRealJointChange() throws Exception {
        var root = new rsim2.scene.SceneNode("world");
        var arm = new rsim2.scene.SceneNode("arm");
        root.addChild(arm);
        var joint = new rsim2.scene.Joint("shoulder", root, arm, new org.joml.Vector3f(0, 1, 0));
        var joints = java.util.List.of(joint);
        var player = new rsim2.motion.MotionPlayer();
        var robot = new McpRobotController();
        try (McpBridge bridge = new McpBridge(temporary.resolve("session.json"))) {
            bridge.start();
            var server = new McpServer(bridge.port(), bridge.token());
            server.handle("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"initialize\",\"params\":{\"protocolVersion\":\"2025-06-18\",\"capabilities\":{},\"clientInfo\":{}}}");
            server.handle("{\"jsonrpc\":\"2.0\",\"method\":\"notifications/initialized\"}");
            var client = CompletableFuture.supplyAsync(() -> server.handle("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"tools/call\",\"params\":{\"name\":\"move_joints\",\"arguments\":{\"targets_degrees\":{\"shoulder\":15}}}}"));
            long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(2);
            while (!client.isDone() && System.nanoTime() < deadline) {
                bridge.drain(request -> robot.execute(request.get("tool").getAsString(), request.getAsJsonObject("arguments"),
                        root, joints, player, false, () -> {}));
                Thread.sleep(5);
            }
            var result = client.get(1, TimeUnit.SECONDS).getAsJsonObject("result");
            assertFalse(result.get("isError").getAsBoolean());
            assertEquals("running", result.getAsJsonObject("structuredContent").get("state").getAsString());
            player.update(5, joints);
            robot.afterStep(root, joints, player, null);
            assertEquals(15, Math.toDegrees(joint.getCurrentAngleRadians()), 0.001);
            assertEquals("completed", robot.execute("get_task_status", new JsonObject(), root, joints, player, false, () -> {}).get("state").getAsString());
        }
    }

    @Test void authenticatedRequestsExecuteOnlyWhenSimulationDrainsQueue() throws Exception {
        try (McpBridge bridge = new McpBridge(temporary.resolve("session.json"))) {
            bridge.start();
            var denied = McpBridge.call(bridge.port(), "wrong-token", "get_robot_state", new JsonObject());
            assertTrue(denied.has("error"));
            var client = CompletableFuture.supplyAsync(() -> {
                try { return McpBridge.call(bridge.port(), bridge.token(), "get_robot_state", new JsonObject()); }
                catch (Exception e) { throw new CompletionException(e); }
            });
            long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(2);
            while (!client.isDone() && System.nanoTime() < deadline) {
                bridge.drain(request -> {
                    assertEquals("get_robot_state", request.get("tool").getAsString());
                    JsonObject response = new JsonObject();
                    response.addProperty("simulatorConnected", true);
                    return response;
                });
                Thread.sleep(5);
            }
            assertTrue(client.get(1, TimeUnit.SECONDS).get("simulatorConnected").getAsBoolean());
            var oldToken = bridge.token();
            bridge.close();
            bridge.start();
            assertNotEquals(oldToken, bridge.token());
        }
    }

    @Test void timedOutRequestNeverExecutesLater() throws Exception {
        try (McpBridge bridge = new McpBridge(temporary.resolve("session.json"))) {
            bridge.start();
            JsonObject response = McpBridge.call(bridge.port(), bridge.token(), "move_joints", new JsonObject());
            assertTrue(response.has("error"));
            bridge.drain(request -> { fail("Expired command executed"); return new JsonObject(); });
        }
    }

    @Test void generatedLaunchConnectsRealStdioServerToLiveBridge() throws Exception {
        try (McpBridge bridge = new McpBridge(temporary.resolve("session.json"))) {
            bridge.start();
            var setup = McpConnectionSetup.detect(bridge);
            var client = CompletableFuture.supplyAsync(() -> {
                try { return setup.testServer(); }
                catch (Exception e) { throw new CompletionException(e); }
            });
            long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(4);
            while (!client.isDone() && System.nanoTime() < deadline) {
                bridge.drain(request -> JsonParser.parseString("{\"simulatorConnected\":true,\"robotControlAvailable\":true}").getAsJsonObject());
                Thread.sleep(5);
            }
            assertEquals("Server test passed. Live simulator connected.", client.get(2, TimeUnit.SECONDS));
        }
    }
}
