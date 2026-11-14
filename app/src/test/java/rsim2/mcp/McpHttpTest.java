package rsim2.mcp;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.file.Path;
import java.time.Duration;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.*;

class McpHttpTest {
    @TempDir Path temporary;

    @Test void realHttpCatalogAndRobotCallUseSameEndpointAcrossRestart() throws Exception {
        Path file = temporary.resolve("session.json");
        try (var bridge = new McpBridge(file, 0); var client = HttpClient.newHttpClient()) {
            bridge.start();
            var connection = bridge.httpConnection();
            String token = connection.token();
            assertTrue(McpConnectionSetup.httpCodexConfig(connection).contains("http_headers"));
            var endpoint = URI.create(connection.url());
            assertEquals(401, client.send(request(endpoint, null, null, "{}"), HttpResponse.BodyHandlers.discarding()).statusCode());
            assertEquals(403, client.send(request(endpoint, token, "https://example.test", "{}"), HttpResponse.BodyHandlers.discarding()).statusCode());
            assertEquals(405, client.send(HttpRequest.newBuilder(endpoint).header("Authorization", "Bearer " + token)
                    .GET().build(), HttpResponse.BodyHandlers.discarding()).statusCode());
            var catalog = client.send(request(endpoint, token, null, """
                    {"jsonrpc":"2.0","id":1,"method":"tools/list"}
                    """), HttpResponse.BodyHandlers.ofString());
            assertEquals(200, catalog.statusCode());
            assertTrue(catalog.body().contains("\"execute_trajectory\""));
            assertTrue(catalog.body().contains("\"pick_object_top_down\""));
            var response = client.sendAsync(request(endpoint, token, null, """
                    {"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"get_robot_snapshot"}}
                    """), HttpResponse.BodyHandlers.ofString());
            long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(2);
            while (!response.isDone() && System.nanoTime() < deadline) {
                bridge.drain(call -> {
                    assertEquals("get_robot_snapshot", call.get("tool").getAsString());
                    JsonObject result = new JsonObject();
                    result.addProperty("received", true);
                    return result;
                });
                Thread.sleep(5);
            }
            var reply = JsonParser.parseString(response.get(1, TimeUnit.SECONDS).body()).getAsJsonObject();
            assertTrue(reply.getAsJsonObject("result").getAsJsonObject("structuredContent").get("received").getAsBoolean());
            bridge.close();
            try (var restarted = new McpBridge(file, endpoint.getPort())) {
                restarted.start();
                assertEquals(connection, restarted.httpConnection());
            }
        }
    }

    private static HttpRequest request(URI uri, String token, String origin, String body) {
        var builder = HttpRequest.newBuilder(uri).timeout(Duration.ofSeconds(5))
                .header("Content-Type", "application/json").header("Accept", "application/json, text/event-stream");
        if (token != null) builder.header("Authorization", "Bearer " + token);
        if (origin != null) builder.header("Origin", origin);
        return builder.POST(HttpRequest.BodyPublishers.ofString(body)).build();
    }
}
