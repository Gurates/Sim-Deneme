package rsim2.mcp;

import com.google.gson.*;
import java.io.*;
import java.net.URISyntaxException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.time.Duration;

/** Builds shell-free launch settings from the running application. */
public record McpConnectionSetup(String command, List<String> arguments) {
    public McpConnectionSetup {
        arguments = List.copyOf(arguments);
    }

    public static McpConnectionSetup detect() throws URISyntaxException {
        boolean windows = System.getProperty("os.name").toLowerCase().contains("win");
        Path java = Path.of(System.getProperty("java.home"), "bin", windows ? "java.exe" : "java");
        String classes = location(McpServer.class);
        String gson = location(Gson.class);
        return new McpConnectionSetup(java.toString(),
                List.of("-cp", classes + File.pathSeparator + gson, McpServer.class.getName()));
    }

    public static McpConnectionSetup detect(McpBridge bridge) throws URISyntaxException {
        McpConnectionSetup base = detect();
        var args = new java.util.ArrayList<>(base.arguments());
        args.addAll(List.of("--session-file", bridge.sessionFile().toString()));
        return new McpConnectionSetup(base.command(), args);
    }

    private static String location(Class<?> type) throws URISyntaxException {
        return Path.of(type.getProtectionDomain().getCodeSource().getLocation().toURI()).toAbsolutePath().toString();
    }

    public String jsonConfig() {
        JsonObject server = new JsonObject();
        server.addProperty("command", command);
        server.add("args", new Gson().toJsonTree(arguments));
        JsonObject servers = new JsonObject();
        servers.add("rsim2", server);
        JsonObject config = new JsonObject();
        config.add("mcpServers", servers);
        return new GsonBuilder().setPrettyPrinting().disableHtmlEscaping().create().toJson(config);
    }

    public String codexConfig() {
        // JSON strings/arrays are valid TOML basic strings/arrays for these paths.
        Gson gson = new GsonBuilder().disableHtmlEscaping().create();
        return "[mcp_servers.rsim2]\ncommand = " + gson.toJson(command)
                + "\nargs = " + gson.toJson(arguments) + "\n";
    }

    public static String httpCodexConfig(McpHttpEndpoint.Connection connection) {
        Gson gson = new GsonBuilder().disableHtmlEscaping().create();
        return "[mcp_servers.rsim2]\nurl = " + gson.toJson(connection.url())
                + "\nhttp_headers = { Authorization = " + gson.toJson("Bearer " + connection.token()) + " }\n";
    }

    public static String httpJsonConfig(McpHttpEndpoint.Connection connection) {
        JsonObject server = new JsonObject();
        server.addProperty("type", "http");
        server.addProperty("url", connection.url());
        JsonObject headers = new JsonObject();
        headers.addProperty("Authorization", "Bearer " + connection.token());
        server.add("headers", headers);
        JsonObject servers = new JsonObject();
        servers.add("rsim2", server);
        JsonObject config = new JsonObject();
        config.add("mcpServers", servers);
        return new GsonBuilder().setPrettyPrinting().create().toJson(config);
    }

    public static String testHttp(McpHttpEndpoint.Connection connection) throws IOException, InterruptedException {
        try (HttpClient client = HttpClient.newBuilder().connectTimeout(Duration.ofSeconds(2)).build()) {
            JsonObject init = httpCall(client, connection, """
                    {"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-11-25","capabilities":{},"clientInfo":{"name":"rsim2-test","version":"1"}}}
                    """);
            httpCall(client, connection, """
                    {"jsonrpc":"2.0","method":"notifications/initialized"}
                    """);
            JsonObject listed = httpCall(client, connection, """
                    {"jsonrpc":"2.0","id":2,"method":"tools/list"}
                    """);
            JsonArray tools = listed.getAsJsonArray("tools");
            boolean trajectory = tools.asList().stream().anyMatch(tool ->
                    "execute_trajectory".equals(tool.getAsJsonObject().get("name").getAsString()));
            if (!trajectory) throw new IOException("Server is missing execute_trajectory.");
            JsonObject result = httpCall(client, connection, """
                    {"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"get_connection_status"}}
                    """);
            JsonObject status = result.getAsJsonObject("structuredContent");
            if (!status.has("trajectory_control") || !status.get("trajectory_control").getAsBoolean())
                throw new IOException("Live simulation did not advertise trajectory control.");
            return "MCP " + init.getAsJsonObject("serverInfo").get("version").getAsString()
                    + " connected. " + tools.size() + " tools available, including execute_trajectory.";
        }
    }

    private static JsonObject httpCall(HttpClient client, McpHttpEndpoint.Connection connection, String json)
            throws IOException, InterruptedException {
        HttpResponse<String> reply = client.send(HttpRequest.newBuilder(URI.create(connection.url()))
                .timeout(Duration.ofSeconds(5))
                .header("Authorization", "Bearer " + connection.token())
                .header("Content-Type", "application/json")
                .header("Accept", "application/json, text/event-stream")
                .header("MCP-Protocol-Version", McpServer.PROTOCOL_VERSION)
                .POST(HttpRequest.BodyPublishers.ofString(json)).build(), HttpResponse.BodyHandlers.ofString());
        if (reply.statusCode() == 202) return new JsonObject();
        if (reply.statusCode() != 200) throw new IOException("MCP returned HTTP " + reply.statusCode());
        JsonObject object = JsonParser.parseString(reply.body()).getAsJsonObject();
        if (!object.has("result")) throw new IOException("MCP request did not return a result.");
        return object.getAsJsonObject("result");
    }

    public String testServer() throws IOException, InterruptedException {
        var launch = new java.util.ArrayList<String>();
        launch.add(command);
        launch.addAll(arguments);
        Process process = new ProcessBuilder(launch).redirectError(ProcessBuilder.Redirect.DISCARD).start();
        try {
            try (Writer input = new OutputStreamWriter(process.getOutputStream(), StandardCharsets.UTF_8)) {
                input.write("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"initialize\",\"params\":{\"protocolVersion\":\"2025-06-18\",\"capabilities\":{},\"clientInfo\":{\"name\":\"rsim2-setup\",\"version\":\"1\"}}}\n");
                input.write("{\"jsonrpc\":\"2.0\",\"method\":\"notifications/initialized\"}\n");
                input.write("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"tools/call\",\"params\":{\"name\":\"get_connection_status\"}}\n");
            }
            if (!process.waitFor(5, TimeUnit.SECONDS)) throw new IOException("Server did not respond within 5 seconds.");
            if (process.exitValue() != 0) throw new IOException("Server exited with code " + process.exitValue());
            try (BufferedReader output = process.inputReader(StandardCharsets.UTF_8)) {
                String first = output.readLine();
                String second = output.readLine();
                if (first == null || second == null) throw new IOException("Incomplete MCP response.");
                var init = JsonParser.parseString(first).getAsJsonObject();
                var reply = JsonParser.parseString(second).getAsJsonObject();
                if (!init.has("result") || !reply.has("result")) throw new IOException("MCP handshake failed.");
                var status = JsonParser.parseString(reply.getAsJsonObject("result").getAsJsonArray("content")
                        .get(0).getAsJsonObject().get("text").getAsString()).getAsJsonObject();
                if (status.has("error")) throw new IOException(status.get("error").getAsString());
                if (status.get("simulatorConnected").getAsBoolean()) return "Server test passed. Live simulator connected.";
                if (arguments.contains("--bridge-port") || arguments.contains("--session-file")) throw new IOException("Live simulator bridge unavailable. Open RSim2 and enable MCP.");
            }
            return "Server test passed. Simulator is not connected yet.";
        } finally {
            if (process.isAlive()) process.destroyForcibly();
        }
    }
}
