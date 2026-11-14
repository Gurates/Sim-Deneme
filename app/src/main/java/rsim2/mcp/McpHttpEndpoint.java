package rsim2.mcp;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.security.MessageDigest;
import java.security.SecureRandom;
import java.util.HexFormat;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/** Local Streamable HTTP transport. JSON replies are sufficient; no SSE stream is opened. */
public final class McpHttpEndpoint implements AutoCloseable {
    public record Connection(String url, String token) {}
    private static final int MAX_REQUEST_BYTES = 262144;
    private final HttpServer http;
    private final ExecutorService workers;
    private final Connection connection;

    McpHttpEndpoint(int port, Path tokenFile, McpServer.ToolCaller caller) throws IOException {
        String token = persistentToken(tokenFile);
        http = HttpServer.create(new InetSocketAddress("127.0.0.1", port), 16);
        connection = new Connection("http://127.0.0.1:" + http.getAddress().getPort() + "/mcp", token);
        workers = Executors.newFixedThreadPool(4, work -> {
            Thread thread = new Thread(work, "RSim2-MCP-HTTP");
            thread.setDaemon(true);
            return thread;
        });
        http.setExecutor(workers);
        http.createContext("/mcp", exchange -> serve(exchange, caller));
        http.start();
    }

    public Connection connection() { return connection; }

    private void serve(HttpExchange exchange, McpServer.ToolCaller caller) throws IOException {
        try (exchange) {
            if (!"/mcp".equals(exchange.getRequestURI().getRawPath())) {
                exchange.sendResponseHeaders(404, -1);
                return;
            }
            int port = http.getAddress().getPort();
            String host = exchange.getRequestHeaders().getFirst("Host");
            String origin = exchange.getRequestHeaders().getFirst("Origin");
            if (!("127.0.0.1:" + port).equals(host) && !("localhost:" + port).equalsIgnoreCase(host)
                    || origin != null && !origin.equals("http://127.0.0.1:" + port)
                                      && !origin.equals("http://localhost:" + port)) {
                exchange.sendResponseHeaders(403, -1);
                return;
            }
            String authorization = exchange.getRequestHeaders().getFirst("Authorization");
            if (authorization == null || !MessageDigest.isEqual(
                    authorization.getBytes(StandardCharsets.UTF_8),
                    ("Bearer " + connection.token()).getBytes(StandardCharsets.UTF_8))) {
                exchange.getResponseHeaders().set("WWW-Authenticate", "Bearer");
                exchange.sendResponseHeaders(401, -1);
                return;
            }
            if (!"POST".equals(exchange.getRequestMethod())) {
                exchange.getResponseHeaders().set("Allow", "POST");
                exchange.sendResponseHeaders(405, -1);
                return;
            }
            String contentType = exchange.getRequestHeaders().getFirst("Content-Type");
            if (contentType == null || !contentType.split(";", 2)[0].trim().toLowerCase(Locale.ROOT).equals("application/json")) {
                exchange.sendResponseHeaders(415, -1);
                return;
            }
            String version = exchange.getRequestHeaders().getFirst("MCP-Protocol-Version");
            if (version != null && !McpServer.supportsVersion(version)) {
                exchange.sendResponseHeaders(400, -1);
                return;
            }
            byte[] request = exchange.getRequestBody().readNBytes(MAX_REQUEST_BYTES + 1);
            if (request.length > MAX_REQUEST_BYTES) {
                exchange.sendResponseHeaders(413, -1);
                return;
            }
            var reply = new McpServer(caller).handle(new String(request, StandardCharsets.UTF_8));
            if (reply == null) {
                exchange.sendResponseHeaders(202, -1);
                return;
            }
            byte[] body = reply.toString().getBytes(StandardCharsets.UTF_8);
            exchange.getResponseHeaders().set("Content-Type", "application/json; charset=utf-8");
            exchange.getResponseHeaders().set("Cache-Control", "no-store");
            exchange.sendResponseHeaders(200, body.length);
            exchange.getResponseBody().write(body);
        }
    }

    private static String persistentToken(Path file) throws IOException {
        if (Files.exists(file)) {
            String token = Files.readString(file).trim();
            if (!token.matches("[a-f0-9]{64}")) throw new IOException("Invalid local MCP access token file.");
            return token;
        }
        byte[] bytes = new byte[32];
        new SecureRandom().nextBytes(bytes);
        String token = HexFormat.of().formatHex(bytes);
        // createTempFile uses restricted permissions on POSIX; Windows inherits the user's directory ACL.
        Path temporary = Files.createTempFile(file.getParent(), "mcp-http-", ".tmp");
        try {
            Files.writeString(temporary, token, StandardOpenOption.TRUNCATE_EXISTING);
            Files.move(temporary, file);
        } finally {
            Files.deleteIfExists(temporary);
        }
        return token;
    }

    @Override public void close() {
        http.stop(0);
        workers.shutdownNow();
    }
}
