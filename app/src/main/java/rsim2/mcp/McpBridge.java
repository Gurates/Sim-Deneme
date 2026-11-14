package rsim2.mcp;

import com.google.gson.*;
import java.io.*;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.nio.channels.FileChannel;
import java.nio.channels.FileLock;
import java.util.UUID;
import java.util.concurrent.*;
import java.util.function.Function;

/** Authenticated loopback IPC. Simulator objects are accessed only by drain() on its main thread. */
public final class McpBridge implements AutoCloseable {
    private record Request(JsonObject payload, CompletableFuture<JsonObject> reply, long deadline) {}
    private static final ThreadLocal<Long> requestDeadline = new ThreadLocal<>();

    public static void ensureRequestActive() {
        Long deadline = requestDeadline.get();
        if (deadline != null && System.nanoTime() >= deadline)
            throw new IllegalArgumentException("Request expired during planning; no new motion applied.");
    }
    private final BlockingQueue<Request> pending = new ArrayBlockingQueue<>(32);
    private ServerSocket listener;
    private String token;
    private volatile boolean enabled;
    private final Path sessionFile;
    private FileChannel lockChannel;
    private FileLock sessionLock;
    public static final int HTTP_PORT = 38172;
    private final Integer httpPort;
    private McpHttpEndpoint http;

    public McpBridge() { this(McpSession.defaultFile(), HTTP_PORT); }
    public McpBridge(Path sessionFile) { this(sessionFile, null); }
    McpBridge(Path sessionFile, Integer httpPort) {
        this.sessionFile = sessionFile.toAbsolutePath();
        this.httpPort = httpPort;
    }
    public Path sessionFile() { return sessionFile; }

    public synchronized void start() throws IOException {
        if (enabled) return;
        try {
            Files.createDirectories(sessionFile.getParent());
            lockChannel = FileChannel.open(sessionFile.resolveSibling(sessionFile.getFileName() + ".lock"),
                    StandardOpenOption.CREATE, StandardOpenOption.WRITE);
            sessionLock = lockChannel.tryLock();
            if (sessionLock == null) throw new IOException("MCP is already enabled in another RSim2 instance.");
            listener = new ServerSocket(0, 8, InetAddress.getByName("127.0.0.1"));
            token = UUID.randomUUID().toString();
            new McpSession(listener.getLocalPort(), token).write(sessionFile);
            if (httpPort != null) {
                ServerSocket session = listener;
                http = new McpHttpEndpoint(httpPort, sessionFile.resolveSibling("mcp-http-token"),
                        (tool, arguments) -> submit(tool, arguments, session));
            }
            enabled = true;
        } catch (Exception e) {
            close();
            throw new IOException("Could not publish MCP session: " + e.getMessage(), e);
        }
        ServerSocket session = listener;
        String secret = token;
        Thread worker = new Thread(() -> accept(session, secret), "MCP-Local-Bridge");
        worker.setDaemon(true);
        worker.start();
    }

    public boolean isEnabled() { return enabled; }
    public synchronized int port() { return listener == null ? 0 : listener.getLocalPort(); }
    public synchronized String token() { return token; }

    public synchronized McpHttpEndpoint.Connection httpConnection() {
        return http == null ? null : http.connection();
    }

    private void accept(ServerSocket session, String secret) {
        while (!session.isClosed()) {
            try (Socket socket = session.accept()) {
                socket.setSoTimeout(1500);
                JsonObject request = JsonParser.parseString(readLine(socket.getInputStream())).getAsJsonObject();
                JsonObject response;
                if (!secret.equals(request.has("token") ? request.get("token").getAsString() : "")) {
                    response = failure("Invalid bridge token. The RSim2 session changed; check connection status before retrying.");
                } else {
                    response = submit(request.get("tool").getAsString(), request.getAsJsonObject("arguments"), session);
                }
                socket.getOutputStream().write((response + "\n").getBytes(StandardCharsets.UTF_8));
            } catch (Exception e) {
                // Invalid/disconnected local clients must not terminate the bridge.
            }
        }
    }

    private JsonObject submit(String tool, JsonObject arguments, ServerSocket session) {
        JsonObject payload = new JsonObject();
        payload.addProperty("tool", tool);
        payload.add("arguments", arguments);
        Request queued = new Request(payload, new CompletableFuture<>(), System.nanoTime() + TimeUnit.SECONDS.toNanos(3));
        synchronized (this) {
            if (!enabled || listener != session || !pending.offer(queued)) return failure("Bridge disabled or queue is full.");
        }
        try { return queued.reply.get(3, TimeUnit.SECONDS); }
        catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return failure("Request interrupted. Check task status before retrying a move.");
        } catch (ExecutionException | TimeoutException e) {
            return failure("Simulation did not process the request in time. Check task status before retrying a move.");
        } finally {
            queued.reply.cancel(false);
            pending.remove(queued);
        }
    }

    public void drain(Function<JsonObject, JsonObject> handler) {
        for (int i = 0; i < 32; i++) {
            Request request = pending.poll();
            if (request == null) break;
            synchronized (this) {
                if (!enabled || request.reply.isCancelled()) continue;
                requestDeadline.set(request.deadline);
                try { ensureRequestActive(); request.reply.complete(handler.apply(request.payload)); }
                catch (RuntimeException e) { request.reply.complete(failure(e.getMessage())); }
                finally { requestDeadline.remove(); }
            }
        }
    }

    @Override public synchronized void close() {
        enabled = false;
        if (http != null) {
            http.close();
            http = null;
        }
        if (listener != null) try { listener.close(); } catch (IOException ignored) {}
        if (sessionLock != null && sessionLock.isValid()) {
            try { Files.deleteIfExists(sessionFile); } catch (IOException ignored) {}
            try { sessionLock.release(); } catch (IOException ignored) {}
        }
        if (lockChannel != null) try { lockChannel.close(); } catch (IOException ignored) {}
        sessionLock = null;
        lockChannel = null;
        Request request;
        while ((request = pending.poll()) != null) request.reply.complete(failure("MCP bridge disabled."));
    }

    static String readLine(InputStream input) throws IOException {
        ByteArrayOutputStream bytes = new ByteArrayOutputStream();
        for (int c; (c = input.read()) != -1 && c != '\n';) {
            if (bytes.size() >= 262144) throw new IOException("Bridge message too large");
            bytes.write(c);
        }
        return bytes.toString(StandardCharsets.UTF_8);
    }

    public static JsonObject call(int port, String token, String tool, JsonObject arguments) throws IOException {
        try (Socket socket = new Socket()) {
            socket.connect(new InetSocketAddress("127.0.0.1", port), 1000);
            socket.setSoTimeout(5000);
            JsonObject request = new JsonObject();
            request.addProperty("token", token);
            request.addProperty("tool", tool);
            request.add("arguments", arguments);
            socket.getOutputStream().write((request + "\n").getBytes(StandardCharsets.UTF_8));
            return JsonParser.parseString(readLine(socket.getInputStream())).getAsJsonObject();
        }
    }

    public static JsonObject failure(String message) {
        JsonObject result = new JsonObject();
        result.addProperty("error", message == null ? "Command failed" : message);
        return result;
    }
}
