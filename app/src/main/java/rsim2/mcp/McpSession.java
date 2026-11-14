package rsim2.mcp;

import com.google.gson.*;
import java.io.IOException;
import java.nio.file.*;

/** Stable discovery path; credentials remain local and rotate with each live session. */
public record McpSession(int port, String token) {
    public static Path defaultFile() {
        return Path.of(System.getProperty("user.home"), ".rsim2", "mcp-session.json").toAbsolutePath();
    }

    public static McpSession read(Path file) throws IOException {
        try {
            JsonObject json = JsonParser.parseString(Files.readString(file)).getAsJsonObject();
            int port = json.get("port").getAsInt();
            String token = json.get("token").getAsString();
            if (port < 1 || port > 65535 || token.isBlank()) throw new IOException("Invalid MCP session");
            return new McpSession(port, token);
        } catch (RuntimeException e) {
            throw new IOException("Invalid MCP session file", e);
        }
    }

    public void write(Path file) throws IOException {
        JsonObject json = new JsonObject();
        json.addProperty("port", port);
        json.addProperty("token", token);
        Path temporary = Files.createTempFile(file.getParent(), "mcp-session-", ".tmp");
        try {
            Files.writeString(temporary, json.toString());
            try { Files.move(temporary, file, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING); }
            catch (AtomicMoveNotSupportedException e) { Files.move(temporary, file, StandardCopyOption.REPLACE_EXISTING); }
        } finally { Files.deleteIfExists(temporary); }
    }
}
