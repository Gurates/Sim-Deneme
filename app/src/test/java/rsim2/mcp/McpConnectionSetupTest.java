package rsim2.mcp;

import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

class McpConnectionSetupTest {
    @TempDir Path temporary;
    @Test void jsonPreservesSpacesUnicodeAndWindowsEscapes() {
        var setup = new McpConnectionSetup("C:\\Program Files\\Java\\java.exe",
                List.of("-cp", "C:\\Masaüstü\\RSim 2\\app.jar;C:\\libs\\gson.jar", "rsim2.mcp.McpServer"));
        var server = JsonParser.parseString(setup.jsonConfig()).getAsJsonObject()
                .getAsJsonObject("mcpServers").getAsJsonObject("rsim2");
        assertEquals(setup.command(), server.get("command").getAsString());
        assertEquals(setup.arguments().get(1), server.getAsJsonArray("args").get(1).getAsString());
        assertTrue(setup.codexConfig().startsWith("[mcp_servers.rsim2]\n"));
        assertTrue(setup.codexConfig().contains("C:\\\\Program Files\\\\Java\\\\java.exe"));
    }

    @Test void detectedRuntimeCompletesRealStdioHandshake() throws Exception {
        var setup = McpConnectionSetup.detect();
        var args = new java.util.ArrayList<>(setup.arguments());
        args.add(0, "-Duser.home=" + temporary);
        setup = new McpConnectionSetup(setup.command(), args);
        assertTrue(Path.of(setup.command()).isAbsolute());
        assertTrue(Files.isRegularFile(Path.of(setup.command())));
        assertEquals("Server test passed. Simulator is not connected yet.", setup.testServer());
    }
}
