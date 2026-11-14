package rsim2.mcp;

import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;
import java.io.StringReader;
import java.io.StringWriter;
import static org.junit.jupiter.api.Assertions.*;

class McpServerTest {
    private static final String INITIALIZE = "{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"initialize\",\"params\":{\"protocolVersion\":\"2025-06-18\",\"capabilities\":{},\"clientInfo\":{\"name\":\"test\",\"version\":\"1\"}}}";
    private static final String READY = "{\"jsonrpc\":\"2.0\",\"method\":\"notifications/initialized\"}";

    @Test void fullSessionUsesOnlyJsonLinesAndReportsDisconnected() throws Exception {
        StringWriter output = new StringWriter();
        new McpServer().serve(new StringReader(INITIALIZE + "\n" + READY + "\n"
                + "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"tools/list\"}\n"
                + "{\"jsonrpc\":\"2.0\",\"id\":\"status\",\"method\":\"tools/call\",\"params\":{\"name\":\"get_connection_status\"}}\n"), output);
        String[] lines = output.toString().lines().toArray(String[]::new);
        assertEquals(3, lines.length); // Notifications must not produce replies.
        var init = JsonParser.parseString(lines[0]).getAsJsonObject().getAsJsonObject("result");
        assertEquals("2025-06-18", init.get("protocolVersion").getAsString());
        var listed = JsonParser.parseString(lines[1]).getAsJsonObject().getAsJsonObject("result").getAsJsonArray("tools");
        assertEquals("get_connection_status", listed.get(0).getAsJsonObject().get("name").getAsString());
        var reply = JsonParser.parseString(lines[2]).getAsJsonObject();
        assertEquals("status", reply.get("id").getAsString());
        var status = JsonParser.parseString(reply.getAsJsonObject("result").getAsJsonArray("content")
                .get(0).getAsJsonObject().get("text").getAsString()).getAsJsonObject();
        assertFalse(status.get("simulatorConnected").getAsBoolean());
        assertFalse(status.get("robotControlAvailable").getAsBoolean());
    }

    @Test void rejectsInvalidMessagesAndRecovers() {
        McpServer server = new McpServer();
        assertEquals(-32700, server.handle("{").getAsJsonObject("error").get("code").getAsInt());
        assertEquals(-32600, server.handle("[]").getAsJsonObject("error").get("code").getAsInt());
        assertEquals(-32600, server.handle("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"tools/list\"}")
                .getAsJsonObject("error").get("code").getAsInt());
        assertNotNull(server.handle(INITIALIZE).get("result"));
        assertNull(server.handle(READY));
        assertEquals(-32602, server.handle("{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"tools/call\",\"params\":{\"name\":\"unknown_tool\"}}")
                .getAsJsonObject("error").get("code").getAsInt());
        assertEquals(-32602, server.handle("{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"tools/call\",\"params\":{\"name\":\"get_connection_status\",\"arguments\":{\"unexpected\":true}}}")
                .getAsJsonObject("error").get("code").getAsInt());
    }
}
