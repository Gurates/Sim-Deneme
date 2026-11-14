package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import imgui.type.ImString;
import rsim2.mcp.McpConnectionSetup;
import rsim2.mcp.McpBridge;
import rsim2.mcp.McpHttpEndpoint;

public final class McpPanel {
    private final McpBridge bridge;
    private final ImBoolean open = new ImBoolean(false);
    private final ImInt format = new ImInt(0);
    private final ImString preview = new ImString(32768);
    private McpConnectionSetup setup;
    private McpHttpEndpoint.Connection http;
    private volatile boolean testing;
    private volatile String status = "Server has not been tested.";
    private String setupError;

    public McpPanel(McpBridge bridge) { this.bridge = bridge; }

    public void toggleVisible() {
        open.set(!open.get());
        if (open.get()) refresh();
    }

    private void refresh() {
        try {
            http = bridge.httpConnection();
            setup = McpConnectionSetup.detect(bridge);
            setupError = null;
            updatePreview();
        } catch (Exception e) {
            setup = null;
            setupError = "Could not locate MCP runtime: " + e.getMessage();
        }
    }

    private void updatePreview() {
        preview.set(switch (format.get()) {
            case 0 -> http == null ? "" : McpConnectionSetup.httpCodexConfig(http);
            case 1 -> http == null ? "" : McpConnectionSetup.httpJsonConfig(http);
            case 2 -> setup == null ? "" : setup.codexConfig();
            default -> setup == null ? "" : setup.jsonConfig();
        });
    }

    public void render() {
        if (!open.get()) return;
        ImGui.setNextWindowSize(690, 630, ImGuiCond.FirstUseEver);
        if (ImGui.begin("MCP Connection", open)) {
            ImGui.textWrapped("Connect a local MCP client to RSim2. HTTP connects directly to the running app.");
            ImGui.spacing();
            ImGui.textColored(bridge.isEnabled() ? 0.1f : 0.8f, 0.55f, 0.2f, 1,
                    bridge.isEnabled() ? "Live simulation bridge enabled" : "MCP disabled");
            ImGui.beginDisabled(testing);
            if (ImGui.button(bridge.isEnabled() ? "Disable MCP" : "Enable MCP")) {
                try {
                    if (bridge.isEnabled()) bridge.close(); else bridge.start();
                    status = bridge.isEnabled() ? "Enabled. Connect the client to the local MCP address." : "Disabled. MCP motion will stop.";
                    refresh();
                } catch (Exception e) { setupError = "Could not enable MCP: " + e.getMessage(); }
            }
            ImGui.endDisabled();
            if (http != null) ImGui.textUnformatted(http.url());
            ImGui.textWrapped("Clients can inspect the robot, run joint or Cartesian trajectories and control playback. Movement requires simulation mode.");
            ImGui.separator();
            if (setupError != null) ImGui.textWrapped(setupError);
            if (ImGui.combo("Configuration format", format, new String[]{"Codex HTTP (TOML)", "Claude Code HTTP (JSON)",
                    "Codex stdio (TOML)", "Other stdio clients (JSON)"})) updatePreview();
            ImGui.textWrapped("Copy the rsim2 entry into your client's MCP settings. Replace an older rsim2 entry and reconnect the client.");
            ImGui.beginDisabled(preview.get().isEmpty() || !bridge.isEnabled());
            if (ImGui.button("Copy Configuration")) ImGui.setClipboardText(preview.get());
            ImGui.sameLine();
            ImGui.beginDisabled(testing);
            if (ImGui.button(testing ? "Testing..." : "Test Server")) {
                testing = true;
                status = "Checking connection and tool catalog...";
                McpConnectionSetup requested = setup;
                McpHttpEndpoint.Connection connection = http;
                boolean useHttp = format.get() < 2;
                Thread worker = new Thread(() -> {
                    try { status = useHttp ? McpConnectionSetup.testHttp(connection) : requested.testServer(); }
                    catch (InterruptedException e) { Thread.currentThread().interrupt(); status = "Test interrupted."; }
                    catch (Exception e) { status = "Test failed: " + e.getMessage(); }
                    finally { testing = false; }
                }, "MCP-Setup-Test");
                worker.setDaemon(true);
                worker.start();
            }
            ImGui.endDisabled();
            ImGui.endDisabled();
            ImGui.textWrapped(status);
            ImGui.separator();
            ImGui.textWrapped("Connect once. The HTTP address and access token stay the same across restarts. The copied HTTP configuration contains the access token.");
        }
        ImGui.end();
    }
}
