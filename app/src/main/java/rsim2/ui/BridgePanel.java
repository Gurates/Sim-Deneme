package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiInputTextFlags;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import imgui.type.ImString;
import org.lwjgl.PointerBuffer;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.util.tinyfd.TinyFileDialogs;
import rsim2.bridge.BridgeManager;
import rsim2.bridge.BridgeStats;
import rsim2.bridge.RobotBridge;
import rsim2.bridge.UdpBridge;
import rsim2.motion.MotionPlayer;
import rsim2.motion.MotionSequence;
import rsim2.safety.SafetyFilter;
import rsim2.telemetry.TrajectoryExporter;

import java.util.List;

import static org.lwjgl.system.MemoryStack.stackPush;

public class BridgePanel {
    private boolean visible = false;
    private final ImBoolean isOpen = new ImBoolean(false);

    private final BridgeManager bridgeManager = BridgeManager.getInstance();
    private final MotionPlayer motionPlayer;
    private List<rsim2.scene.Joint> joints;

    private final ImString hostInput = new ImString("192.168.1.50", 128);
    private final ImInt portInput = new ImInt(8888);
    private final ImInt selectedProtocolIdx = new ImInt(0);
    private final ImInt publishRateHz = new ImInt(50);
    private final ImBoolean enforceServo0To180 = new ImBoolean(false);
    private final ImBoolean liveSyncToggle = new ImBoolean(false);
    private final ImBoolean digitalTwinToggle = new ImBoolean(false);

    private String statusMessage = "Bridge ready. Select protocol and click Connect.";

    public BridgePanel(MotionPlayer motionPlayer) {
        this(motionPlayer, null);
    }

    public BridgePanel(MotionPlayer motionPlayer, List<rsim2.scene.Joint> joints) {
        this.motionPlayer = motionPlayer;
        this.joints = joints;
    }

    public void setJoints(List<rsim2.scene.Joint> joints) {
        this.joints = joints;
    }

    public boolean isVisible() {
        return visible;
    }

    public void setVisible(boolean visible) {
        this.visible = visible;
        this.isOpen.set(visible);
    }

    public void toggleVisible() {
        setVisible(!visible);
    }

    public void render() {
        if (!visible) return;

        ImGui.setNextWindowSize(480.0f, 600.0f, ImGuiCond.FirstUseEver);
        int flags = ImGuiWindowFlags.NoCollapse;

        if (ImGui.begin("Sim-to-Real Hardware Bridge", isOpen, flags)) {
            if (!isOpen.get()) {
                visible = false;
            }

            renderConnectionSection();
            ImGui.separator();
            renderControlSection();
            ImGui.separator();
            renderSafetySection();
            ImGui.separator();
            renderTelemetrySection();
            ImGui.separator();
            renderExportSection();
        }
        ImGui.end();
    }

    private void renderConnectionSection() {
        ImGui.textColored(0.2f, 0.6f, 1.0f, 1.0f, "1. Target Hardware Protocol");

        List<RobotBridge> bridges = bridgeManager.getAvailableBridges();
        String[] protocolNames = new String[bridges.size()];
        for (int i = 0; i < bridges.size(); i++) {
            protocolNames[i] = bridges.get(i).getName();
        }

        if (ImGui.combo("Protocol", selectedProtocolIdx, protocolNames)) {
            bridgeManager.setActiveBridge(bridges.get(selectedProtocolIdx.get()));
        }

        RobotBridge active = bridgeManager.getActiveBridge();

        if (active instanceof UdpBridge) {
            ImGui.inputText("Target IP", hostInput, ImGuiInputTextFlags.None);
            ImGui.inputInt("Target Port", portInput);
        }

        boolean isConnected = active != null && active.isConnected();

        if (!isConnected) {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.6f, 0.3f, 1.0f);
            if (ImGui.button("Connect to Robot", 140.0f, 28.0f)) {
                boolean ok = active.connect(hostInput.get(), portInput.get());
                statusMessage = ok ? "Connected successfully!" : "Connection failed: " + active.getStats().getLastError();
            }
            ImGui.popStyleColor();
        } else {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.7f, 0.2f, 0.2f, 1.0f);
            if (ImGui.button("Disconnect", 140.0f, 28.0f)) {
                active.disconnect();
                statusMessage = "Disconnected from robot.";
            }
            ImGui.popStyleColor();
        }

        ImGui.sameLine();
        if (isConnected) {
            ImGui.textColored(0.2f, 0.9f, 0.3f, 1.0f, "[● CONNECTED]");
        } else {
            ImGui.textColored(0.7f, 0.7f, 0.7f, 1.0f, "[○ DISCONNECTED]");
        }

        ImGui.textDisabled(statusMessage);
    }

    private void renderControlSection() {
        ImGui.textColored(0.2f, 0.6f, 1.0f, 1.0f, "2. Live Synchronization");

        liveSyncToggle.set(bridgeManager.isLiveSyncEnabled());
        if (ImGui.checkbox("Stream Real-Time Motion to Hardware", liveSyncToggle)) {
            bridgeManager.setLiveSyncEnabled(liveSyncToggle.get());
        }

        digitalTwinToggle.set(bridgeManager.isDigitalTwinEnabled());
        if (ImGui.checkbox("Digital Twin (Mirror Real Robot Inbound Telemetry)", digitalTwinToggle)) {
            bridgeManager.setDigitalTwinEnabled(digitalTwinToggle.get());
        }
        if (ImGui.isItemHovered()) {
            ImGui.setTooltip("When active, robot joint angles received over UDP will update the 3D model in real time.");
        }

        publishRateHz.set(bridgeManager.getTargetPublishRateHz());
        if (ImGui.sliderInt("Stream Rate (Hz)", publishRateHz.getData(), 10, 200)) {
            bridgeManager.setTargetPublishRateHz(publishRateHz.get());
        }

        ImGui.spacing();

        SafetyFilter safety = bridgeManager.getSafetyFilter();
        boolean isEstop = safety.isEmergencyStopActive();

        if (!isEstop) {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.85f, 0.15f, 0.15f, 1.0f);
            ImGui.pushStyleColor(ImGuiCol.ButtonHovered, 1.0f, 0.25f, 0.25f, 1.0f);
            ImGui.pushStyleColor(ImGuiCol.ButtonActive, 0.7f, 0.1f, 0.1f, 1.0f);
            if (ImGui.button("EMERGENCY STOP (E-STOP)", -1.0f, 36.0f)) {
                bridgeManager.triggerEmergencyStop();
                statusMessage = "EMERGENCY STOP ACTIVATED!";
            }
            ImGui.popStyleColor(3);
        } else {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.9f, 0.5f, 0.1f, 1.0f);
            if (ImGui.button("RESET EMERGENCY STOP", -1.0f, 36.0f)) {
                bridgeManager.resetEmergencyStop();
                statusMessage = "E-Stop reset. Ready for control.";
            }
            ImGui.popStyleColor();
        }
    }

    private void renderSafetySection() {
        ImGui.textColored(0.2f, 0.6f, 1.0f, 1.0f, "3. Safety & Normalization Envelope");

        SafetyFilter safety = bridgeManager.getSafetyFilter();

        ImFloat maxDegPerSec = new ImFloat(safety.getMaxDegreesPerSecond());
        if (ImGui.sliderFloat("Max Velocity (Deg/s)", maxDegPerSec.getData(), 10.0f, 720.0f)) {
            safety.setMaxDegreesPerSecond(maxDegPerSec.get());
        }
        if (ImGui.isItemHovered()) {
            ImGui.setTooltip("Maximum angular velocity in degrees per second.\nThis is time-based and independent of publish rate.");
        }

        enforceServo0To180.set(safety.isEnforceServo0To180());
        if (ImGui.checkbox("Map to 0..180 deg (Standard Servos)", enforceServo0To180)) {
            safety.setEnforceServo0To180(enforceServo0To180.get());
        }

        ImGui.spacing();
        if (safety.isCollisionStopActive()) {
            ImGui.textColored(1.0f, 0.25f, 0.25f, 1.0f, "[⚠️ COLLISION STOP ACTIVE — Hardware Stream Blocked]");
        } else {
            ImGui.textColored(0.2f, 0.85f, 0.35f, 1.0f, "[✓ Collision Clear]");
        }
    }

    private void renderTelemetrySection() {
        ImGui.textColored(0.2f, 0.6f, 1.0f, 1.0f, "4. Real-Time Telemetry & Diagnostics");

        RobotBridge active = bridgeManager.getActiveBridge();
        BridgeStats stats = active != null ? active.getStats() : new BridgeStats();

        ImGui.columns(2, "telemetryCols", false);
        ImGui.text("Packets Sent:");
        ImGui.nextColumn();
        ImGui.text(String.valueOf(stats.getPacketsSent()));
        ImGui.nextColumn();

        ImGui.text("Payload Bytes:");
        ImGui.nextColumn();
        ImGui.text((stats.getBytesSent() / 1024) + " KB");
        ImGui.nextColumn();

        ImGui.text("Latency / Ping:");
        ImGui.nextColumn();
        ImGui.text(String.format("%.2f ms", stats.getRoundTripLatencyMs()));
        ImGui.nextColumn();

        ImGui.text("Error Count:");
        ImGui.nextColumn();
        if (stats.getErrorCount() > 0) {
            ImGui.textColored(1.0f, 0.3f, 0.3f, 1.0f, String.valueOf(stats.getErrorCount()));
        } else {
            ImGui.text("0");
        }
        ImGui.columns(1);
    }

    private void renderExportSection() {
        ImGui.textColored(0.2f, 0.6f, 1.0f, 1.0f, "5. Embedded Code & Dataset Export");

        MotionSequence seq = (motionPlayer != null) ? motionPlayer.getSequence() : null;
        if (seq == null || seq.getKeyframes().isEmpty()) {
            ImGui.textDisabled("No active motion sequence loaded in MotionPlayer.");
            return;
        }

        ImGui.text("Active Motion: " + seq.getName() + " (" + seq.getDurationSeconds() + "s)");

        if (ImGui.button("Export Full Arduino Sketch (.ino)", 230.0f, 26.0f)) {
            exportTrajectory("ino", "Arduino Sketch (*.ino)", TrajectoryExporter.toArduinoSketch(seq, joints, publishRateHz.get()));
        }

        ImGui.sameLine();
        if (ImGui.button("Export to Header (.h)", 160.0f, 26.0f)) {
            exportTrajectory("h", "Arduino C++ Header (*.h)", TrajectoryExporter.toArduinoHeader(seq, joints, publishRateHz.get()));
        }

        ImGui.sameLine();
        if (ImGui.button("Export to CSV (.csv)", 150.0f, 26.0f)) {
            exportTrajectory("csv", "CSV Trajectory (*.csv)", TrajectoryExporter.toCsv(seq, publishRateHz.get()));
        }

        ImGui.spacing();
        if (ImGui.button("Export Raspberry Pi Python (.py)", 230.0f, 26.0f)) {
            exportTrajectory("py", "Raspberry Pi Python Script (*.py)", TrajectoryExporter.toPythonScript(seq, joints, publishRateHz.get()));
        }

        ImGui.sameLine();
        if (ImGui.button("Export RPi Live Receiver Agent (.py)", 260.0f, 26.0f)) {
            exportTrajectory("py", "Raspberry Pi Receiver Script (*.py)", TrajectoryExporter.toRpiLiveReceiverScript());
        }
    }

    private void exportTrajectory(String ext, String filterDesc, String content) {
        try (MemoryStack stack = stackPush()) {
            PointerBuffer filters = stack.mallocPointer(1);
            filters.put(stack.UTF8("*." + ext));
            filters.flip();

            String defaultName = "robot_trajectory." + ext;
            String path = TinyFileDialogs.tinyfd_saveFileDialog("Export Trajectory", defaultName, filters, filterDesc);
            if (path != null && !path.trim().isEmpty()) {
                boolean ok = TrajectoryExporter.saveToFile(content, path);
                statusMessage = ok ? "Saved: " + path : "Failed to save file!";
            }
        }
    }
}
