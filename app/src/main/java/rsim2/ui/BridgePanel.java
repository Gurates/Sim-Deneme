package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImBoolean;
import org.lwjgl.PointerBuffer;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.util.tinyfd.TinyFileDialogs;
import rsim2.bridge.BridgeManager;
import rsim2.bridge.FirmataBridge;
import rsim2.motion.MotionPlayer;
import rsim2.motion.MotionSequence;
import rsim2.scene.Joint;
import rsim2.telemetry.TrajectoryExporter;

import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;

import static org.lwjgl.system.MemoryStack.stackPush;

public class BridgePanel {
    private static final int EXPORT_RATE_HZ = 50;

    private boolean visible = false;
    private final ImBoolean isOpen = new ImBoolean(false);

    private final MotionPlayer motionPlayer;
    private List<Joint> joints;
    private String statusMessage = "";
    private List<String> serialPorts;
    private String selectedPort = "";
    private String firmataMessage = "";

    public BridgePanel(MotionPlayer motionPlayer) {
        this(motionPlayer, null);
    }

    public BridgePanel(MotionPlayer motionPlayer, List<Joint> joints) {
        this.motionPlayer = motionPlayer;
        this.joints = joints;
    }

    public void setJoints(List<Joint> joints) {
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

        ImGui.setNextWindowSize(410.0f, 365.0f, ImGuiCond.FirstUseEver);
        ImGui.setNextWindowSizeConstraints(325.0f, 270.0f, Float.MAX_VALUE, Float.MAX_VALUE);
        if (ImGui.begin("Sim2Real", isOpen, ImGuiWindowFlags.NoCollapse)) {
            renderFirmata();
            ImGui.spacing();
            ImGui.separator();
            ImGui.spacing();
            renderExportButtons();
        }
        ImGui.end();
        visible = isOpen.get();
    }

    private void renderFirmata() {
        BridgeManager manager = BridgeManager.getInstance();
        FirmataBridge firmata = manager.getFirmataBridge();
        FirmataBridge.ConnectionState state = firmata.getConnectionState();
        boolean disconnected = state == FirmataBridge.ConnectionState.DISCONNECTED;
        boolean connected = state == FirmataBridge.ConnectionState.CONNECTED;
        boolean streaming = connected && manager.isLiveSyncEnabled();
        boolean stopped = manager.getSafetyFilter().isEmergencyStopActive();

        ImGui.text("Firmata live sync");
        if (serialPorts == null) refreshSerialPorts();
        ImGui.beginDisabled(!disconnected);
        ImGui.setNextItemWidth(130.0f);
        String preview = serialPorts.isEmpty() ? "No ports found" : "Select port";
        if (ImGui.beginCombo("##firmataPort", selectedPort.isEmpty() ? preview : selectedPort)) {
            for (String port : serialPorts) {
                if (ImGui.selectable(port, port.equals(selectedPort))) selectedPort = port;
            }
            ImGui.endCombo();
        }
        ImGui.sameLine();
        if (ImGui.button("Refresh Ports")) refreshSerialPorts();
        ImGui.endDisabled();

        ImGui.beginDisabled(state == FirmataBridge.ConnectionState.DISCONNECTING || (disconnected && selectedPort.isEmpty()));
        if (ImGui.button(disconnected ? "Connect" : "Disconnect")) {
            firmataMessage = "";
            if (disconnected) manager.connectFirmata(selectedPort);
            else firmata.disconnect();
        }
        ImGui.endDisabled();
        ImGui.sameLine();
        ImGui.text(switch (state) {
            case DISCONNECTED -> "Disconnected";
            case CONNECTING -> "Connecting to StandardFirmata...";
            case CONNECTED -> streaming ? "Streaming" : "Connected";
            case DISCONNECTING -> "Disconnecting...";
        });

        ImGui.beginDisabled(!connected || stopped);
        if (ImGui.button(streaming ? "Stop Live Sync" : "Start Live Sync")) {
            firmataMessage = "";
            try {
                manager.setLiveSyncEnabled(!streaming);
            } catch (IllegalArgumentException error) {
                firmataMessage = error.getMessage();
            }
        }
        ImGui.endDisabled();
        ImGui.sameLine();
        ImGui.beginDisabled(!connected && !stopped);
        if (ImGui.button(stopped ? "Reset E-stop" : "E-stop")) {
            firmataMessage = "";
            if (stopped) manager.resetEmergencyStop();
            else manager.triggerEmergencyStop();
        }
        ImGui.endDisabled();

        String error = firmataMessage.isEmpty() ? firmata.getStats().getLastError() : firmataMessage;
        if (!error.isEmpty()) ImGui.textWrapped(error);
        if (stopped) ImGui.textWrapped("E-stop active. Live sync requires a manual restart.");
        if (disconnected) {
            ImGui.textWrapped("Upload StandardFirmata (57600 baud) and assign servo pins in the Inspector.");
        } else if (connected) {
            ImGui.textWrapped("Live sync starts at the current pose. Servos can hold their last target after a stop.");
        }
    }

    private void refreshSerialPorts() {
        firmataMessage = "";
        try {
            serialPorts = FirmataBridge.serialPortNames();
        } catch (RuntimeException | LinkageError error) {
            serialPorts = List.of();
            firmataMessage = "Could not list serial ports. " + error.getMessage();
        }
        if (!serialPorts.contains(selectedPort)) {
            selectedPort = serialPorts.size() == 1 ? serialPorts.getFirst() : "";
        }
    }

    private void renderExportButtons() {
        MotionSequence sequence = motionPlayer == null ? null : motionPlayer.getSequence();
        boolean ready = sequence != null && !sequence.isIncomplete()
                && sequence.getKeyframes().stream().anyMatch(frame -> !frame.getJointAnglesDegrees().isEmpty());
        float speed = ready ? motionPlayer.getPlaybackSpeed() : 1.0f;

        ImGui.beginDisabled(!ready);
        exportButton("Export Arduino Sketch (.ino)", "ino",
                () -> TrajectoryExporter.toArduinoSketch(sequence, joints, EXPORT_RATE_HZ, speed));
        exportButton("Export Arduino Header (.h)", "h",
                () -> TrajectoryExporter.toArduinoHeader(sequence, joints, EXPORT_RATE_HZ, speed));
        exportButton("Export Raspberry Pi Script (.py)", "py",
                () -> TrajectoryExporter.toPythonScript(sequence, joints, EXPORT_RATE_HZ, speed));
        ImGui.endDisabled();

        if (!ready) {
            ImGui.textWrapped("Load or finish recording a motion to export code.");
        } else if (!statusMessage.isEmpty()) {
            ImGui.textWrapped(statusMessage);
        }
        if (ready && !sequence.getEvents().isEmpty()) {
            ImGui.textWrapped("Exports joint motion only. Scene object capture/release events are not exported.");
        }
    }

    private void exportButton(String label, String extension, Supplier<String> generateCode) {
        if (!ImGui.button(label, -1.0f, 0.0f)) return;

        statusMessage = "";
        try {
            saveCode(extension, generateCode.get());
        } catch (IllegalArgumentException error) {
            statusMessage = error.getMessage();
        }
    }

    private void saveCode(String extension, String code) {
        String suffix = "." + extension;
        try (MemoryStack stack = stackPush()) {
            PointerBuffer filters = stack.mallocPointer(1);
            filters.put(stack.UTF8("*" + suffix)).flip();
            String path = TinyFileDialogs.tinyfd_saveFileDialog(
                    "Export Code", "robot_trajectory" + suffix, filters, "*" + suffix);
            if (path == null || path.isBlank()) return;

            if (!path.toLowerCase(Locale.ROOT).endsWith(suffix)) path += suffix;
            statusMessage = TrajectoryExporter.saveToFile(code, path)
                    ? "Saved: " + path
                    : "Could not save: " + path;
        }
    }
}
