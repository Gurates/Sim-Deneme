package rsim2.bridge;

import java.util.HashMap;
import java.util.Map;

public class MockHardwareBridge implements RobotBridge {
    private final BridgeStats stats = new BridgeStats();
    private boolean connected = false;
    private final Map<String, Float> lastReceivedJoints = new HashMap<>();

    @Override
    public String getName() {
        return "Mock Loopback (Testing & Diagnostics)";
    }

    @Override
    public synchronized boolean connect(String targetAddress, int portOrBaud) {
        this.connected = true;
        this.stats.reset();
        return true;
    }

    @Override
    public synchronized void disconnect() {
        this.connected = false;
    }

    @Override
    public synchronized boolean isConnected() {
        return connected;
    }

    @Override
    public synchronized void sendJointPositions(Map<String, Float> jointAnglesDegrees, float timestampSeconds) {
        if (!connected) return;

        lastReceivedJoints.clear();
        if (jointAnglesDegrees != null) {
            lastReceivedJoints.putAll(jointAnglesDegrees);
        }

        stats.recordPacketSent(64);
        stats.recordPacketReceived();
        stats.setLatencyMs(0.45f);
    }

    @Override
    public synchronized void sendEmergencyStop() {
        if (!connected) return;
        lastReceivedJoints.clear();
        stats.recordPacketSent(32);
    }

    @Override
    public BridgeStats getStats() {
        return stats;
    }

    private TelemetryListener telemetryListener;

    @Override
    public synchronized void setTelemetryListener(TelemetryListener listener) {
        this.telemetryListener = listener;
    }

    @Override
    public synchronized boolean isListening() {
        return connected && telemetryListener != null;
    }

    public synchronized void simulateIncomingTelemetry(Map<String, Float> angles) {
        if (connected && telemetryListener != null && angles != null) {
            telemetryListener.onJointAnglesReceived(angles);
            stats.recordPacketReceived();
        }
    }

    public synchronized Map<String, Float> getLastReceivedJoints() {
        return new HashMap<>(lastReceivedJoints);
    }
}

