package rsim2.bridge;

import rsim2.safety.SafetyFilter;
import rsim2.scene.Joint;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class BridgeManager {
    private static final BridgeManager INSTANCE = new BridgeManager();

    private final List<RobotBridge> availableBridges = new ArrayList<>();
    private final FirmataBridge firmata = new FirmataBridge();
    private RobotBridge activeBridge;
    private final SafetyFilter safetyFilter = new SafetyFilter();

    private boolean liveSyncEnabled = false;
    private int targetPublishRateHz = 50;
    private long minPublishIntervalNanos = 20_000_000L;
    private long lastPublishTimeNanos = 0;

    private final Map<String, Float> reusableDegreesMap = new HashMap<>();

    private long ppsWindowStartNanos = 0;
    private int ppsPacketCount = 0;

    private boolean digitalTwinEnabled = false;
    private List<Joint> registeredJoints;
    private final Map<String, Float> lastReceivedTelemetryAngles = new HashMap<>();

    private BridgeManager() {
        UdpBridge udp = new UdpBridge();
        MockHardwareBridge mock = new MockHardwareBridge();

        availableBridges.add(udp);
        availableBridges.add(mock);
        availableBridges.add(firmata);

        for (RobotBridge b : availableBridges) {
            b.setTelemetryListener(this::onTelemetryReceived);
        }

        this.activeBridge = udp;
    }

    public static BridgeManager getInstance() {
        return INSTANCE;
    }

    public List<RobotBridge> getAvailableBridges() {
        return availableBridges;
    }

    public RobotBridge getActiveBridge() {
        return activeBridge;
    }

    public FirmataBridge getFirmataBridge() {
        return firmata;
    }

    public synchronized boolean connectFirmata(String portName) {
        if (firmata.getConnectionState() != FirmataBridge.ConnectionState.DISCONNECTED) return false;
        setActiveBridge(firmata);
        safetyFilter.clearAngleHistory();
        return firmata.connect(portName, FirmataBridge.BAUD_RATE);
    }

    public synchronized void setActiveBridge(RobotBridge bridge) {
        if (this.activeBridge != null) {
            this.activeBridge.disconnect();
        }
        liveSyncEnabled = false;
        lastPublishTimeNanos = 0;
        this.activeBridge = bridge != null ? bridge : availableBridges.get(0);
        this.activeBridge.setTelemetryListener(this::onTelemetryReceived);
    }

    public synchronized void registerJoints(List<Joint> joints) {
        if (registeredJoints != joints && activeBridge instanceof FirmataBridge serial) {
            serial.stopStreaming("Robot changed. Check the hardware mapping and restart live sync.");
            safetyFilter.clearAngleHistory();
        }
        this.registeredJoints = joints;
    }

    public synchronized boolean isDigitalTwinEnabled() {
        return digitalTwinEnabled;
    }

    public synchronized void setDigitalTwinEnabled(boolean enabled) {
        this.digitalTwinEnabled = enabled;
    }

    public synchronized Map<String, Float> getLastReceivedTelemetryAngles() {
        return new HashMap<>(lastReceivedTelemetryAngles);
    }

    public synchronized void onTelemetryReceived(Map<String, Float> angles) {
        if (angles == null) return;
        lastReceivedTelemetryAngles.putAll(angles);
        if (digitalTwinEnabled && registeredJoints != null) {
            for (Joint j : registeredJoints) {
                if (j == null || j.getId() == null) continue;
                Float deg = angles.get(j.getId());
                if (deg != null && Float.isFinite(deg)) {
                    float adjustedDeg = deg;
                    adjustedDeg -= j.getZeroOffsetDeg();
                    if (j.isInverted()) {
                        adjustedDeg = 180.0f - adjustedDeg;
                    }
                    float rad = (float) Math.toRadians(adjustedDeg);
                    j.setAngle(rad);
                    if (j.getMotor() != null) {
                        j.getMotor().setTargetAngleRadians(rad);
                    }
                }
            }
        }
    }

    public SafetyFilter getSafetyFilter() {
        return safetyFilter;
    }

    public synchronized boolean isLiveSyncEnabled() {
        return activeBridge instanceof FirmataBridge serial ? serial.isStreaming() : liveSyncEnabled;
    }

    public synchronized void setLiveSyncEnabled(boolean liveSyncEnabled) {
        if (activeBridge instanceof FirmataBridge serial) {
            if (liveSyncEnabled) {
                if (safetyFilter.isEmergencyStopActive()) throw new IllegalArgumentException("Reset E-stop before starting live sync.");
                if (collisionBlocksStreaming()) throw new IllegalArgumentException("Resolve the collision before starting live sync.");
                serial.arm(registeredJoints);
                lastPublishTimeNanos = 0;
            } else {
                serial.disarm();
            }
            return;
        }
        this.liveSyncEnabled = liveSyncEnabled;
    }

    public int getTargetPublishRateHz() {
        return targetPublishRateHz;
    }

    public void setTargetPublishRateHz(int rateHz) {
        this.targetPublishRateHz = Math.max(1, Math.min(200, rateHz));
        this.minPublishIntervalNanos = (long) (1_000_000_000.0 / this.targetPublishRateHz);
    }

    public synchronized void streamJointPositions(Map<String, Float> rawAnglesDegrees, float timestamp, List<Joint> joints) {
        if (!isLiveSyncEnabled() || activeBridge == null || !activeBridge.isConnected()) {
            return;
        }

        boolean emergencyStopped = safetyFilter.isEmergencyStopActive();
        FirmataBridge serial = activeBridge instanceof FirmataBridge bridge ? bridge : null;
        if (serial != null && (emergencyStopped || collisionBlocksStreaming())) {
            serial.stopStreaming(emergencyStopped ? "E-stop activated." : "Collision detected. Resolve it before restarting live sync.");
            return;
        }
        if (emergencyStopped) {
            return;
        }

        long now = System.nanoTime();
        if (lastPublishTimeNanos != 0 && (now - lastPublishTimeNanos) < minPublishIntervalNanos) {
            return;
        }
        lastPublishTimeNanos = now;

        if (serial != null && !serial.verifyMapping(joints)) return;
        Map<String, Float> safeAngles = serial == null
                ? safetyFilter.processAndFilter(rawAnglesDegrees, joints)
                : safetyFilter.processSimulationAngles(rawAnglesDegrees, joints);

        activeBridge.sendJointPositions(safeAngles, timestamp);

        updatePacketsPerSecond(now);
    }

    public synchronized void streamCurrentJointStates(List<Joint> joints, float timestamp) {
        if (joints == null) return;
        registerJoints(joints);
        if (!isLiveSyncEnabled()) return;

        reusableDegreesMap.clear();
        for (Joint j : joints) {
            if (j != null && j.getId() != null) {
                reusableDegreesMap.put(j.getId(), (float) Math.toDegrees(j.getCurrentAngleRadians()));
            }
        }
        streamJointPositions(reusableDegreesMap, timestamp, joints);
    }

    public synchronized void triggerEmergencyStop() {
        safetyFilter.triggerEmergencyStop();
        if (activeBridge != null && activeBridge.isConnected()) {
            activeBridge.sendEmergencyStop();
        }
    }

    public synchronized void resetEmergencyStop() {
        safetyFilter.resetEmergencyStop();
    }

    private boolean collisionBlocksStreaming() {
        return safetyFilter.isPreventCollisionStreaming() && safetyFilter.isCollisionStopActive();
    }

    private void updatePacketsPerSecond(long nowNanos) {
        ppsPacketCount++;
        if (ppsWindowStartNanos == 0) {
            ppsWindowStartNanos = nowNanos;
        }

        long elapsed = nowNanos - ppsWindowStartNanos;
        if (elapsed >= 1_000_000_000L) {
            float pps = ppsPacketCount / (elapsed / 1_000_000_000.0f);
            if (activeBridge != null) {
                activeBridge.getStats().updateRate(pps);
            }
            ppsPacketCount = 0;
            ppsWindowStartNanos = nowNanos;
        }
    }
}
