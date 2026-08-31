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
    private RobotBridge activeBridge;
    private final SafetyFilter safetyFilter = new SafetyFilter();

    private boolean liveSyncEnabled = false;
    private int targetPublishRateHz = 50;
    private long minPublishIntervalNanos = 20_000_000L;
    private long lastPublishTimeNanos = 0;

    private final Map<String, Float> reusableDegreesMap = new HashMap<>();

    private long ppsWindowStartNanos = 0;
    private int ppsPacketCount = 0;

    private BridgeManager() {
        UdpBridge udp = new UdpBridge();
        MockHardwareBridge mock = new MockHardwareBridge();

        availableBridges.add(udp);
        availableBridges.add(mock);

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

    public synchronized void setActiveBridge(RobotBridge bridge) {
        if (this.activeBridge != null && this.activeBridge.isConnected()) {
            this.activeBridge.disconnect();
        }
        this.activeBridge = bridge != null ? bridge : availableBridges.get(0);
    }

    public SafetyFilter getSafetyFilter() {
        return safetyFilter;
    }

    public boolean isLiveSyncEnabled() {
        return liveSyncEnabled;
    }

    public void setLiveSyncEnabled(boolean liveSyncEnabled) {
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
        if (!liveSyncEnabled || activeBridge == null || !activeBridge.isConnected()) {
            return;
        }

        if (safetyFilter.isEmergencyStopActive()) {
            return;
        }

        long now = System.nanoTime();
        if (lastPublishTimeNanos != 0 && (now - lastPublishTimeNanos) < minPublishIntervalNanos) {
            return;
        }
        lastPublishTimeNanos = now;

        Map<String, Float> safeAngles = safetyFilter.processAndFilter(rawAnglesDegrees, joints);

        activeBridge.sendJointPositions(safeAngles, timestamp);

        updatePacketsPerSecond(now);
    }

    public synchronized void streamCurrentJointStates(List<Joint> joints, float timestamp) {
        if (joints == null || !liveSyncEnabled) return;

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
