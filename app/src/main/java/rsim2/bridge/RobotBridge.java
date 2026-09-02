package rsim2.bridge;

import java.util.Map;

public interface RobotBridge {

    @FunctionalInterface
    interface TelemetryListener {
        void onJointAnglesReceived(Map<String, Float> jointAnglesDegrees);
    }

    String getName();

    boolean connect(String targetAddress, int portOrBaud);

    void disconnect();

    boolean isConnected();

    void sendJointPositions(Map<String, Float> jointAnglesDegrees, float timestampSeconds);

    void sendEmergencyStop();

    BridgeStats getStats();

    default void setTelemetryListener(TelemetryListener listener) {}

    default boolean isListening() {
        return false;
    }
}

