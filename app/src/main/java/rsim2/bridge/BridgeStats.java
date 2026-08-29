package rsim2.bridge;

public class BridgeStats {
    private long packetsSent = 0;
    private long packetsReceived = 0;
    private long bytesSent = 0;
    private long lastPacketTimeMs = 0;
    private float packetsPerSecond = 0.0f;
    private float roundTripLatencyMs = 0.0f;
    private long errorCount = 0;
    private String lastError = "";

    public synchronized void recordPacketSent(int byteLength) {
        packetsSent++;
        bytesSent += byteLength;
        lastPacketTimeMs = System.currentTimeMillis();
    }

    public synchronized void recordPacketReceived() {
        packetsReceived++;
    }

    public synchronized void recordError(String error) {
        errorCount++;
        this.lastError = error != null ? error : "Unknown error";
    }

    public synchronized void updateRate(float pps) {
        this.packetsPerSecond = pps;
    }

    public synchronized void setLatencyMs(float latencyMs) {
        this.roundTripLatencyMs = latencyMs;
    }

    public synchronized void reset() {
        packetsSent = 0;
        packetsReceived = 0;
        bytesSent = 0;
        lastPacketTimeMs = 0;
        packetsPerSecond = 0.0f;
        roundTripLatencyMs = 0.0f;
        errorCount = 0;
        lastError = "";
    }

    public synchronized long getPacketsSent() { return packetsSent; }
    public synchronized long getPacketsReceived() { return packetsReceived; }
    public synchronized long getBytesSent() { return bytesSent; }
    public synchronized long getLastPacketTimeMs() { return lastPacketTimeMs; }
    public synchronized float getPacketsPerSecond() { return packetsPerSecond; }
    public synchronized float getRoundTripLatencyMs() { return roundTripLatencyMs; }
    public synchronized long getErrorCount() { return errorCount; }
    public synchronized String getLastError() { return lastError; }
}
