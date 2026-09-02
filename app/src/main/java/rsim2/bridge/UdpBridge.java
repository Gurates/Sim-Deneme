package rsim2.bridge;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;

public class UdpBridge implements RobotBridge {
    private final BridgeStats stats = new BridgeStats();
    private final Gson gson = new Gson();

    private DatagramSocket socket;
    private InetAddress targetAddress;
    private int targetPort = 8888;
    private boolean connected = false;
    private String targetHost = "192.168.1.50";

    private long sequenceNumber = 0;

    private static final int ESTOP_BURST_COUNT = 5;

    private byte[] sendBuffer = new byte[2048];

    private TelemetryListener telemetryListener;
    private Thread listenerThread;

    @Override
    public String getName() {
        return "UDP Socket (ESP32 / Wi-Fi / Ethernet)";
    }

    @Override
    public synchronized boolean connect(String host, int port) {
        disconnect();
        try {
            this.targetHost = (host != null && !host.trim().isEmpty()) ? host.trim() : "127.0.0.1";
            this.targetPort = port > 0 ? port : 8888;
            this.targetAddress = InetAddress.getByName(this.targetHost);
            this.socket = new DatagramSocket();
            this.socket.setSoTimeout(1000);
            this.connected = true;
            this.sequenceNumber = 0;
            this.stats.reset();

            this.listenerThread = new Thread(this::listenLoop, "UdpBridge-Receiver");
            this.listenerThread.setDaemon(true);
            this.listenerThread.start();

            return true;
        } catch (Exception e) {
            stats.recordError("UDP Connect Failed: " + e.getMessage());
            connected = false;
            return false;
        }
    }

    @Override
    public synchronized void disconnect() {
        connected = false;
        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
        socket = null;
        if (listenerThread != null) {
            listenerThread.interrupt();
            listenerThread = null;
        }
    }

    private void listenLoop() {
        byte[] rxBuffer = new byte[4096];
        DatagramPacket rxPacket = new DatagramPacket(rxBuffer, rxBuffer.length);
        while (connected && socket != null && !socket.isClosed()) {
            try {
                socket.receive(rxPacket);
                String msg = new String(rxPacket.getData(), rxPacket.getOffset(), rxPacket.getLength(), StandardCharsets.UTF_8).trim();
                stats.recordPacketReceived();
                if (!msg.isEmpty() && telemetryListener != null) {
                    try {
                        JsonObject json = gson.fromJson(msg, JsonObject.class);
                        if (json != null && json.has("joints") && json.get("joints").isJsonObject()) {
                            JsonObject jointsObj = json.getAsJsonObject("joints");
                            Map<String, Float> angles = new HashMap<>();
                            for (String key : jointsObj.keySet()) {
                                angles.put(key, jointsObj.get(key).getAsFloat());
                            }
                            telemetryListener.onJointAnglesReceived(angles);
                        }
                    } catch (Exception ignored) {
                    }
                }
            } catch (SocketTimeoutException ignored) {
            } catch (SocketException ignored) {
                break;
            } catch (Exception e) {
                if (connected) {
                    stats.recordError("UDP Rx Error: " + e.getMessage());
                }
            }
        }
    }

    @Override
    public synchronized void setTelemetryListener(TelemetryListener listener) {
        this.telemetryListener = listener;
    }

    @Override
    public synchronized boolean isListening() {
        return connected && listenerThread != null && listenerThread.isAlive();
    }

    @Override
    public synchronized boolean isConnected() {
        return connected && socket != null && !socket.isClosed();
    }

    @Override
    public synchronized void sendJointPositions(Map<String, Float> jointAnglesDegrees, float timestampSeconds) {
        if (!isConnected()) return;

        try {
            JsonObject json = new JsonObject();
            json.addProperty("type", "cmd");
            json.addProperty("seq", sequenceNumber++);
            json.addProperty("t", timestampSeconds);

            JsonObject jointsObj = new JsonObject();
            if (jointAnglesDegrees != null) {
                for (Map.Entry<String, Float> entry : jointAnglesDegrees.entrySet()) {
                    jointsObj.addProperty(entry.getKey(), Math.round(entry.getValue() * 100.0f) / 100.0f);
                }
            }
            json.add("joints", jointsObj);

            sendPacket(gson.toJson(json));
        } catch (Exception e) {
            handleSendError("UDP Send Error: " + e.getMessage());
        }
    }

    @Override
    public synchronized void sendEmergencyStop() {
        if (!isConnected()) return;

        for (int i = 0; i < ESTOP_BURST_COUNT; i++) {
            try {
                JsonObject json = new JsonObject();
                json.addProperty("type", "estop");
                json.addProperty("seq", sequenceNumber++);
                json.addProperty("msg", "EMERGENCY_STOP");

                sendPacket(gson.toJson(json));

                if (i < ESTOP_BURST_COUNT - 1) {
                    Thread.sleep(5);
                }
            } catch (Exception e) {
                stats.recordError("UDP E-Stop Send Error: " + e.getMessage());
            }
        }
    }

    private void sendPacket(String jsonString) throws Exception {
        byte[] data = (jsonString + "\n").getBytes(StandardCharsets.UTF_8);
        DatagramPacket packet = new DatagramPacket(data, data.length, targetAddress, targetPort);
        socket.send(packet);
        stats.recordPacketSent(data.length);
    }

    private void handleSendError(String errorMsg) {
        stats.recordError(errorMsg);
        if (stats.getErrorCount() > 10) {
            connected = false;
        }
    }

    @Override
    public BridgeStats getStats() {
        return stats;
    }

    public String getTargetHost() {
        return targetHost;
    }

    public int getTargetPort() {
        return targetPort;
    }
}
