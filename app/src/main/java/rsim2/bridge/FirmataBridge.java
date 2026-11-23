package rsim2.bridge;

import com.fazecast.jSerialComm.SerialPort;
import org.firmata4j.Pin;
import org.firmata4j.firmata.FirmataDevice;
import org.firmata4j.firmata.FirmataMessageFactory;
import org.firmata4j.transport.TransportInterface;
import rsim2.scene.Joint;
import rsim2.scene.JointType;

import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.function.BiFunction;

public class FirmataBridge implements RobotBridge {
    public enum ConnectionState { DISCONNECTED, CONNECTING, CONNECTED, DISCONNECTING }

    public static final int BAUD_RATE = JSerialCommTransport.BAUD_RATE;

    private record Channel(Joint joint, int pin, boolean inverted, float offset) {
        int servoAngle(float simulationAngle) {
            double angle = 90.0 + offset + (inverted ? -simulationAngle : simulationAngle);
            if (!Double.isFinite(angle) || angle < -0.001 || angle > 180.001) {
                throw new IllegalArgumentException(String.format(Locale.ROOT,
                        "%s maps to %.2f servo degrees. Adjust its angle or offset to fit 0–180 degrees.",
                        joint.getId(), angle));
            }
            return (int) Math.round(angle);
        }
    }

    private record Frame(Map<String, Channel> channels, Map<Integer, Integer> angles) {}

    private final BridgeStats stats = new BridgeStats();
    private final BiFunction<String, Runnable, TransportInterface> transportFactory;

    private Thread worker;
    private boolean stopping;
    private Set<Integer> servoPins;
    private volatile Map<String, Channel> streamingChannels;
    private Frame pendingFrame;

    public FirmataBridge() {
        this(JSerialCommTransport::new);
    }

    FirmataBridge(BiFunction<String, Runnable, TransportInterface> transportFactory) {
        this.transportFactory = transportFactory;
    }

    public static List<String> serialPortNames() {
        return Arrays.stream(SerialPort.getCommPorts()).map(SerialPort::getSystemPortName).sorted().toList();
    }

    @Override
    public String getName() {
        return "Firmata (Arduino USB)";
    }

    @Override
    public synchronized boolean connect(String portName, int baud) {
        if (worker != null) return false;
        stats.reset();
        if (portName == null || portName.isBlank() || baud != BAUD_RATE) {
            stats.recordError("Select a serial port for StandardFirmata (57600 baud).");
            return false;
        }

        stopping = false;
        worker = new Thread(() -> runConnection(portName.strip()), "Firmata-Serial");
        worker.setDaemon(true);
        worker.start();
        return true;
    }

    @Override
    public synchronized void disconnect() {
        if (worker == null || stopping) return;
        stopping = true;
        disarm();
        if (servoPins == null) worker.interrupt();
        notifyAll();
    }

    public void close() {
        Thread connection;
        synchronized (this) {
            connection = worker;
            disconnect();
        }
        if (connection == null) return;
        try {
            connection.join(2000);
        } catch (InterruptedException error) {
            Thread.currentThread().interrupt();
        }
    }

    public synchronized ConnectionState getConnectionState() {
        if (worker == null) return ConnectionState.DISCONNECTED;
        if (stopping) return ConnectionState.DISCONNECTING;
        return servoPins == null ? ConnectionState.CONNECTING : ConnectionState.CONNECTED;
    }

    @Override
    public synchronized boolean isConnected() {
        return getConnectionState() == ConnectionState.CONNECTED;
    }

    boolean isStreaming() {
        return streamingChannels != null;
    }

    synchronized void arm(List<Joint> joints) {
        if (!isConnected()) throw new IllegalArgumentException("Connect to StandardFirmata first.");
        if (isStreaming()) return;

        Map<String, Channel> channels = readChannels(joints);
        for (Channel channel : channels.values()) {
            channel.servoAngle((float) Math.toDegrees(channel.joint().getCurrentAngleRadians()));
        }
        stats.clearLastError();
        streamingChannels = channels;
    }

    synchronized void disarm() {
        streamingChannels = null;
        pendingFrame = null;
    }

    synchronized void stopStreaming(String reason) {
        if (!isStreaming()) return;
        disarm();
        stats.recordError(reason);
    }

    synchronized boolean verifyMapping(List<Joint> joints) {
        if (!isStreaming()) return false;
        try {
            if (streamingChannels.equals(readChannels(joints))) return true;
            stopStreaming("Hardware mapping changed. Check the pins and restart live sync.");
        } catch (IllegalArgumentException error) {
            stopStreaming(error.getMessage());
        }
        return false;
    }

    private Map<String, Channel> readChannels(List<Joint> joints) {
        Map<String, Channel> channels = new LinkedHashMap<>();
        Set<Integer> usedPins = new HashSet<>();
        if (joints != null) {
            for (Joint joint : joints) {
                if (joint == null || joint.getType() != JointType.REVOLUTE || joint.getPin() == -1) continue;
                int pin = joint.getPin();
                String id = joint.getId();
                if (!"SERVO_PWM".equalsIgnoreCase(joint.getMotorType()) || !servoPins.contains(pin)) {
                    throw new IllegalArgumentException(id + " needs a PWM servo and a servo-capable board pin (set to " + pin + ").");
                }
                if (!usedPins.add(pin)) {
                    throw new IllegalArgumentException("Arduino pin " + pin + " is assigned more than once.");
                }
                Channel channel = new Channel(joint, pin, joint.isInverted(), joint.getZeroOffsetDeg());
                if (channels.putIfAbsent(id, channel) != null) {
                    throw new IllegalArgumentException("Duplicate joint ID: " + id);
                }
            }
        }
        if (channels.isEmpty()) {
            throw new IllegalArgumentException("Assign a PWM servo pin in Inspector > Hardware / Pin Mapping.");
        }
        return channels;
    }

    @Override
    public synchronized void sendJointPositions(Map<String, Float> simulationAngles, float timestampSeconds) {
        if (!isStreaming()) return;
        Map<Integer, Integer> angles = new LinkedHashMap<>();
        try {
            for (var entry : streamingChannels.entrySet()) {
                Float angle = simulationAngles.get(entry.getKey());
                if (angle == null) throw new IllegalArgumentException("No valid angle for " + entry.getKey() + ".");
                Channel channel = entry.getValue();
                angles.put(channel.pin(), channel.servoAngle(angle));
            }
        } catch (IllegalArgumentException error) {
            stopStreaming(error.getMessage());
            return;
        }
        pendingFrame = new Frame(streamingChannels, angles);
        notifyAll();
    }

    @Override
    public void sendEmergencyStop() {
        disarm();
    }

    @Override
    public BridgeStats getStats() {
        return stats;
    }

    private void runConnection(String portName) {
        FirmataDevice device = null;
        TransportInterface transport = null;
        try {
            if (Thread.currentThread().isInterrupted()) return;
            transport = transportFactory.apply(portName, this::onPortDisconnected);
            device = new FirmataDevice(transport);
            device.start();
            device.ensureInitializationIsDone();

            Set<Integer> supportedPins = new HashSet<>();
            for (Pin pin : device.getPins()) {
                int number = Byte.toUnsignedInt(pin.getIndex());
                if (number < 128 && pin.getSupportedModes().contains(Pin.Mode.SERVO)) supportedPins.add(number);
            }
            synchronized (this) {
                if (stopping) return;
                servoPins = Set.copyOf(supportedPins);
            }
            writeFrames(device);
        } catch (InterruptedException error) {
            recordConnectionError("StandardFirmata did not respond on " + portName + ".");
        } catch (IOException | RuntimeException | LinkageError error) {
            recordConnectionError("Firmata on " + portName + ": " + error.getMessage());
        } finally {
            disconnect();
            Thread.interrupted();
            try {
                if (transport instanceof JSerialCommTransport serial) serial.prepareForClose();
                if (device != null) device.stop();
            } catch (IOException | RuntimeException error) {
                if (stats.getLastError().isEmpty()) stats.recordError(error.getMessage());
            } finally {
                synchronized (this) {
                    servoPins = null;
                    worker = null;
                }
            }
        }
    }

    private synchronized Frame takeFrame() throws InterruptedException {
        while (!stopping && pendingFrame == null) wait();
        if (stopping) return null;
        Frame frame = pendingFrame;
        pendingFrame = null;
        return frame;
    }

    private void writeFrames(FirmataDevice device) throws IOException, InterruptedException {
        Map<Integer, Integer> sentAngles = new HashMap<>();
        for (Frame frame; (frame = takeFrame()) != null;) {
            int bytesSent = 0;
            for (var entry : frame.angles().entrySet()) {
                if (streamingChannels != frame.channels()) break;
                int pin = entry.getKey();
                int angle = entry.getValue();
                Integer previous = sentAngles.get(pin);
                if (previous == null) {
                    byte[] mode = FirmataMessageFactory.setMode((byte) pin, Pin.Mode.SERVO);
                    device.sendMessage(mode);
                    bytesSent += mode.length;
                }
                if (streamingChannels != frame.channels()) break;
                if (previous == null || previous != angle) {
                    byte[] position = FirmataMessageFactory.setAnalogPinValue((byte) pin, angle);
                    device.sendMessage(position);
                    sentAngles.put(pin, angle);
                    bytesSent += position.length;
                }
            }
            if (bytesSent > 0) stats.recordPacketSent(bytesSent);
        }
    }

    private synchronized void onPortDisconnected() {
        recordConnectionError("Arduino serial port disconnected.");
        disconnect();
    }

    private synchronized void recordConnectionError(String message) {
        if (!stopping) stats.recordError(message);
    }
}
