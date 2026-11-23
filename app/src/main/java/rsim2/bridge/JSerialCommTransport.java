package rsim2.bridge;

import com.fazecast.jSerialComm.SerialPort;
import com.fazecast.jSerialComm.SerialPortDataListener;
import com.fazecast.jSerialComm.SerialPortEvent;
import org.firmata4j.Parser;
import org.firmata4j.transport.TransportInterface;

import java.io.IOException;

final class JSerialCommTransport implements TransportInterface, SerialPortDataListener {
    static final int BAUD_RATE = 57600;

    private final SerialPort port;
    private final Runnable onDisconnect;
    private Parser parser;
    private volatile boolean closing;

    JSerialCommTransport(String portName, Runnable onDisconnect) {
        this.port = SerialPort.getCommPort(portName);
        this.onDisconnect = onDisconnect;
        port.setComPortParameters(BAUD_RATE, 8, SerialPort.ONE_STOP_BIT, SerialPort.NO_PARITY);
    }

    @Override
    public void start() throws IOException {
        if (!port.openPort()) {
            throw new IOException("Could not open " + port.getSystemPortName() + ". Check the USB connection and close its serial monitor.");
        }
        if (!port.addDataListener(this)) {
            port.closePort();
            throw new IOException("Could not read from " + port.getSystemPortName() + ".");
        }
    }

    @Override
    public void stop() throws IOException {
        port.removeDataListener();
        if (port.isOpen() && !port.closePort()) {
            throw new IOException("Could not close " + port.getSystemPortName() + ".");
        }
    }

    @Override
    public void write(byte[] bytes) throws IOException {
        if (!closing && port.writeBytes(bytes, bytes.length) != bytes.length) {
            throw new IOException("Could not write to " + port.getSystemPortName() + ".");
        }
    }

    void prepareForClose() {
        closing = true;
    }

    @Override
    public void setParser(Parser parser) {
        this.parser = parser;
    }

    @Override
    public int getListeningEvents() {
        return SerialPort.LISTENING_EVENT_DATA_RECEIVED | SerialPort.LISTENING_EVENT_PORT_DISCONNECTED;
    }

    @Override
    public void serialEvent(SerialPortEvent event) {
        if (event.getEventType() == SerialPort.LISTENING_EVENT_PORT_DISCONNECTED) {
            onDisconnect.run();
        } else if (event.getEventType() == SerialPort.LISTENING_EVENT_DATA_RECEIVED) {
            parser.parse(event.getReceivedData());
        }
    }
}
