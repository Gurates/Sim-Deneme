package rsim2.telemetry;

import rsim2.motion.Keyframe;
import rsim2.motion.MotionSequence;
import rsim2.scene.Joint;

import java.io.File;
import java.io.IOException;
import java.util.*;

public class TrajectoryExporter {
    public static String toArduinoHeader(MotionSequence sequence, List<Joint> joints, int sampleRateHz) {
        return toArduinoHeader(sequence, joints, sampleRateHz, 1);
    }

    public static String toArduinoHeader(MotionSequence sequence, List<Joint> joints, int sampleRateHz, float speed) {
        return ServoCodeWriter.arduinoHeader(compile(sequence, joints, sampleRateHz, speed));
    }

    public static String toArduinoSketch(MotionSequence sequence, List<Joint> joints, int sampleRateHz) {
        return toArduinoSketch(sequence, joints, sampleRateHz, 1);
    }

    public static String toArduinoSketch(MotionSequence sequence, List<Joint> joints, int sampleRateHz, float speed) {
        return ServoCodeWriter.arduinoSketch(compile(sequence, joints, sampleRateHz, speed));
    }

    public static String toArduinoSketch(MotionSequence sequence, List<Joint> joints, int sampleRateHz,
                                         String selectedJoint, int selectedPin, boolean centered) {
        return ServoCodeWriter.arduinoSketch(
                ServoTrajectory.compile(sequence, joints, sampleRateHz, 1, selectedJoint, selectedPin, centered));
    }

    public static String toPythonScript(MotionSequence sequence, List<Joint> joints, int sampleRateHz) {
        return toPythonScript(sequence, joints, sampleRateHz, 1);
    }

    public static String toPythonScript(MotionSequence sequence, List<Joint> joints, int sampleRateHz, float speed) {
        return ServoCodeWriter.python(compile(sequence, joints, sampleRateHz, speed));
    }

    private static ServoTrajectory compile(MotionSequence sequence, List<Joint> joints, int rateHz, float speed) {
        return ServoTrajectory.compile(sequence, joints, rateHz, speed, null, -1, true);
    }

    public static String toCsv(MotionSequence sequence, int sampleRateHz) {
        if (sequence == null) return "time\n";

        float duration = sequence.getDurationSeconds();
        int totalFrames = Math.max(2, (int) (duration * sampleRateHz));
        float dt = duration / (float) (totalFrames - 1);

        Set<String> jointNamesSet = new LinkedHashSet<>();
        for (Keyframe kf : sequence.getKeyframes()) {
            jointNamesSet.addAll(kf.getJointAnglesDegrees().keySet());
        }
        List<String> jointNames = new ArrayList<>(jointNamesSet);
        Collections.sort(jointNames);

        StringBuilder sb = new StringBuilder();
        sb.append("time_sec");
        for (String name : jointNames) {
            sb.append(",").append(name);
        }
        sb.append("\n");

        Map<String, Float> sampled = new HashMap<>();
        for (int f = 0; f < totalFrames; f++) {
            float t = f * dt;
            sequence.sample(t, sampled);

            sb.append(String.format(Locale.US, "%.4f", t));
            for (String name : jointNames) {
                float deg = sampled.getOrDefault(name, 0.0f);
                sb.append(String.format(Locale.US, ",%.2f", deg));
            }
            sb.append("\n");
        }

        return sb.toString();
    }

    public static String toRpiLiveReceiverScript() {
        return "#!/usr/bin/env python3\n" +
               "# ==========================================================\n" +
               "# RSim2 Raspberry Pi Live UDP Receiver Daemon\n" +
               "# Listens to real-time motion streamed from RSim2 over Wi-Fi/Ethernet\n" +
               "# ==========================================================\n\n" +
               "import socket\n" +
               "import json\n" +
               "import sys\n\n" +
               "UDP_HOST = \"0.0.0.0\"  # Listen on all local interfaces\n" +
               "UDP_PORT = 8888\n" +
               "USE_PCA9685 = False  # Set to True if using I2C PCA9685 Hat\n\n" +
               "# Map joint names to GPIO pins or PCA9685 channels:\n" +
               "PIN_MAPPING = {\n" +
               "    \"joint_1\": 18,\n" +
               "    \"joint_2\": 19,\n" +
               "    \"joint_3\": 20,\n" +
               "    \"joint_4\": 21\n" +
               "}\n\n" +
               "servos = {}\n\n" +
               "def init_hardware():\n" +
               "    global servos\n" +
               "    if USE_PCA9685:\n" +
               "        try:\n" +
               "            from adafruit_servokit import ServoKit\n" +
               "            kit = ServoKit(channels=16)\n" +
               "            for name, pin in PIN_MAPPING.items():\n" +
               "                servos[name] = kit.servo[pin]\n" +
               "            print(f\"[INFO] PCA9685 initialized for {len(servos)} joints.\")\n" +
               "        except ImportError:\n" +
               "            print(\"[ERROR] adafruit-circuitpython-servokit not installed!\")\n" +
               "            sys.exit(1)\n" +
               "    else:\n" +
               "        try:\n" +
               "            from gpiozero import AngularServo\n" +
               "            for name, pin in PIN_MAPPING.items():\n" +
               "                servos[name] = AngularServo(pin, min_angle=0, max_angle=180)\n" +
               "            print(f\"[INFO] gpiozero initialized for {len(servos)} joints.\")\n" +
               "        except ImportError:\n" +
               "            print(\"[ERROR] gpiozero not installed!\")\n" +
               "            sys.exit(1)\n\n" +
               "def main():\n" +
               "    init_hardware()\n" +
               "    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)\n" +
               "    sock.bind((UDP_HOST, UDP_PORT))\n" +
               "    print(f\"[READY] RSim2 Live Receiver listening on port {UDP_PORT}...\")\n\n" +
               "    try:\n" +
               "        while True:\n" +
               "            data, addr = sock.recvfrom(4096)\n" +
               "            try:\n" +
               "                payload = json.loads(data.decode('utf-8').strip())\n" +
               "                msg_type = payload.get(\"type\")\n" +
               "                if msg_type == \"estop\":\n" +
               "                    print(\"[WARNING] EMERGENCY STOP RECEIVED FROM RSIM2!\")\n" +
               "                elif msg_type == \"cmd\":\n" +
               "                    joints = payload.get(\"joints\", {})\n" +
               "                    for name, deg in joints.items():\n" +
               "                        if name in servos:\n" +
               "                            angle = max(0.0, min(180.0, float(deg)))\n" +
               "                            servos[name].angle = angle\n" +
               "            except Exception as e:\n" +
               "                pass\n" +
               "    except KeyboardInterrupt:\n" +
               "        print(\"\\n[STOP] Receiver shutting down.\")\n" +
               "    finally:\n" +
               "        sock.close()\n\n" +
               "if __name__ == \"__main__\":\n" +
               "    main()\n";
    }

    public static boolean saveToFile(String content, String filePath) {
        try {
            File file = new File(filePath);
            File parent = file.getParentFile();
            if (parent != null && !parent.exists()) {
                parent.mkdirs();
            }
            try (java.io.OutputStreamWriter writer = new java.io.OutputStreamWriter(
                    new java.io.FileOutputStream(file), java.nio.charset.StandardCharsets.UTF_8)) {
                writer.write(content);
            }
            return true;
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
    }
}
