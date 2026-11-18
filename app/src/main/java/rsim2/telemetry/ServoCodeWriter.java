package rsim2.telemetry;

import java.util.StringJoiner;

final class ServoCodeWriter {
    private ServoCodeWriter() {}

    static String arduinoHeader(ServoTrajectory trajectory) {
        return "#pragma once\n" + arduinoData(trajectory);
    }

    static String arduinoSketch(ServoTrajectory trajectory) {
        StringBuilder code = new StringBuilder(arduinoData(trajectory));
        code.append("#include <Servo.h>\n\n");
        code.append("#ifdef MAX_SERVOS\nstatic_assert(TRAJECTORY_JOINTS <= MAX_SERVOS, \"Too many servos for this board\");\n#endif\n");
        for (int i = 0; i < trajectory.channels().size(); i++) {
            code.append("#ifdef NUM_DIGITAL_PINS\nstatic_assert(SERVOS[").append(i)
                    .append("].pin < NUM_DIGITAL_PINS, \"Invalid servo pin for this board\");\n#endif\n");
            code.append("static_assert(SERVOS[").append(i).append("].minUs >= MIN_PULSE_WIDTH && SERVOS[")
                    .append(i).append("].maxUs <= MAX_PULSE_WIDTH && SERVOS[").append(i)
                    .append("].minUs < SERVOS[").append(i).append("].maxUs, \"Invalid servo pulse range\");\n");
        }
        code.append("constexpr uint32_t MAX_LATENESS_US = ").append(ServoTrajectory.MAX_LATENESS_US).append(";\n");
        return code.append("""

                Servo outputs[TRAJECTORY_JOINTS];
                bool playEnabled = false;

                void writeTarget(uint32_t frame, size_t joint) {
                    const uint16_t angle = pgm_read_word(&TRAJECTORY_DATA[frame][joint]);
                    const auto& cfg = SERVOS[joint];
                    const uint32_t span = cfg.maxUs - cfg.minUs;
                    outputs[joint].writeMicroseconds(cfg.minUs + (span * angle + 9000) / 18000);
                }

                bool playTrajectory() {
                    const uint32_t start = micros();
                    for (uint32_t frame = 0; frame < TRAJECTORY_FRAMES; ++frame) {
                        const uint32_t deadline = pgm_read_dword(&TRAJECTORY_TIME_US[frame]);
                        uint32_t elapsed;
                        while ((elapsed = (uint32_t)(micros() - start)) < deadline) {
                            delayMicroseconds(50);
                        }
                        if (elapsed - deadline > MAX_LATENESS_US) return false;
                        for (size_t joint = 0; joint < TRAJECTORY_JOINTS; ++joint) writeTarget(frame, joint);
                    }
                    return true;
                }

                void setup() {
                    for (size_t joint = 0; joint < TRAJECTORY_JOINTS; ++joint) {
                        outputs[joint].attach(SERVOS[joint].pin);
                        writeTarget(0, joint);
                    }
                    delay(1000); // Let the servos reach the starting pose before starting the clock.
                    playEnabled = true;
                }

                void loop() {
                    if (playEnabled) playEnabled = playTrajectory() && TRAJECTORY_LOOP;
                }
                """).toString();
    }

    private static String arduinoData(ServoTrajectory trajectory) {
        StringBuilder code = new StringBuilder();
        describe(code, "//", trajectory);
        code.append("#include <Arduino.h>\n\n");
        code.append("constexpr uint32_t TRAJECTORY_FRAMES = ").append(trajectory.samples().size()).append(";\n");
        code.append("constexpr size_t TRAJECTORY_JOINTS = ").append(trajectory.channels().size()).append(";\n");
        code.append("constexpr bool TRAJECTORY_LOOP = ").append(trajectory.loop()).append(";\n\n");
        code.append("struct ServoChannel { uint8_t pin; uint16_t minUs; uint16_t maxUs; };\n");
        code.append("constexpr ServoChannel SERVOS[TRAJECTORY_JOINTS] = {\n");
        for (var channel : trajectory.channels()) {
            code.append("    { ").append(channel.pin()).append(", ").append(ServoTrajectory.MIN_PULSE_US)
                    .append(", ").append(ServoTrajectory.MAX_PULSE_US).append(" }, // ").append(comment(channel.id())).append('\n');
        }
        code.append("};\n\nconst uint32_t TRAJECTORY_TIME_US[TRAJECTORY_FRAMES] PROGMEM = {\n");
        for (int i = 0; i < trajectory.samples().size(); i++) {
            if (i % 10 == 0) code.append("    ");
            code.append(trajectory.samples().get(i).timeUs()).append(i % 10 == 9 ? ",\n" : ", ");
        }
        if (trajectory.samples().size() % 10 != 0) code.append('\n');
        code.append("};\n\nconst uint16_t TRAJECTORY_DATA[TRAJECTORY_FRAMES][TRAJECTORY_JOINTS] PROGMEM = {\n");
        for (var sample : trajectory.samples()) {
            var row = new StringJoiner(", ", "    { ", " },\n");
            for (int value : sample.centidegrees()) row.add(Integer.toString(value));
            code.append(row);
        }
        return code.append("};\n\n").toString();
    }

    static String python(ServoTrajectory trajectory) {
        for (var channel : trajectory.channels()) {
            if (channel.pin() > 27) {
                throw new IllegalArgumentException(channel.id() + " requires a Raspberry Pi BCM GPIO (0–27) or PCA9685 channel (0–15).");
            }
        }
        StringBuilder code = new StringBuilder("#!/usr/bin/env python3\n");
        describe(code, "#", trajectory);
        code.append("import time\n\nUSE_PCA9685 = False  # Set True when the configured numbers are PCA9685 channels.\n");
        code.append("TRAJECTORY_LOOP = ").append(trajectory.loop() ? "True" : "False").append('\n');
        code.append("MAX_LATENESS_US = ").append(ServoTrajectory.MAX_LATENESS_US).append("\n\n");
        code.append("SERVOS = (\n");
        for (var channel : trajectory.channels()) {
            code.append("    (").append(channel.pin()).append(", ").append(ServoTrajectory.MIN_PULSE_US)
                    .append(", ").append(ServoTrajectory.MAX_PULSE_US).append("),  # ").append(comment(channel.id())).append('\n');
        }
        code.append(")\n\nTRAJECTORY = (\n");
        for (var sample : trajectory.samples()) {
            var targets = new StringJoiner(", ", "(", ",)");
            for (int value : sample.centidegrees()) targets.add(Integer.toString(value));
            code.append("    (").append(sample.timeUs()).append(", ").append(targets).append("),\n");
        }
        return code.append("""
                )

                def setup_servos(outputs):
                    pins = [config[0] for config in SERVOS]
                    if len(set(pins)) != len(pins) or not all(0 <= pin < (16 if USE_PCA9685 else 28) for pin in pins):
                        raise ValueError("Invalid or duplicate GPIO pin / PCA9685 channel")
                    if not all(544 <= low < high <= 2400 for _, low, high in SERVOS):
                        raise ValueError("Invalid servo pulse range")
                    first = TRAJECTORY[0][1]
                    if USE_PCA9685:
                        from adafruit_servokit import ServoKit
                        kit = ServoKit(channels=16, frequency=50)
                        for (pin, low, high), angle in zip(SERVOS, first):
                            servo = kit.servo[pin]
                            outputs.append(servo)
                            servo.set_pulse_width_range(low, high)
                            servo.angle = angle / 100
                    else:
                        from gpiozero import AngularServo
                        for (pin, low, high), angle in zip(SERVOS, first):
                            outputs.append(AngularServo(pin, min_angle=0, max_angle=180, initial_angle=angle / 100,
                                                        min_pulse_width=low / 1_000_000, max_pulse_width=high / 1_000_000))

                def play_trajectory(outputs):
                    start = time.monotonic_ns()
                    for offset_us, angles in TRAJECTORY:
                        deadline = start + offset_us * 1000
                        now = time.monotonic_ns()
                        while now < deadline:
                            time.sleep((deadline - now) / 1_000_000_000)
                            now = time.monotonic_ns()
                        if now - deadline > MAX_LATENESS_US * 1000:
                            return False
                        for servo, angle in zip(outputs, angles):
                            servo.angle = angle / 100
                    return True

                def main():
                    outputs = []
                    try:
                        setup_servos(outputs)
                        time.sleep(1)
                        while True:
                            if not play_trajectory(outputs):
                                print("Playback stopped: timing overrun. Holding the last pose.", flush=True)
                                break
                            if not TRAJECTORY_LOOP:
                                print("Playback complete. Holding the final pose.", flush=True)
                                break
                        while True:
                            time.sleep(1)
                    except KeyboardInterrupt:
                        pass
                    finally:
                        for servo in outputs:
                            if USE_PCA9685:
                                servo.angle = None
                            else:
                                servo.close()

                if __name__ == "__main__":
                    main()
                """).toString();
    }

    private static void describe(StringBuilder code, String prefix, ServoTrajectory trajectory) {
        code.append(prefix).append(" RSim2 PWM servo trajectory: ").append(comment(trajectory.name())).append('\n');
        code.append(prefix).append(" Absolute times in us; targets in hundredths of a calibrated servo degree.\n");
        code.append(prefix).append(" Servo channel calibration: pin, minimum pulse (us), maximum pulse (us).\n");
        if (trajectory.hasSceneEvents()) {
            code.append(prefix).append(" Joint motion only. Scene object capture/release events are not hardware commands.\n");
        }
        code.append('\n');
    }

    private static String comment(String value) {
        return value.replaceAll("[\\p{Cntrl}\\p{Zl}\\p{Zp}\\\\]", " ");
    }
}
