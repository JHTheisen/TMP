// Diagnostic wrapper ONLY. Build separately via PLATFORMIO_SRC_DIR/BUILD_DIR.
// Both stock source trees remain unchanged. No motion/control commands are run.
#define setup stockSetup
#define loop stockLoop
#ifdef BNO_OBSERVER_M08
#include "../../../M08_go_to_pose/src/main.cpp"
#else
#include "../../src/main.cpp"
#endif
#undef setup
#undef loop

void setup() {
    stockSetup(); // Identical pins, timers, BNO init, reset and report configuration.
    queueText("BNO OBSERVER: sensor acquisition only; automatic motor control omitted; X aborts.\n");
}

void loop() {
    // Never dispatch POSE/MOVE/JOG: the observation test cannot command movement.
    for (unsigned n = 0; n < 32 && Serial.available(); ++n) {
        const int input = Serial.read();
        if (input == 'X' || input == 'x') abortTest("Operator X abort in sensor observer");
    }
    if (!finalPrinted) {
        serviceBno();
        if (millis() - lastDisplay >= 750) { lastDisplay = millis(); telemetry(); }
    }
    serviceSerialOutput();
    delay(1);
}
