// Actual application path, with offline sensor/motor/UART fixtures only.
#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>

static unsigned checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) { std::fprintf(stderr, "FAIL BNO diagnostics line %d: %s\n", __LINE__, #x); std::exit(1); } } while (0)
void advance(uint32_t ms) { const uint32_t at = millis(); while (millis() - at < ms) loop(); }
bool near(double a, double b) { return fabs(a-b) < 0.001; }

int main() {
    simulated::physicalPitchUsesRoll = false;
    // Known rotations, independent of the simulated motor/quaternion generator.
    sh2_RotationVectorWAcc_t q = {0, 0, 0, 0, 0}; q.real = 1;
    EulerAngles e;
    CHECK(quaternionToEuler(q, e) && near(e.heading, 0) && near(e.pitch, 0) && near(e.roll, 0));
    q.real = q.i = sqrtf(0.5f);
    CHECK(quaternionToEuler(q, e) && near(e.roll, 90) && near(e.pitch, 0) && near(e.heading, 0));
    q.i = 0; q.j = 0.5f; q.real = sqrtf(0.75f);
    CHECK(quaternionToEuler(q, e) && near(e.pitch, 60) && near(e.roll, 0));
    q.j = 0; q.k = -sqrtf(0.5f); q.real = sqrtf(0.5f);
    CHECK(quaternionToEuler(q, e) && near(e.heading, 270));
    q.real *= -1; q.k *= -1;
    CHECK(quaternionToEuler(q, e) && near(e.heading, 270));
    q.real *= 2; q.k *= 2;
    CHECK(quaternionToEuler(q, e) && near(e.heading, 270));
    q = {0, 0, 0, 0, 0}; CHECK(!quaternionToEuler(q, e));
    q.real = INFINITY; CHECK(!quaternionToEuler(q, e));

    // Recorded powered JOG 0 1000 0 samples, independent of the simulated plant:
    // audit/milestones/M09_manual_sensor_worker_verified_2026-09-27/powered.log
    // lines 256 and 264, received at 12507 and 12998 ms. Negative generated
    // pitch steps increased BNO pitch by 8.59 deg while roll barely changed.
    const sh2_RotationVectorWAcc_t before = {
        0.871337891f, -0.0151977539f, -0.0911254883f, -0.481872559f, 1.44677734f};
    const sh2_RotationVectorWAcc_t after = {
        0.874023438f, 0.0210571289f, -0.0256347656f, -0.484741211f, 1.44775391f};
    EulerAngles beforeEuler, afterEuler;
    CHECK(quaternionToEuler(before, beforeEuler) && quaternionToEuler(after, afterEuler));
    CHECK(near(beforeEuler.pitch, -9.98875523) && near(afterEuler.pitch, -1.39795303));
    CHECK(near(beforeEuler.roll, 3.57087493) && near(afterEuler.roll, 3.53625894));
    CHECK(afterEuler.pitch - beforeEuler.pitch > 8.59);
    CHECK(fabs(afterEuler.roll - beforeEuler.roll) < 0.04);
    CHECK(fabs(shortestDifference(afterEuler.heading, beforeEuler.heading)) < 0.14);
    orientation = beforeEuler;
    CHECK(near(physicalPitch(), beforeEuler.pitch));
    orientation = afterEuler;
    CHECK(near(physicalPitch(), afterEuler.pitch));

    setup();
    sensorTelemetry();
    CHECK(std::string(txBuffer, txLength).find("has_sample=NO") != std::string::npos);
    CHECK(std::string(txBuffer, txLength).find("q_w=nan") != std::string::npos);
    simulated::accuracy = 0;
    advance(600);
    CHECK(bnoAccuracy == 0 && bnoDiagnostics.status == 0 && bnoDiagnostics.accepted);
    CHECK(bnoDiagnostics.statusCounts[0] > 0 && bnoDiagnostics.statusCounts[3] == 0);
    CHECK(bnoDiagnostics.eulerValid && near(bnoDiagnostics.euler.roll, orientation.roll));
    CHECK(near(bnoDiagnostics.rotation.accuracy, 0.125));
    CHECK(bnoDiagnostics.timestamp == static_cast<uint64_t>(bnoDiagnostics.rotationAt)*1000+7);
    CHECK(bnoDiagnostics.sequence == acceptedBnoSequence && bnoDiagnostics.timestamp == acceptedBnoTimestamp);
    simulated::accuracy = 3; advance(30);
    CHECK(bnoAccuracy == 3 && bnoDiagnostics.status == 3 && bnoDiagnostics.statusChanges == 1);
    CHECK(bnoDiagnostics.statusCounts[3] > 0);

    simulated::wrongReportType = true; simulated::accuracy = 0;
    const auto rotations = bnoDiagnostics.rotations;
    advance(30);
    CHECK(bnoDiagnostics.lastReport == 8 && bnoDiagnostics.lastStatus == 0 && bnoDiagnostics.otherReports > 0);
    CHECK(bnoDiagnostics.rotations == rotations && bnoDiagnostics.status == 3 && bnoAccuracy == 3);
    simulated::wrongReportType = false; simulated::invalidQuaternion = true;
    advance(30);
    CHECK(bnoDiagnostics.status == 0 && !bnoDiagnostics.eulerValid && !bnoDiagnostics.accepted);
    CHECK(std::string(bnoDiagnostics.reason) == "invalid_quaternion" && bnoAccuracy == 3);
    sensorTelemetry(); advance(30);
    CHECK(Serial.output.find("reason=invalid_quaternion") != std::string::npos);
    simulated::invalidQuaternion = false; advance(30);
    const auto timestamp = bnoDiagnostics.timestamp;
    const auto receivedAt = bnoDiagnostics.rotationAt;
    simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis()+1000;
    advance(600);
    CHECK(!fresh(millis()) && bnoDiagnostics.timestamp == timestamp && bnoDiagnostics.rotationAt == receivedAt);
    CHECK(millis() - receivedAt >= 600 && commandIdle() && manualReady());
    CHECK(Serial.output.find("BNO_RAW has_sample=YES report_id=0x05") != std::string::npos);
    CHECK(Serial.output.find("heading_accuracy_rad=0.125") != std::string::npos);
    CHECK(Serial.output.find("BNO_STATE available=YES fresh=NO") != std::string::npos);
    CHECK(simulated::commands.empty());
    CHECK(telemetryDrops == 0);
    txLength = sizeof(txBuffer); txOffset = 0; queueText("optional\n");
    CHECK(telemetryDrops == 1 && manualReady());
    std::printf("PASS BNO diagnostics: %u checks; raw/accepted/stale separation and unchanged Euler math\n", checks);
}
