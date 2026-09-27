// The real worker enters a blocking fixture read. Its wait yields simulated
// CPU time to the real foreground loop and independent host/watchdog ticks.
// No cached UART bytes or synthetic sensor events refresh the command lease.
#include "../src/main.cpp"
#include <cassert>
#include <cstdio>
#include <string>

namespace {
std::string mode;
uint32_t nextHost=0, lastHost=0, switchAt=0, handledAt=0;
bool feed=false, switched=false, tracking=false, lostManual=false;
int requestedYaw=100, requestedPitch=100, requestedCarriage=100;
void tick(uint32_t now) {
    motionWatchdog.check(now); commandWatchdog.check(now);
    if (tracking && !switched && now >= switchAt) {
        switched=true;
        Wire1.encoder.requestDelayMs=0; // one injected encoder stall only
        if (mode=="host_loss" || mode=="buffered_noise") feed=false;
        else if (mode=="stop") { feed=false; simulated::serialInput += "STOP\n"; }
        else if (mode=="center") requestedYaw=requestedPitch=requestedCarriage=0;
        else { requestedYaw=400; requestedPitch=500; requestedCarriage=600; }
    }
    if (tracking && switched && mode=="buffered_noise" && now%20==0)
        simulated::serialInput += "not_a_JOG\n"; // UART activity cannot renew the lease.
    if (feed && now >= nextHost) {
        simulated::serialInput += "JOG " + std::to_string(requestedYaw) + " " +
            std::to_string(requestedPitch) + " " + std::to_string(requestedCarriage) + "\n";
        nextHost=now+20; lastHost=now;
    }
    if (tracking && (commandWatchdog.tripped() || !manualActive)) lostManual=true;
    if (tracking && switched && !handledAt && mode!="host_loss" && mode!="buffered_noise") {
        const bool handled = mode=="stop" ? manualEnding :
            manualYaw.request==requestedYaw && manualPitch.request==requestedPitch &&
            manualCarriage.request==requestedCarriage;
        if (handled) handledAt=now;
    }
}
void advance(uint32_t duration) { const auto start=millis(); while (millis()-start<duration) loop(); }
}
int main(int argc, char **argv) {
    assert(argc==2); mode=argv[1];
    simulated::independentTick=tick;
    Wire.encoder.present=Wire1.encoder.present=true;
    setup(); advance(50);
    simulated::serialInput="JOG 0 0 0\n"; advance(20);
    assert(manualActive);
    feed=true; nextHost=millis(); advance(600);
    assert(yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
    const uint32_t begin=millis();
    tracking=true; switchAt=begin+100;
    simulated::forceStops=0; simulated::firstForceStopAt=0;
    if (mode=="encoder") Wire1.encoder.requestDelayMs=1000;
    else simulated::blockBnoMs=1000;
    advance(1100);
    assert(switched);
    if (mode=="encoder") assert(bnoTrace.encoderMaxUs>=1000000);
    else assert(bnoTrace.acquireMaxUs>=1000000);
    if (mode=="host_loss" || mode=="buffered_noise") {
        assert(!manualActive && commandIdle() && manualMotorsStopped());
        assert(simulated::firstForceStopAt>=lastHost+250 && simulated::firstForceStopAt<=lastHost+260);
        assert(Serial.output.find("command stream lost for 250 ms")!=std::string::npos);
        printf("PASS sensor-stall %s: all three stop request after %lu ms; stall >=1000 ms\n",
            mode.c_str(), (unsigned long)(simulated::firstForceStopAt-lastHost));
    } else {
        assert(handledAt && handledAt-switchAt<=20);
        assert(simulated::forceStops==0);
        assert(Serial.output.find("command stream lost for 250 ms")==std::string::npos);
        if (mode=="stop") assert(!manualActive && commandIdle() && manualMotorsStopped());
        else if (mode=="center") assert(!lostManual && manualActive && manualMotorsStopped());
        else {
            assert(!lostManual && manualActive);
            assert(yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
            assert(manualYaw.rate==YAW_SLEW_SPEED_HZ*4/10 && manualPitch.rate==PITCH_SLEW_SPEED_HZ/2);
            assert(manualCarriage.rate==CARRIAGE_MAX_SPEED_HZ*6/10);
        }
        printf("PASS sensor-stall %s: command handled in %lu ms during >=1000 ms I/O stall; no lease trip\n",
            mode.c_str(), (unsigned long)(handledAt-switchAt));
    }
    assert(bnoResets==0 && sensorSampleDrops==0); // no invented reset/sample loss
}
