#include "../src/main.cpp"
#include <cassert>
#include <cstdio>
#include <string>

namespace {
void tick(uint32_t now) { motionWatchdog.check(now); commandWatchdog.check(now); }
void advance(uint32_t duration) { const auto start=millis(); while (millis()-start<duration) loop(); }
void command(const std::string &s) { simulated::serialInput += s + "\n"; advance(10); }
void stream(int pitch, uint32_t duration) {
    const auto start=millis();
    while (millis()-start<duration) { command("JOG 0 " + std::to_string(pitch) + " 0"); advance(10); }
}
size_t count(const std::string &text, const std::string &needle) {
    size_t n=0, at=0;
    while ((at=text.find(needle, at))!=std::string::npos) { ++n; at+=needle.size(); }
    return n;
}
bool recordHas(const std::string &key, const std::string &field) {
    size_t at=0;
    while ((at=Serial.output.find("PITCH_DIR ",at))!=std::string::npos) {
        const auto line=Serial.output.substr(at,Serial.output.find('\n',at)-at);
        if (line.find(key)!=std::string::npos && line.find(field)!=std::string::npos) return true;
        ++at;
    }
    return false;
}
}
int main() {
    simulated::independentTick=tick;
    setup(); advance(100); command("JOG 0 0 0");
    assert(manualActive);
    // Deliberately inconsistent latch: evidence must be read, not inferred from
    // the expected direction, and must never correct/interlock motor control.
    simulated::gpioOutput=1U<<26;
    stream(250,500);
    assert(recordHas("event=RUN cmd=250 step_dir=-1 call=runBackward rc=0", "dir26_out=1"));
    assert(recordHas("event=DIR_CHECK", "dir26_out=1"));
    assert(pitchMotor->getCurrentSpeedInMilliHz()<0 && manualPitch.rate==PITCH_TRAVEL_SPEED_HZ/4);
    const auto before=count(Serial.output,"PITCH_DIR ");
    stream(300,1000); // Held sign and changing magnitude emit no packet chatter.
    assert(count(Serial.output,"PITCH_DIR ")==before);

    simulated::gpioOutput=0;
    stream(-250,1000);
    assert(recordHas("event=BRAKE_REVERSE cmd=-250 step_dir=1", "active_sign=1 braking=1"));
    assert(recordHas("event=STOPPED", "observing=1"));
    assert(recordHas("event=RUN cmd=-250 step_dir=1 call=runForward rc=0", "dir26_out=0"));
    assert(recordHas("event=DIR_CHECK cmd=-250", "dir26_out=0"));
    assert(pitchMotor->getCurrentSpeedInMilliHz()>0 && manualPitch.direction==-1);
    assert(manualActive && !commandWatchdog.tripped());
    command("STOP"); advance(800);
    assert(recordHas("event=BRAKE_ZERO cmd=0 step_dir=0", "ending=1"));
    assert(manualMotorsStopped() && !manualActive);

    // Failed call reports the real return code and retains existing axis-local
    // failure behavior. It must not fabricate a successful delayed DIR check.
    command("JOG 0 0 0");
    simulated::moveRejected=true; simulated::rejectedStepPin=12;
    command("JOG 0 250 0"); command("STOP"); advance(600);
    assert(recordHas("event=RUN cmd=250 step_dir=-1 call=runBackward rc=-1", "dir26_out=0"));
    assert(!pitchDirectionDiagnostics.checkPending && manualMotorsStopped());

    // Saturated serial and transition chatter remain bounded without waiting,
    // moving a motor, changing a request, or touching the watchdog.
    simulated::serialWriteSpace=0;
    pitchDirectionDiagnostics.windowAt=millis(); pitchDirectionDiagnostics.emitted=0;
    pitchDirectionDiagnostics.suppressed=0;
    const auto now=millis();
    const auto moves=simulated::commands.size();
    const int request=manualPitch.request;
    for (unsigned n=0;n<20;++n) tracePitchDirection("TEST");
    assert(millis()==now && pitchDirectionDiagnostics.emitted==8 && pitchDirectionDiagnostics.suppressed==12);
    assert(simulated::commands.size()==moves && manualPitch.request==request && simulated::gpioOutput==0);
    simulated::serialWriteSpace=128; advance(1100);
    tracePitchDirection("TEST"); advance(50);
    assert(recordHas("event=TEST", "suppressed=12"));
    assert(Serial.output.find("PITCH_DIR at_ms=")!=std::string::npos);
    puts("PASS pitch direction diagnostics: signs/calls/rc, independent latch, brake/reversal/STOP, delayed check, bounded nonblocking output");
}
