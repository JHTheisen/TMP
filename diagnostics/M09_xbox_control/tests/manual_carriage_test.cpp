#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>
namespace {
void require(bool ok, const char *expr, int line) {
    if (!ok) { std::fprintf(stderr,"FAIL carriage line %d: %s\n%s",line,expr,Serial.output.c_str()); std::exit(1); }
}
#define CHECK(value) require((value), #value, __LINE__)
void tick(uint32_t now) { commandWatchdog.check(now); motionWatchdog.check(now); }
void advance(uint32_t duration) { const uint32_t at=millis(); while(millis()-at<duration) loop(); }
void line(const std::string &s) { simulated::serialInput+=s+"\n"; advance(10); }
void stream(int carriage, uint32_t duration, int yaw=0, int pitch=0) {
    const uint32_t at=millis(); while(millis()-at<duration) {
        line("JOG "+std::to_string(yaw)+" "+std::to_string(pitch)+" "+std::to_string(carriage)); advance(10);
    }
}
}
int main(int argc,char **argv) {
    if(argc!=2) return 1;
    const std::string scenario=argv[1];
    simulated::physicalPitchUsesRoll=false;
    simulated::independentTick=tick;
    if(scenario=="sensor_independent") simulated::busBInitFails=true;
    setup(); advance(50); CHECK(commandIdle());
    line("JOG 0 0 0"); CHECK(manualActive);
    if(scenario=="protocol") {
        line("JOG 0 0 0.5"); line("JOG 0 0 1001"); line("JOG 0 0 nan");
        CHECK(simulated::commands.empty());
        stream(500,500); CHECK(carriageMotor->isRunning());
        line("JOG 0 0"); advance(1500); CHECK(!carriageMotor->isRunning() && manualActive);
    } else if(scenario=="axis_rejected") {
        simulated::moveRejected=true; simulated::rejectedStepPin=22;
        stream(200,400,200,200);
        CHECK(!carriageMotor->isRunning() && yawMotor->isRunning() && pitchMotor->isRunning());
        simulated::moveRejected=false; stream(200,400,200,200); CHECK(carriageMotor->isRunning());
    } else {
        stream(1000,2200);
        CHECK(carriageMotor->getCurrentPosition()>1500 && manualActive);
        CHECK(!yawMotor->isRunning() && !pitchMotor->isRunning());
        if(scenario=="rates") {
            const auto count=simulated::commands.size(); stream(100,1500);
            CHECK(count==simulated::commands.size() && manualCarriage.rate==CARRIAGE_MAX_SPEED_HZ/10);
            CHECK(simulated::motors[2].acceleration==CARRIAGE_ACCELERATION);
        } else if(scenario=="reverse") {
            stream(-1000,2200); CHECK(manualCarriage.direction==-1 && carriageMotor->getCurrentSpeedInMilliHz()<0);
            for(const auto &command:simulated::commands) CHECK(!command.wasBraking && command.continuous);
        } else if(scenario=="host_loss") {
            advance(400); CHECK(commandIdle() && manualReady() && !carriageMotor->isRunning());
            const auto count=simulated::commands.size(); line("JOG 0 0 500"); CHECK(count==simulated::commands.size());
            line("JOG 0 0 0"); CHECK(manualActive);
        } else if(scenario=="braking_timeout") {
            simulated::motors[2].neverStops=true; stream(0,4200,100,100);
            CHECK(!carriageMotor->isRunning() && yawMotor->isRunning() && pitchMotor->isRunning());
            CHECK(Serial.output.find("CARRIAGE braking timeout")!=std::string::npos);
            simulated::motors[2].neverStops=false;
        } else if(scenario!="axes" && scenario!="unbounded" && scenario!="sensor_independent") return 2;
    }
    line("STOP"); advance(2200); CHECK(commandIdle() && manualReady() && manualMotorsStopped());
    std::printf("PASS carriage: %s\n",scenario.c_str());
}
