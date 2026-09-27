// Independent continuous-profile checks; hardware pulse quantization is separate.
#include "../src/keyframe_math.h"
#include <cassert>
#include <cmath>
#include <climits>
#include <cstdio>
#include <initializer_list>

using namespace m09::keyframes;

double distanceAt(const AxisPlan &plan, double t, double distance) {
    const double speed = plan.speedMilliHz / 1000.0;
    const double ramp = speed / plan.acceleration;
    const double finish = distance / speed + ramp;
    if (t <= ramp) return 0.5 * plan.acceleration * t * t;
    if (t < finish - ramp) return speed * (t - 0.5 * ramp);
    if (t < finish) return distance - 0.5 * plan.acceleration * (finish - t) * (finish - t);
    return distance;
}

int main() {
    AxisPlan plans[3];
    const int distances[] = {160, 320, 80};
    for (unsigned i = 0; i < 3; ++i) {
        assert(planAxis(0, distances[i], 10000, 80, 250, plans[i]));
        const auto &p = plans[i];
        assert(p.moving && p.target == distances[i]);
        const double speed = p.speedMilliHz / 1000.0;
        const double independentlyCalculated = distances[i] / speed + speed / p.acceleration;
        assert(fabs(independentlyCalculated - 10) < 0.02);
        assert(fabs(independentlyCalculated - p.predictedSeconds) < 0.001);
        assert(speed <= 80 && p.acceleration > 0 && p.acceleration <= 250);
    }
    // Shared ramp/cruise timing means normalized intermediate progress agrees,
    // rather than three finite moves merely having the same end time.
    for (double time : {1.0, 2.0, 5.0, 8.0, 9.0}) {
        const double first = distanceAt(plans[0], time, distances[0]) / distances[0];
        for (unsigned i = 1; i < 3; ++i)
            assert(fabs(distanceAt(plans[i], time, distances[i]) / distances[i] - first) < 0.01);
    }
    AxisPlan p;
    assert(planAxis(120, -40, 10000, 80, 250, p));
    assert(p.target == -40 && p.speedMilliHz == plans[0].speedMilliHz);
    assert(planAxis(12, 12, 10000, 80, 250, p) && !p.moving);
    assert(!planAxis(0, 100000, 1000, 80, 250, p)); // Speed ceiling.
    assert(!planAxis(0, 1000, 1000, 2000, 1, p)); // Acceleration ceiling.
    assert(!planAxis(INT32_MIN, INT32_MAX, 10000, 80, 250, p));
    assert(!planAxis(0, 100, 0, 80, 250, p));
    assert(!planAxis(0, 100, 999, 80, 250, p));
    assert(!planAxis(0, 100, 60001, 80, 250, p));
    assert(!planAxis(0, 100, 10000, 0, 250, p));
    assert(!planAxis(0, 100, 10000, 80, 0, p));
    puts("PASS keyframe math: shared normalized progress, duration, signs, zero axes and infeasible limits");
}
