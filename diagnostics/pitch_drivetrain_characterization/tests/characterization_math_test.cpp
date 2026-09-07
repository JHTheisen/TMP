// Native synthetic checks only. These exercise arithmetic, not physical hardware.
#include "../src/characterization_math.h"
#include <stdio.h>
#include <stdlib.h>
#include <float.h>

using namespace milestone4;

static unsigned checks = 0;

static void check(bool condition, const char* description) {
  ++checks;
  if (!condition) {
    fprintf(stderr, "FAIL: %s\n", description);
    exit(1);
  }
}

static bool closeTo(double a, double b, double epsilon = 1e-9) {
  return fabs(a - b) <= epsilon;
}

static Endpoint point(double motor, double pitch, double sd = 0.01,
                      double range = 0.02, double drift = 0.01) {
  Endpoint e;
  e.motorDeg = motor;
  e.pitchMean = pitch;
  e.pitchStddev = sd;
  e.pitchRange = range;
  e.pitchDrift = drift;
  e.count = 40;
  e.valid = true;
  return e;
}

static void unwrapChecks() {
  AngleUnwrapper unwrap;
  check(!unwrap.update(10), "uninitialized unwrap rejects update");
  check(!unwrap.begin(4096), "invalid initial raw code rejected");
  check(!unwrap.initialized, "invalid initialization remains uninitialized");
  check(unwrap.begin(4090), "valid initial raw code accepted");
  check(unwrap.update(6), "positive wrap accepted");
  check(unwrap.lastDelta == 12, "positive wrap nearest delta");
  check(closeTo(unwrap.accumulatedDegrees(), 12.0 * 360.0 / 4096.0), "positive wrap scale");
  check(unwrap.update(4090), "negative wrap accepted");
  check(unwrap.lastDelta == -12, "negative wrap nearest delta");
  check(unwrap.accumulatedTicks == 0, "reverse returns relative angle to zero");
  check(unwrap.absoluteTravelTicks == 24, "absolute travel preserves both directions");
  check(!unwrap.update(2042), "ambiguous negative half turn rejected");
  check(unwrap.lastRaw == 4090 && unwrap.accumulatedTicks == 0 &&
        unwrap.absoluteTravelTicks == 24, "ambiguous update leaves accepted state unchanged");
  check(!unwrap.update(5000), "invalid raw update rejected");
  check(unwrap.lastRaw == 4090, "invalid raw update preserves last accepted raw");
  unwrap.begin(0);
  check(!unwrap.update(2048), "ambiguous positive half turn rejected");
  check(unwrap.update(2047), "just below half turn accepted");
  check(unwrap.lastDelta == 2047, "just below half turn direction preserved");
  unwrap.begin(0);
  check(unwrap.update(2049), "nearest negative delta accepted");
  check(unwrap.lastDelta == -2047, "just over half turn uses nearest reverse direction");
  unwrap.begin(123);
  uint16_t raw = 123;
  // Sixty-four forward revolutions, then the exact reverse path. Deltas stay
  // well below the caller's half-turn acquisition bound throughout this test.
  for (unsigned i = 0; i < 512; ++i) {
    raw = static_cast<uint16_t>((raw + 512) % 4096);
    check(unwrap.update(raw), "synthetic multirevolution forward update");
  }
  check(unwrap.accumulatedTicks == 64 * 4096, "multiple turns retained beyond 16-bit count");
  check(closeTo(unwrap.accumulatedDegrees(), 64.0 * 360.0), "multirevolution degree scale");
  for (unsigned i = 0; i < 512; ++i) {
    raw = static_cast<uint16_t>((raw + 4096 - 512) % 4096);
    check(unwrap.update(raw), "synthetic multirevolution reverse update");
  }
  check(unwrap.accumulatedTicks == 0, "multirevolution reversal returns net position");
  check(unwrap.absoluteTravelTicks == 128 * 4096, "multirevolution path preserves total travel");
  check(closeTo(unwrap.travelDegrees(), 128.0 * 360.0), "total travel degree scale");
  check(unwrap.begin(100), "reinitialization accepted");
  check(unwrap.accumulatedTicks == 0 && unwrap.absoluteTravelTicks == 0 &&
        unwrap.lastDelta == 0, "reinitialization clears all accumulated state");
}

static void ratioChecks() {
  const Endpoint start = point(0, 3);
  RatioResult result = calculateRatio(start, point(112.5, 10.5));
  check(result.valid, "clear synthetic displacement gives ratio");
  check(closeTo(result.magnitude, 15), "ratio computed from endpoint displacement");
  check(result.direction == 1, "same mounting direction reported");
  check(closeTo(result.noiseGateDeg, 1), "ratio floor is one degree");
  check(closeTo(result.lower, 112.5 / 8.5) && closeTo(result.upper, 112.5 / 6.5),
        "sensitivity bounds use observed noise gate");
  result = calculateRatio(start, point(112.5, -4.5));
  check(result.valid && closeTo(result.magnitude, 15) && result.direction == -1,
        "opposite sensor mounting retains positive ratio and negative direction");
  result = calculateRatio(point(112.5, 10.5), start);
  check(result.valid && closeTo(result.magnitude, 15) && result.direction == 1,
        "reverse displacement ratio and direction");
  result = calculateRatio(start, point(-112.5, 10.5));
  check(result.valid && result.direction == -1, "reverse motor opposite pitch sign");
  result = calculateRatio(start, point(112.5, 8));
  check(result.valid && closeTo(result.magnitude, 22.5), "no assumed reduction ratio imposed");
  check(!calculateRatio(start, point(112.5, 3)).valid, "zero pitch displacement inconclusive");
  check(!calculateRatio(start, point(112.5, 4)).valid, "pitch equal to gate inconclusive");
  check(!calculateRatio(start, point(0.999, 10.5)).valid, "tiny motor displacement rejected");
  check(calculateRatio(start, point(1.0, 10.5)).valid, "one degree motor displacement accepted");
  check(!calculateRatio(start, point(112.5, 4.1, 0.5)).valid, "stddev noise rejects weak signal");
  check(!calculateRatio(start, point(112.5, 4.1, 0.01, 0.6)).valid,
        "observed range noise rejects weak signal");
  check(!calculateRatio(start, point(112.5, 4.1, 0.01, 0.02, -0.6)).valid,
        "drift magnitude rejects weak signal regardless of drift sign");
  check(closeTo(ratioNoiseGate(point(0, 0, 0.3, 0, 0), point(10, 5, 0.4, 0, 0)), 2.5),
        "combined standard deviation gate");
  check(closeTo(ratioNoiseGate(point(0, 0, 0, 0.4, 0), point(10, 5, 0, 0.6, 0)), 2),
        "range sum gate");
  check(closeTo(ratioNoiseGate(point(0, 0, 0, 0, -0.4), point(10, 5, 0, 0, 0.6)), 2),
        "absolute drift sum gate");
  Endpoint invalid = start;
  invalid.valid = false;
  check(!calculateRatio(invalid, point(100, 20)).valid, "explicit invalid endpoint rejected");
  invalid = start;
  invalid.count = 1;
  check(!calculateRatio(invalid, point(100, 20)).valid, "insufficient samples rejected");
  invalid = start;
  invalid.pitchStddev = -0.1;
  check(!calculateRatio(invalid, point(100, 20)).valid, "negative standard deviation rejected");
  invalid = start;
  invalid.pitchRange = -0.1;
  check(!calculateRatio(invalid, point(100, 20)).valid, "negative range rejected");
  check(!calculateRatio(start, point(INFINITY, 20)).valid, "infinite motor endpoint rejected");
  check(!calculateRatio(start, point(100, NAN)).valid, "NaN pitch endpoint rejected");
  check(!calculateRatio(point(-DBL_MAX, 0), point(DBL_MAX, 10)).valid,
        "displacement subtraction overflow rejected");
  check(!calculateRatio(start, point(100, 20, DBL_MAX)).valid,
        "noise gate overflow rejected");
  check(directionSign(-1, 0.5) == -1 && directionSign(1, 0.5) == 1,
        "direction signs above supplied tolerance");
  check(directionSign(0.5, 0.5) == 0 && directionSign(-0.5, 0.5) == 0,
        "direction at supplied tolerance unresolved");
  check(directionSign(NAN) == 0 && directionSign(1, -1) == 0 &&
        directionSign(1, INFINITY) == 0, "invalid direction inputs unresolved");
}

static void reversalChecks() {
  ReversalTracker reversal;
  check(!reversal.observe(point(1, 1)), "reversal observation before begin rejected");
  check(reversal.begin(point(20, 5), -1, -1), "valid reverse direction baseline");
  check(!reversal.observe(point(18, 4.98)), "noise-level first increment not detected");
  check(!reversal.observe(point(16, 4.90)), "subthreshold second increment not detected");
  check(!reversal.observe(point(14, 4.60)), "one suprathreshold endpoint not confirmed");
  check(reversal.result().inconclusive && !reversal.result().confirmed,
        "single candidate remains inconclusive");
  check(reversal.observe(point(12, 4.40)), "second consecutive endpoint confirms direction");
  check(reversal.result().confirmed && !reversal.result().inconclusive,
        "confirmed reversal response status");
  check(closeTo(reversal.result().lowerMotorDeg, 4) &&
        closeTo(reversal.result().upperMotorDeg, 6),
        "bracket ends at first candidate not confirmation endpoint");
  check(closeTo(reversal.result().pitchDelta, -0.4) &&
        closeTo(reversal.result().thresholdDeg, 0.25),
        "candidate signed pitch delta and floor threshold retained");
  check(reversal.observe(point(-100, -10)) && closeTo(reversal.result().upperMotorDeg, 6),
        "later large confirmation move does not widen first crossing interval");

  reversal.begin(point(0, 0), 1, -1);
  reversal.observe(point(2, -0.1));
  reversal.observe(point(4, -0.5));
  check(reversal.observe(point(60, -4)), "large following segment may confirm a candidate");
  check(closeTo(reversal.result().lowerMotorDeg, 2) &&
        closeTo(reversal.result().upperMotorDeg, 4), "large confirmation preserves small-step bracket");

  reversal.begin(point(0, 0), 1, 1);
  reversal.observe(point(2, 0.4));
  reversal.observe(point(4, 0.1));
  check(reversal.result().consecutiveDetections == 0, "isolated excursion loses candidacy");
  reversal.observe(point(6, 0.5));
  check(reversal.observe(point(8, 0.7)), "later consecutive response confirms");
  check(closeTo(reversal.result().lowerMotorDeg, 4) &&
        closeTo(reversal.result().upperMotorDeg, 6), "noise spike does not retain obsolete bracket");

  reversal.begin(point(0, 0), 1, 1);
  for (unsigned i = 1; i <= 10; ++i) reversal.observe(point(i * 2, i % 2 ? 0.1 : -0.1));
  check(!reversal.result().confirmed && reversal.result().inconclusive,
        "noisy subthreshold sequence remains inconclusive");
  check(!reversal.result().wrongPitchDirection, "subthreshold opposite jitter is tolerated");

  reversal.begin(point(0, 0), 1, 1);
  reversal.observe(point(2, 0.5, 0.2, 0.5, 0.3));
  reversal.observe(point(4, 0.9, 0.2, 0.5, 0.3));
  check(!reversal.result().confirmed, "noise-scaled gate avoids false response detection");

  reversal.begin(point(0, 0), 1, 1);
  check(!reversal.observe(point(-2, 0.5)) && reversal.result().wrongMotorDirection,
        "opposite motor direction makes reversal inconclusive");
  check(!reversal.observe(point(4, 1)), "wrong-direction sequence cannot later recover silently");
  reversal.begin(point(0, 0), 1, 1);
  reversal.observe(point(2, 0.1));
  check(!reversal.observe(point(2, 0.5)) && reversal.result().wrongMotorDirection,
        "unchanged motor endpoint does not count as another reversal increment");
  reversal.begin(point(0, 0), 1, 1);
  check(!reversal.observe(point(2, -0.5)) && reversal.result().wrongPitchDirection,
        "resolved pitch motion opposite expected direction rejected");
  reversal.begin(point(0, 0), 1, 1);
  Endpoint invalid = point(2, 1);
  invalid.valid = false;
  check(!reversal.observe(invalid) && reversal.result().invalidInput,
        "invalid settled endpoint makes result inconclusive");
  check(!reversal.begin(point(0, 0), 0, 1) && reversal.result().invalidInput,
        "unknown motor direction cannot establish reversal test");
  check(!reversal.begin(point(0, 0), 1, 0), "unknown pitch direction cannot establish reversal test");
  check(!reversal.begin(invalid, 1, 1), "invalid baseline cannot establish reversal test");
  check(reversal.begin(point(0, 0), 1, 1), "new baseline explicitly resets invalid prior result");
  check(!reversal.result().invalidInput && reversal.result().observations == 0,
        "new baseline clears accumulated reversal status");
}

int main() {
  unwrapChecks();
  ratioChecks();
  reversalChecks();
  printf("PASS: %u synthetic math checks (no physical validation claimed)\n", checks);
  return 0;
}
