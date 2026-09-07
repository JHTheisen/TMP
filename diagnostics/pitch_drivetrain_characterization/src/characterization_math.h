#ifndef MILESTONE4_CHARACTERIZATION_MATH_H
#define MILESTONE4_CHARACTERIZATION_MATH_H

#include <stdint.h>
#include <math.h>

namespace milestone4 {

// This unwrap is valid only when the caller guarantees less than half a motor
// revolution between accepted acquisitions. Acquisition gaps and commanded path
// distance must be checked by the caller; raw angles alone cannot detect aliasing.
struct AngleUnwrapper {
  bool initialized;
  uint16_t lastRaw;
  int32_t lastDelta;
  int64_t accumulatedTicks;
  uint64_t absoluteTravelTicks;

  AngleUnwrapper() : initialized(false), lastRaw(0), lastDelta(0),
      accumulatedTicks(0), absoluteTravelTicks(0) {}

  bool begin(uint16_t raw) {
    initialized = raw < 4096;
    lastRaw = initialized ? raw : 0;
    lastDelta = 0;
    accumulatedTicks = 0;
    absoluteTravelTicks = 0;
    return initialized;
  }

  bool update(uint16_t raw) {
    if (!initialized || raw >= 4096) return false;
    int32_t delta = static_cast<int32_t>(raw) - lastRaw;
    // Exactly 180 degrees has no identifiable direction.
    if (delta == 2048 || delta == -2048) return false;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    accumulatedTicks += delta;
    absoluteTravelTicks += static_cast<uint32_t>(delta < 0 ? -delta : delta);
    lastRaw = raw;
    lastDelta = delta;
    return true;
  }

  double accumulatedDegrees() const { return static_cast<double>(accumulatedTicks) * (360.0 / 4096.0); }
  double travelDegrees() const { return static_cast<double>(absoluteTravelTicks) * (360.0 / 4096.0); }
};

inline bool finiteNumber(double value) { return isfinite(value) != 0; }
inline double larger(double a, double b) { return a > b ? a : b; }

inline int directionSign(double value, double epsilon = 0.0) {
  if (!finiteNumber(value) || !finiteNumber(epsilon) || epsilon < 0.0) return 0;
  if (fabs(value) <= epsilon) return 0;
  return value > 0.0 ? 1 : -1;
}

struct Endpoint {
  double motorDeg;
  double pitchMean;
  double pitchStddev;
  double pitchRange;
  double pitchDrift;
  unsigned count;
  bool valid;

  Endpoint() : motorDeg(0), pitchMean(0), pitchStddev(0), pitchRange(0),
      pitchDrift(0), count(0), valid(false) {}
};

inline bool validEndpoint(const Endpoint& endpoint) {
  return endpoint.valid && endpoint.count >= 2 && finiteNumber(endpoint.motorDeg) &&
      finiteNumber(endpoint.pitchMean) && finiteNumber(endpoint.pitchStddev) &&
      finiteNumber(endpoint.pitchRange) && finiteNumber(endpoint.pitchDrift) &&
      endpoint.pitchStddev >= 0.0 && endpoint.pitchRange >= 0.0;
}

inline double endpointNoiseGate(const Endpoint& a, const Endpoint& b, double floorDeg) {
  if (!validEndpoint(a) || !validEndpoint(b)) return INFINITY;
  // Conservative observed-noise bound, not a statistical confidence interval.
  double gate = larger(floorDeg, 5.0 * hypot(a.pitchStddev, b.pitchStddev));
  gate = larger(gate, 2.0 * (a.pitchRange + b.pitchRange));
  gate = larger(gate, 2.0 * (fabs(a.pitchDrift) + fabs(b.pitchDrift)));
  return gate;
}

inline double ratioNoiseGate(const Endpoint& a, const Endpoint& b) {
  return endpointNoiseGate(a, b, 1.0);
}

struct RatioResult {
  bool valid;
  double magnitude;
  double pitchDelta;
  double motorDelta;
  double noiseGateDeg;
  // Sensitivity bounds derived from +/- noiseGateDeg on pitch displacement;
  // these are neither calibration guarantees nor statistical confidence bounds.
  double lower;
  double upper;
  int direction;

  RatioResult() : valid(false), magnitude(0), pitchDelta(0), motorDelta(0),
      noiseGateDeg(0), lower(0), upper(0), direction(0) {}
};

inline RatioResult calculateRatio(const Endpoint& a, const Endpoint& b) {
  RatioResult result;
  if (!validEndpoint(a) || !validEndpoint(b)) return result;
  result.motorDelta = b.motorDeg - a.motorDeg;
  result.pitchDelta = b.pitchMean - a.pitchMean;
  result.noiseGateDeg = ratioNoiseGate(a, b);
  if (!finiteNumber(result.motorDelta) || !finiteNumber(result.pitchDelta) ||
      !finiteNumber(result.noiseGateDeg)) return result;
  const double motor = fabs(result.motorDelta);
  const double pitch = fabs(result.pitchDelta);
  if (motor < 1.0 || pitch <= result.noiseGateDeg) return result;
  result.magnitude = motor / pitch;
  result.lower = motor / (pitch + result.noiseGateDeg);
  result.upper = motor / (pitch - result.noiseGateDeg);
  result.direction = directionSign(result.motorDelta) * directionSign(result.pitchDelta);
  result.valid = finiteNumber(result.magnitude) && finiteNumber(result.lower) &&
      finiteNumber(result.upper) && result.magnitude > 0.0;
  return result;
}

struct ReversalResult {
  bool confirmed;
  bool inconclusive;
  bool invalidInput;
  bool wrongMotorDirection;
  bool wrongPitchDirection;
  double lowerMotorDeg;
  double upperMotorDeg;
  double pitchDelta;
  double thresholdDeg;
  unsigned observations;
  unsigned consecutiveDetections;

  ReversalResult() : confirmed(false), inconclusive(true), invalidInput(false),
      wrongMotorDirection(false), wrongPitchDirection(false), lowerMotorDeg(0),
      upperMotorDeg(0), pitchDelta(0), thresholdDeg(0), observations(0),
      consecutiveDetections(0) {}
};

// Reports an observed response-threshold crossing after reversal. The interval
// includes sensor resolution, filtering, compliance and motion; it is NOT a
// measurement of pure mechanical backlash. BNO continuity is a caller precondition.
class ReversalTracker {
 public:
  ReversalTracker() : motorSign_(0), pitchSign_(0), previousTravel_(0),
      previousNonDetected_(0), started_(false) {}

  bool begin(const Endpoint& baseline, int expectedMotorSign, int expectedPitchSign) {
    baseline_ = baseline;
    motorSign_ = expectedMotorSign;
    pitchSign_ = expectedPitchSign;
    previousTravel_ = 0.0;
    previousNonDetected_ = 0.0;
    result_ = ReversalResult();
    started_ = validEndpoint(baseline) && (motorSign_ == 1 || motorSign_ == -1) &&
        (pitchSign_ == 1 || pitchSign_ == -1);
    result_.invalidInput = !started_;
    return started_;
  }

  bool observe(const Endpoint& endpoint) {
    if (!started_ || result_.invalidInput || result_.wrongMotorDirection ||
        result_.wrongPitchDirection) return false;
    // Once two observations have established the first crossing, preserve it.
    if (result_.confirmed) return true;
    ++result_.observations;
    if (!validEndpoint(endpoint)) return invalidateInput();
    const double motorTravel = (endpoint.motorDeg - baseline_.motorDeg) * motorSign_;
    const double pitchTravel = (endpoint.pitchMean - baseline_.pitchMean) * pitchSign_;
    const double gate = endpointNoiseGate(baseline_, endpoint, 0.25);
    if (!finiteNumber(motorTravel) || !finiteNumber(pitchTravel) || !finiteNumber(gate))
      return invalidateInput();
    if (motorTravel <= 0.0 || motorTravel <= previousTravel_) {
      result_.wrongMotorDirection = true;
      return false;
    }
    previousTravel_ = motorTravel;
    if (pitchTravel < -gate) {
      result_.wrongPitchDirection = true;
      return false;
    }
    if (pitchTravel > gate) {
      if (result_.consecutiveDetections == 0) {
        result_.lowerMotorDeg = previousNonDetected_;
        result_.upperMotorDeg = motorTravel;
        result_.pitchDelta = endpoint.pitchMean - baseline_.pitchMean;
        result_.thresholdDeg = gate;
      }
      ++result_.consecutiveDetections;
      if (result_.consecutiveDetections >= 2) {
        result_.confirmed = true;
        result_.inconclusive = false;
      }
    } else {
      result_.consecutiveDetections = 0;
      previousNonDetected_ = motorTravel;
      result_.lowerMotorDeg = 0.0;
      result_.upperMotorDeg = 0.0;
      result_.pitchDelta = 0.0;
      result_.thresholdDeg = gate;
    }
    return result_.confirmed;
  }

  const ReversalResult& result() const { return result_; }

 private:
  bool invalidateInput() {
    result_.invalidInput = true;
    result_.confirmed = false;
    result_.inconclusive = true;
    return false;
  }
  Endpoint baseline_;
  int motorSign_;
  int pitchSign_;
  double previousTravel_;
  double previousNonDetected_;
  bool started_;
  ReversalResult result_;
};

}  // namespace milestone4

#endif
