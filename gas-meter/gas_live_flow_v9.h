// Live gas flow from the magnetometer waveform, with self-adapting shape.
//
// A diaphragm meter is positive displacement: the magnetic field depends only on
// how far the mechanism has turned, not on how fast. One cycle of the field
// (the SHAPE) therefore maps the field back to the meter's position within its
// cycle at any flow rate, and flow is the slope of that position over time.
//
// The field is modelled as  z = offset + amplitude * shape(phase), with shape
// scaled to -1..+1. Three things adapt:
//   1. offset and amplitude, from every peak and trough measured during slow
//      cycles, which tracks temperature drift and gain;
//   2. the shape itself, re-learned from each full cycle of steady flow (the
//      water heater pilot provides one every ~11 min), timed purely from the
//      field's own crossings so a wrong shape cannot block its own correction;
//   3. both are saved to flash (a new shape right away, offset/amplitude at
//      most every 30 min), so a restart resumes with what was learned.
// At faster flows the reading is taken over exactly the last full cycle once it
// agrees with the windowed fit, which cancels the shape's small repeating
// errors. Where the field is flat (around each turn) it cannot tell position;
// it then gives the fit "somewhere in this stretch" positions instead.
// The seed shape was recorded from the pilot on 2026-10-24.
//
// Fed at 2 Hz. Smoothing, windows and fit weights are set in seconds or per
// cycle, not per sample, so other rates work too; 10 Hz was tried and helped
// high flows in simulation, but made the real pilot reading scatter more.
// The fit allows for the shape being a few percent off (it always is a
// little) and is fed unbiased positions, otherwise the slow pilot reading
// scatters by tens of percent; and only slow cycles teach the shape, since
// the input averaging rounds off the peak of a short one.
//
// Phase 0 = the field rising through shape(0); one cycle = one count.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>
#ifdef LF_DEBUG
#include <cstdio>
#endif
#ifdef ESP_PLATFORM
#include "esp_timer.h"  // esp_timer_get_time(): 64-bit microseconds, no 49-day wrap
#endif
#if defined(USE_ESP32) || defined(USE_ESP8266) || defined(USE_RP2040) || defined(USE_LIBRETINY)
#define LIVE_FLOW_PERSIST 1
#include "esphome/core/preferences.h"
#include "esphome/core/log.h"
#endif

namespace live_flow {

static const int N = 128;
static const float SEED_SHAPE[N] = {
    +0.0295f, +0.0742f, +0.1196f, +0.1656f, +0.2113f, +0.2568f, +0.3027f, +0.3485f,
    +0.3946f, +0.4408f, +0.4876f, +0.5349f, +0.5825f, +0.6298f, +0.6770f, +0.7228f,
    +0.7668f, +0.8094f, +0.8502f, +0.8890f, +0.9249f, +0.9558f, +0.9795f, +0.9945f,
    +1.0000f, +0.9958f, +0.9811f, +0.9563f, +0.9222f, +0.8806f, +0.8328f, +0.7815f,
    +0.7285f, +0.6746f, +0.6190f, +0.5624f, +0.5049f, +0.4470f, +0.3893f, +0.3317f,
    +0.2737f, +0.2155f, +0.1576f, +0.0999f, +0.0430f, -0.0129f, -0.0676f, -0.1214f,
    -0.1744f, -0.2261f, -0.2765f, -0.3258f, -0.3734f, -0.4196f, -0.4644f, -0.5068f,
    -0.5470f, -0.5853f, -0.6210f, -0.6543f, -0.6852f, -0.7141f, -0.7413f, -0.7670f,
    -0.7914f, -0.8148f, -0.8374f, -0.8583f, -0.8780f, -0.8960f, -0.9126f, -0.9268f,
    -0.9393f, -0.9502f, -0.9601f, -0.9686f, -0.9765f, -0.9835f, -0.9898f, -0.9945f,
    -0.9979f, -0.9995f, -1.0000f, -0.9991f, -0.9976f, -0.9950f, -0.9922f, -0.9887f,
    -0.9848f, -0.9795f, -0.9731f, -0.9652f, -0.9563f, -0.9466f, -0.9369f, -0.9275f,
    -0.9179f, -0.9078f, -0.8974f, -0.8868f, -0.8756f, -0.8632f, -0.8494f, -0.8339f,
    -0.8169f, -0.7987f, -0.7796f, -0.7592f, -0.7373f, -0.7136f, -0.6884f, -0.6611f,
    -0.6313f, -0.5985f, -0.5629f, -0.5248f, -0.4847f, -0.4437f, -0.4013f, -0.3593f,
    -0.3170f, -0.2747f, -0.2318f, -0.1894f, -0.1463f, -0.1027f, -0.0590f, -0.0149f,
};
static const float SEED_OFFSET = 56.265f;
static const float SEED_AMPLITUDE = 18.406f;

// Tracking (validated offline against the pilot log and simulated burner steps).
// Everything below is either estimator tuning or capacity. Nothing assumes a
// particular house, flow or meter: the noise level, sample rate, turn
// hysteresis and which cycles are good enough to learn from are all measured.
static const float H_SIGMAS = 5.0f;   // turn hysteresis, in measured noise sigmas...
#ifndef LF_MERR_K
#define LF_MERR_K 3.0
#endif
static const double MERR_ALPHA = 0.3;  // ...or this many times the recent model miss at turning points
static const double MERR_SMOOTHINGS = 100; // cycles used for that must last this many input smoothing times
static const int MA_MIN = 1;      // smallest moving average for turn detection (samples)
// Extra smoothing on the input, as a time constant so it means the same at any
// input rate (0.31 s = a factor of 0.8 per sample at 2 Hz, the validated rate).
static const float EMA_TAU = 0.3107f;
static const float EMA_REF_A = 0.8f;   // per-sample factor at which the fit weights were tuned
static const double TAU_PER_CYCLE = 30; // ...but never more than 1/30 of the current cycle
static const float START_S = 20;       // seconds of input used to pick the starting branch
static const double FLAT_REF_S = 0.5;  // one full-weight flat position per this many seconds
static const float NOISE_TAU = 50;     // seconds over which the noise level is averaged
static const float NOISE_INIT = 0.3f;  // starting guess for the raw noise (uT), measured from then on
static const float MIN_SAMPLES_EXT = 40;   // samples per cycle needed to measure a peak/trough
static const float MIN_SAMPLES_NOISE = 200; // samples per cycle below which the signal barely moves between samples
#ifndef LF_CHI
#define LF_CHI 2.0f
#endif
static const float CHI = LF_CHI;    // fit-consistency limit that marks a flow change
// The learned shape is never exact (it is learned from noisy cycles and the
// meter does not repeat itself perfectly), so positions read off it carry a
// systematic error besides the noise. Treating that as this fraction of the
// amplitude keeps the fit on its long windows; without it a shape error of a
// few percent breaks every window longer than ~10 s and the reading scatters.
#ifndef LF_SHAPE_TOL
#define LF_SHAPE_TOL 0.02f
#endif
static const float SHAPE_TOL = LF_SHAPE_TOL;
static const float MINW = 0.15f;  // a window must be this full before it is used
static const int NW = 7;
static const float WINS[NW] = {5, 10, 20, 40, 80, 160, 320};  // seconds
// Fit history: samples are combined into bins of 1/BIN_PER_CYCLE of a cycle
// (at most BIN_MAX_S), so a slow pilot fills the 320 s window with 0.5 s bins
// while a fast flow keeps the full time resolution of the input (over a
// shorter span, which is all a fast flow needs).
static const int HMAX = 700;
static const float BIN_PER_CYCLE = 32;
static const float BIN_MAX_S = 0.5f;
static const double FIT_EVERY_S = 0.25;   // the fit runs at most this often (bounds CPU at any input rate)
static const double CYCLE_LOCK_AGREE = 0.08;  // last-cycle estimate is used while it agrees with the fit this closely

// Adaptation.
static const float EXT_ALPHA = 0.5f;      // weight of each new peak/trough pair
static const float STEADY_TOL = 0.03f;    // consecutive cycles must agree this closely (or the timing resolution)
static const float BIN_FILL = 1.0f;       // samples per shape bin before a learned shape is used
// Weight of each learned update grows with how long the flow it was learned
// from lasted, so a stretch of flow counts by its duration, not by how many
// cycles it happened to contain (a 5 min burn of 10 s cycles once made a dozen
// full-weight updates and overwrote the shape seen at slow flow).
#ifndef LF_LEARN_TAU
#define LF_LEARN_TAU 2400.0
#endif
static const double LEARN_TAU_S = LF_LEARN_TAU;  // one ~11 min pilot cycle gives ~0.25
static const float SHAPE_BIG = 0.25f;     // RMS change that counts as "different"
// A cycle must last this many input smoothing times to be learned from. The
// input averaging rounds off the sharp peak of a short cycle (the water heater's
// 11 s cycles at 2 Hz came out ~3% different from the pilot's), and a shape a
// few percent off makes the slow pilot reading scatter; so only cycles long
// enough to be smeared by well under 1% teach the shape. The pilot gives one
// every ~11 min, which is all the shape needs.
#ifndef LF_LEARN_SMOOTHINGS
#define LF_LEARN_SMOOTHINGS 150
#endif
static const float LEARN_SMOOTHINGS = LF_LEARN_SMOOTHINGS;
static const int REC_MAX = 2500;          // samples of one cycle (thinned out if a cycle is longer)
static const uint32_t MAGIC = 0x4C46310Bu;
static const double SAVE_EVERY_S = 1800;  // flash writes at most every 30 min

struct Learned {
  uint32_t magic;
  float offset, amplitude;
  float shape[N];
  uint32_t shape_updates;
};

class Tracker {
 public:
  Tracker() { seed_(); }

  // Feed one Z sample (uT) at time t (s), at any steady rate from ~2 to ~20 Hz.
  // Returns true once tracking.
  bool update(double t, float z) {
#ifdef ESP_PLATFORM
    int64_t t0 = esp_timer_get_time();
    bool ok = update_(t, z);
    int64_t us = esp_timer_get_time() - t0;
    if (us > max_us_) max_us_ = us;
    return ok;
#else
    return update_(t, z);
#endif
  }

  // Longest single update() since the last call, in ms (a CPU-load check).
  float take_max_update_ms() { float v = max_us_ / 1000.0f; max_us_ = 0; return v; }

 private:
  int64_t max_us_ = 0;
  bool update_(double t, float z) {
    if (!loaded_) load_();
    if (!started_) return start_(t, z);
    double dt = t - tp_;
    tp_ = t;
    if (dt <= 0) return true;
    if (dt > 30 * dt_avg_) dt = 30 * dt_avg_;   // a stall (e.g. Wi-Fi hiccup) is not a new sample rate
    dt_avg_ += 0.01 * (dt - dt_avg_);
    // Less smoothing when the cycle is short, so it does not round off the
    // waveform the shape lookup relies on (the signal moves fast then anyway).
    double tau = EMA_TAU;
    if (r_ > 1e-9 && 1.0 / r_ / TAU_PER_CYCLE < tau) tau = 1.0 / r_ / TAU_PER_CYCLE;
    float a = 1.0f - std::exp((float) (-dt_avg_ / tau));
    // Noise: second differences of the raw input, but only while the meter is
    // slow enough that the waveform itself does not curve between samples.
    // (Not before the first rate estimate: right after a start or recalibration
    // the rate reads 0 even if the meter is turning fast.)
    if (r_known_ && r_ * dt_avg_ * MIN_SAMPLES_NOISE <= 1.0 && n2_ >= 2) {
      double d2 = z - 2 * z1_ + z2_;
      nvar_ += std::min(1.0, dt_avg_ / NOISE_TAU) * (d2 * d2 / 6.0 - nvar_);
      if (nvar_ < 1e-4) nvar_ = 1e-4;
    }
    z2_ = z1_; z1_ = z; if (n2_ < 2) n2_++;
    sz_ = std::sqrt(nvar_ * a / (2 - a));              // after the EMA below
    // The EMA output is correlated over a few samples; scale the fit weights so
    // a second of input carries the same weight at any rate (1 at 2 Hz).
    wscale_ = (a / (2 - a)) / (EMA_REF_A / (2 - EMA_REF_A));
    // Turn hysteresis: above the noise, and above how far the field's recent
    // turning points have been from the model (drift between corrections).
    H_ = std::max(H_SIGMAS * sz_, (float) (LF_MERR_K * merr_));
    H_ = std::max(H_, 3.0f * SHAPE_TOL * amp_);   // and above the shape's own tolerance at a turn
    if (H_ > 0.15f * amp_ * (smax_ - smin_)) H_ = 0.15f * amp_ * (smax_ - smin_);
    zf_ += a * (z - zf_);
    ring_[ri_] = zf_;
    ri_ = (ri_ + 1) % RING;
    if (rn_ < RING) rn_++;
    float zx = smoothed_();
    // How far the smoothed signal trails the input (EMA + moving average), so
    // cycle timing can be referred back to the input itself.
    ema_lag_ = dt_avg_ * (1 - a) / a;
    lag_ = ema_lag_ + 0.5 * (m_cur_ - 1) * dt_avg_;

    // Branch (rising/falling) with hysteresis on the cycle-sized moving average,
    // and only where a turn is plausible (a peak above the midline, a trough
    // below it), so noise on the long flat bottom cannot flip the branch.
    float umid = off_ + amp_ * (smax_ + smin_) / 2;
    if (rising_) {
      if (zx > ext_) ext_ = zx;
      if (zx < ext_ - H_ && ext_ > umid) { on_extreme_(true, ext_, t); rising_ = false; ext_ = zx; }
    } else {
      if (zx < ext_) ext_ = zx;
      if (zx > ext_ + H_ && ext_ < umid) { on_extreme_(false, ext_, t); rising_ = true; ext_ = zx; }
    }

    // Reference-level crossings (alternating up/down) time each cycle for shape
    // learning.
    float lref = off_ + amp_ * shape_[0];
    if (zxprev_ < lref && zx >= lref && rising_ && !last_up_) {
      on_cross_(t - dt * (zx - lref) / (zx - zxprev_) - lag_, true);
      last_up_ = true;
    }
    if (zxprev_ > lref && zx <= lref && !rising_ && last_up_) {
      on_cross_(t - dt * (lref - zx) / (zxprev_ - zx) - lag_, false);
      last_up_ = false;
    }
    zxprev_ = zx;
    // Record the input itself (not the smoothed signal, whose lag and rounding
    // would bend the shape at faster flows); the crossings above were moved back
    // by the smoothing lag, so both use the same time reference.
    if (rec_on_) {
      rec_sz_ += z; rec_st_ += t - rec_t0_;
      if (++rec_skip_ >= rec_step_) {
        if (rec_n_ == REC_MAX) {           // long cycle: average pairs and halve the rate
          for (int k = 0; k < REC_MAX / 2; k++) {
            rec_t_[k] = 0.5f * (rec_t_[2*k] + rec_t_[2*k+1]); rec_z_[k] = 0.5f * (rec_z_[2*k] + rec_z_[2*k+1]);
          }
          rec_n_ = REC_MAX / 2; rec_step_ *= 2;
        }
        rec_t_[rec_n_] = (float) (rec_st_ / rec_skip_);
        rec_z_[rec_n_] = (float) (rec_sz_ / rec_skip_);
        rec_n_++;
        rec_skip_ = 0; rec_sz_ = 0; rec_st_ = 0;
      }
    }

    // Position within the cycle on the current branch, forward only.
    double base = phase_(zf_, rising_);
    double c = base + std::floor(phi_ - base);
    if (c < phi_ - 0.02) c += 1;
    if (c - phi_ > 0.5) c -= 1;
    // The fit gets this unbiased position; the tracked phase below only ever
    // moves forward, and the largest of many noisy readings runs ahead where
    // the field changes slowly, which the fit would read as a burst of flow
    // followed by a standstill.
    double c_raw = c;
    // "Flat": near the top/bottom of the model, or already turning back from
    // a peak/trough that the turn detector has not confirmed yet. Either way
    // the field cannot say where in the turn the meter is.
    bool turning = rising_ ? (ext_ > umid && ext_ - zx > 0.5f * H_) : (ext_ < umid && zx - ext_ > 0.5f * H_);
    bool flat = (rising_ && zf_ > zmax_() - H_) || (!rising_ && zf_ < zmin_() + H_) || turning;
    double lim = 0;
    if (flat) {
      // Flat top/bottom: coast at the current rate, never past the turning point.
      lim = rising_ ? pmax_ : pmin_;
      lim += std::floor(phi_ - lim);
      if (lim < phi_ - 0.02) lim += 1;
      double coast = phi_ + r_ * dt;
      if (coast > lim) coast = lim;
      if (coast > c) c = coast;
    }
    if (c > phi_) phi_ = c;

    flat_now_ = flat;
    if (flat) {
      // Here the field says only that the meter is somewhere in the flat
      // stretch around the turn, not where: give the fit the middle of the
      // stretch with the uncertainty of anywhere inside it.
      double da, db;
      flat_span_(rising_, da, db);
      p_exit_ = lim + db;
      // Weighted per second of flat stretch (as at the validated 2 Hz), so the
      // flat stretches count the same at any input rate. Right after a flow
      // stops inside a flat stretch, these keep pulling the reading down.
      double sf = (db - da) / std::sqrt(12.0);
      double share = std::min(1.0, dt_avg_ / FLAT_REF_S);
      add_(t, lim + (da + db) / 2, share / (sf * sf), false);
    } else {
      double p = phi_ - std::floor(phi_);
      float dzdp = amp_ * std::fabs(sh_(p + 0.004) - sh_(p - 0.004)) / 0.008f;
      double sze = std::sqrt((double) sz_ * sz_ + (double) (SHAPE_TOL * amp_) * (SHAPE_TOL * amp_));
      double sp = sze / (dzdp > 1e-3f ? dzdp : 1e-3f);
      add_(t, c_raw, wscale_ / (sp * sp), true);
      t_inf_ = t; p_inf_ = phi_; r_inf_ = r_;
    }
    if (bin_due_()) {
      flush_();
      if (t - t_fit_ >= FIT_EVERY_S * 0.999) { fit_(t); t_fit_ = t; }
    }
    // Save a newly learned shape right away (at most once per pilot cycle,
    // ~11 min); offset/amplitude-only changes at most every SAVE_EVERY_S.
    if (dirty_ && (save_now_ || (t - last_save_) >= SAVE_EVERY_S)) { save_(t); save_now_ = false; }
    return true;
  }


 public:
  bool ready() const { return started_; }
  double cycles_per_s() const { return r_out_; }
  double phase() const { return phi_; }  // unwrapped, in counts
  float offset() const { return off_; }
  float amplitude() const { return amp_; }
  uint32_t shape_updates() const { return updates_; }
  int last_learn() const { return last_learn_; }  // see LearnResult

  enum LearnResult { NONE = 0, ACCEPTED = 1, NOT_STEADY = 2, OUT_OF_RANGE = 3, DIFFERENT_WAITING = 4, ACCEPTED_NEW_SHAPE = 5, FIRST_CYCLE = 6, COLLECTING = 7, RECALIBRATED = 8 };
  float noise() const { return std::sqrt(nvar_); }   // measured raw field noise (uT)

  // After the meter package recalibrates: take the new range of the field, and
  // start tracking afresh. Same axis: keep the learned shape. New axis: the
  // shape is unknown, so start from the seed and re-learn it.
  void recalibrate(float zmin, float zmax, bool new_axis, float span_ratio) {
    if (new_axis) seed_();
    amp_ = span_ratio * (zmax - zmin) / (smax_ - smin_);
    off_ = (zmax + zmin) / 2 - amp_ * (smax_ + smin_) / 2;
    started_ = false; n0_ = 0; rec_on_ = false; hn_ = 0; hs_ = 0; bn_ = 0; r_ = 0; r_out_ = 0; r_known_ = false;
    have_pk_ = have_tr_ = false; prev_period_ = 0; prev_split_ = 0; diff_count_ = 0;
    acc_reset_();
    last_learn_ = RECALIBRATED;
    dirty_ = true; save_now_ = true;
  }

  // Forget everything learned and go back to the recorded seed shape.
  void reset() {
    seed_();
    started_ = false; n0_ = 0; rec_on_ = false; hn_ = 0; hs_ = 0; bn_ = 0; r_ = 0; r_out_ = 0; r_known_ = false;
    have_pk_ = have_tr_ = false; prev_period_ = 0; prev_split_ = 0; diff_count_ = 0;
    dirty_ = true; last_save_ = -1e9;
#ifdef LIVE_FLOW_PERSIST
    save_(0);
#endif
  }

 private:
  // ---- shape model -------------------------------------------------------
  float shape_[N];
  float off_ = SEED_OFFSET, amp_ = SEED_AMPLITUDE;
  uint32_t updates_ = 0;
  double pmax_ = 0, pmin_ = 0;
  float smax_ = 1, smin_ = -1;
  static const int B = 256;
  float fz_[B], fp_[B], rz_[B], rp_[B];

  void seed_() {
    std::memcpy(shape_, SEED_SHAPE, sizeof(shape_));
    off_ = SEED_OFFSET; amp_ = SEED_AMPLITUDE; updates_ = 0;
    build_();
    acc_reset_();
  }
  float sh_(double p) const {
    double x = (p - std::floor(p)) * N;
    int i = (int) x;
    double f = x - i;
    i %= N;
    int j = (i + 1) % N;
    return (float) (shape_[i] * (1 - f) + shape_[j] * f);
  }
  float zmax_() const { return off_ + amp_ * smax_; }
  // Phase offsets (cycles) before and after a turn within which the field
  // stays inside the turn hysteresis, from the current shape.
  void flat_span_(bool peak, double &da, double &db) const {
    int i0 = 0;
    for (int i = 1; i < N; i++) if (peak ? shape_[i] > shape_[i0] : shape_[i] < shape_[i0]) i0 = i;
    float thr = H_ / amp_;
    int a = 1, b = 1;
    while (a < N / 2 && std::fabs(shape_[(i0 - a + N) % N] - shape_[i0]) < thr) a++;
    while (b < N / 2 && std::fabs(shape_[(i0 + b) % N] - shape_[i0]) < thr) b++;
    da = -(double) a / N; db = (double) b / N;
  }
  float zmin_() const { return off_ + amp_ * smin_; }
  void build_() {
    int imax = 0, imin = 0;
    for (int i = 1; i < N; i++) {
      if (shape_[i] > shape_[imax]) imax = i;
      if (shape_[i] < shape_[imin]) imin = i;
    }
    pmax_ = (double) imax / N; pmin_ = (double) imin / N;
    smax_ = shape_[imax]; smin_ = shape_[imin];
    double pmin_r = pmin_ < pmax_ ? pmin_ + 1 : pmin_;  // falling: pmax -> pmin (wrapping)
    double pmax_r = pmax_ < pmin_ ? pmax_ + 1 : pmax_;  // rising: pmin -> pmax (wrapping)
    for (int k = 0; k < B; k++) {
      double pf = pmax_ + (pmin_r - pmax_) * k / (B - 1);
      double pr = pmin_ + (pmax_r - pmin_) * k / (B - 1);
      fp_[k] = (float) pf; rp_[k] = (float) pr; fz_[k] = sh_(pf); rz_[k] = sh_(pr);
      if (k > 0) {
        if (fz_[k] > fz_[k - 1]) fz_[k] = fz_[k - 1];
        if (rz_[k] < rz_[k - 1]) rz_[k] = rz_[k - 1];
      }
    }
  }
  // Position within the cycle for field z on the given branch.
  double phase_(float z, bool rising) const {
    float u = (z - off_) / amp_;
    if (u > smax_) u = smax_;
    if (u < smin_) u = smin_;
    const float *zz = rising ? rz_ : fz_;
    const float *pp = rising ? rp_ : fp_;
    int lo = 0, hi = B - 1;
    while (hi - lo > 1) {
      int mid = (lo + hi) / 2;
      bool before = rising ? (zz[mid] <= u) : (zz[mid] >= u);
      if (before) lo = mid; else hi = mid;
    }
    float a = zz[lo], b = zz[hi];
    double f = (b != a) ? (u - a) / (b - a) : 0;
    if (f < 0) f = 0;
    if (f > 1) f = 1;
    return pp[lo] + (pp[hi] - pp[lo]) * f;
  }

  // ---- startup -----------------------------------------------------------
  bool started_ = false;
  int n0_ = 0;
  float z0_ = 0;
  double t0_ = 0;
  bool start_(double t, float z) {
    // Decide the initial branch from the trend of the first ~20 s.
    if (n0_ == 0) { z0_ = z; t0_ = t; }
    zf_ = z;
    ++n0_;
    if (t - t0_ < START_S || n0_ < 3) return false;
    dt_avg_ = (t - t0_) / (n0_ - 1);
    rising_ = z > z0_;
    ext_ = zf_ = zxprev_ = z;
    // the last crossing was upward if we are above the reference level now
    last_up_ = z > off_ + amp_ * shape_[0];
    rn_ = 0; ri_ = 0;
    phi_ = phase_(zf_, rising_);
    tp_ = t; started_ = true; r_ = 0; bn_ = 0;
    return false;
  }

  // ---- branch / extremes -------------------------------------------------
  bool rising_ = true;
  float zf_ = 0, ext_ = 0, zxprev_ = 0;
  bool last_up_ = false;
  double tp_ = 0, phi_ = 0, r_ = 0, r_out_ = 0;
  double dt_avg_ = 0.5, nvar_ = NOISE_INIT * NOISE_INIT, z1_ = 0, z2_ = 0;
  int n2_ = 0;
  float sz_ = 0.3f, H_ = 1.5f, wscale_ = 1.0f;
  double ema_lag_ = 0.125, lag_ = 0.125, t_fit_ = -1e9;
  double merr_ = 0;
  bool r_known_ = false;  // a rate has been fitted since the (re)start   // recent distance of measured turning points from the model (uT)
  bool flat_now_ = false;
  double t_inf_ = 0, p_inf_ = 0, r_inf_ = 0, p_exit_ = 0;
  static const int RING = 160;           // capacity; the window itself is at most RING_S
  static constexpr float RING_S = 15.0f;
  float ring_[RING];
  int ri_ = 0, rn_ = 0;
  // Peaks/troughs are taken from a moving average sized to the cycle so noise
  // does not inflate them; a few seconds at pilot speed.
  int m_cur_ = MA_MIN;
  float smoothed_() {
    double period = r_ > 1e-6 ? 1.0 / r_ : 1e9;
    int mmax = (int) (RING_S / dt_avg_);
    if (mmax > RING) mmax = RING;
    if (mmax < MA_MIN) mmax = MA_MIN;
    int m = (int) (period / 40.0 / dt_avg_);   // 1/40 of a cycle
    if (m < MA_MIN) m = MA_MIN;
    if (m > mmax) m = mmax;
    // Shrink at once, but grow by one sample at a time: a sudden longer window
    // would pull older samples back in and make the average jump, which looked
    // like a peak or trough when a big flow stopped.
    if (m < m_cur_) m_cur_ = m; else if (m > m_cur_) m_cur_++;
    m = m_cur_;
    if (m > rn_) m = rn_;
    double s = 0;
    for (int k = 1; k <= m; k++) s += ring_[(ri_ - k + RING) % RING];
    return (float) (s / m);
  }
  bool have_pk_ = false, have_tr_ = false;
  float pk_ = 0, tr_ = 0;
  double t_pk_ = 0, t_tr_ = 0, r_pk_ = 0, r_tr_ = 0;
  void on_extreme_(bool peak, float value, double t) {
    double period = r_ > 1e-6 ? 1.0 / r_ : 1e9;
    // Enough samples, and enough smoothing time constants, to see the turn fully.
    bool slow = period > MIN_SAMPLES_EXT * std::max(dt_avg_, 2 * ema_lag_) && period < 1e8;
    // Only from cycles far slower than the input smoothing, which would shave
    // the peaks and look like a model miss.
    if (slow && period > MERR_SMOOTHINGS * (ema_lag_ + 0.5 * dt_avg_)) {
      double d = std::fabs(peak ? value - zmax_() : value - zmin_());
      // A real drift grows a little each cycle; a single wild value (a false
      // turn while the flow changes) is limited so it cannot inflate it.
      double lim = std::max(3 * merr_, (double) (H_SIGMAS * sz_));
      if (d > lim) d = lim;
      merr_ += MERR_ALPHA * (d - merr_);
    }
    if (peak) { pk_ = value; have_pk_ = slow; t_pk_ = t; r_pk_ = r_; }
    else      { tr_ = value; have_tr_ = slow; t_tr_ = t; r_tr_ = r_; }
    if (!(have_pk_ && have_tr_)) return;
    // Only from a steady stretch: the flow estimate agreed at both turns, and
    // the time between them matches what the shape predicts at that flow.
    double rm = (r_pk_ + r_tr_) / 2;
    if (std::fabs(r_pk_ - r_tr_) > 0.35 * rm) return;
    double gap = std::fabs(t_tr_ - t_pk_) * rm;  // in cycles
    double fall = pmin_ - pmax_;
    if (fall < 0) fall += 1;
    double expect = peak ? 1 - fall : fall;       // trough->peak is the rising half
    if (std::fabs(gap - expect) > 0.35 * expect) return;
    float amp_obs = (pk_ - tr_) / (smax_ - smin_);
    float off_obs = (pk_ + tr_) / 2 - amp_obs * (smax_ + smin_) / 2;
    // Only small corrections here (temperature drift is slow); anything larger,
    // like a moved case, is left to shape learning, which needs it to repeat.
    if (std::fabs(amp_obs / amp_ - 1) > 0.10f || std::fabs(off_obs - off_) > 0.15f * amp_) return;
    amp_ += EXT_ALPHA * (amp_obs - amp_);
    off_ += EXT_ALPHA * (off_obs - off_);
    dirty_ = true;
  }

  // ---- shape learning ----------------------------------------------------
  bool rec_on_ = false;
  double rec_t0_ = 0, rec_fall_ = -1;
  int rec_n_ = 0, rec_step_ = 1, rec_skip_ = 0;
  double rec_sz_ = 0, rec_st_ = 0;
  float rec_t_[REC_MAX], rec_z_[REC_MAX];
  float prev_period_ = 0, prev_split_ = 0;
  int diff_count_ = 0, last_learn_ = NONE;
  void on_cross_(double tc, bool up) {
    if (!up) { if (rec_on_ && rec_fall_ < 0) rec_fall_ = tc - rec_t0_; return; }
    if (rec_on_) finish_cycle_(tc);
    rec_on_ = true; rec_t0_ = tc; rec_n_ = 0; rec_fall_ = -1; rec_step_ = 1; rec_skip_ = 0; rec_sz_ = 0; rec_st_ = 0;
  }
  // Shape samples from consecutive steady cycles, folded onto one cycle. A slow
  // cycle fills every bin on its own; a fast one only has a few samples, so
  // several steady cycles are combined until every bin has data.
  float acc_sum_[N], acc_cnt_[N];
  int acc_cycles_ = 0;
  double acc_span_ = 0;  // seconds of flow folded into the accumulators
  void acc_reset_() { for (int i = 0; i < N; i++) { acc_sum_[i] = 0; acc_cnt_[i] = 0; } acc_cycles_ = 0; acc_span_ = 0; }
  void finish_cycle_(double tc) {
    float period = (float) (tc - rec_t0_);
    float split = rec_fall_ > 0 ? (float) rec_fall_ / period : -1;
    // "Steady": same period and same rise/fall split as the previous cycle, to
    // within 3% or the timing resolution of the samples, whichever is larger.
    float tol = std::max(STEADY_TOL, (float) (2 * dt_avg_ / period));
    bool steady = prev_period_ > 0 && split > 0 && prev_split_ > 0 &&
                  std::fabs(period / prev_period_ - 1) < tol &&
                  std::fabs(split - prev_split_) < tol;
    bool range = rec_n_ >= 8;          // at least a handful of samples in the cycle
    bool first = prev_period_ <= 0;
    prev_period_ = period; prev_split_ = split;
#ifdef LF_DEBUG
    fprintf(stderr, "t=%.0f CYCLE period=%.1f split=%.3f steady=%d n=%d acc=%d\n", tc, period, split, steady, rec_n_, acc_cycles_);
#endif
    // Learn only from cycles that are long next to the input's own smoothing
    // (block average + EMA), which would otherwise round off the peaks.
    if (period < LEARN_SMOOTHINGS * (ema_lag_ + 0.5 * dt_avg_)) range = false;
    if (!range) { acc_reset_(); last_learn_ = OUT_OF_RANGE; return; }
    if (first) { acc_reset_(); last_learn_ = FIRST_CYCLE; return; }  // nothing to compare with yet
    if (!steady) { acc_reset_(); last_learn_ = NOT_STEADY; return; }
    // Fold this cycle onto the accumulated one (time is proportional to volume).
    // Shape index i is phase i/N, so each sample is shared between the two
    // nearest indices (plain bucketing would shift the shape half a bin per
    // update and walk it around the cycle).
    for (int k = 0; k < rec_n_; k++) {
      // Recording started when the crossing was detected, one lag after it, so
      // the last lag's worth of samples belongs to the start of the cycle.
      float x = rec_t_[k] / period * N;
      if (x >= N && x < 2 * N) x -= N;
      if (x < 0 || x >= N) continue;
      int i0 = (int) x;
      float f = x - i0;
      int i1 = (i0 + 1) % N;
      float u = (rec_z_[k] - off_) / amp_;
      acc_sum_[i0] += (1 - f) * u; acc_cnt_[i0] += 1 - f;
      acc_sum_[i1] += f * u; acc_cnt_[i1] += f;
    }
    acc_cycles_++;
    acc_span_ += period;
    float nw[N];
    for (int i = 0; i < N; i++) {
      if (acc_cnt_[i] < BIN_FILL) { last_learn_ = COLLECTING; return; }
      nw[i] = acc_sum_[i] / acc_cnt_[i];
    }
    float beta = (float) (1.0 - std::exp(-acc_span_ / LEARN_TAU_S));
    acc_reset_();
    float sm[N];
    for (int i = 0; i < N; i++) {
      float s = 0;
      for (int d = -2; d <= 2; d++) s += nw[(i + d + N) % N];
      sm[i] = s / 5;
    }
    float rms = 0;
    for (int i = 0; i < N; i++) rms += (sm[i] - shape_[i]) * (sm[i] - shape_[i]);
    rms = std::sqrt(rms / N);
    if (rms > SHAPE_BIG) {
      // Much different from what we have: only believe it once it repeats
      // (e.g. the case was moved), then adopt it quickly.
      if (++diff_count_ < 2) { last_learn_ = DIFFERENT_WAITING; return; }
      beta = 0.75f;
      last_learn_ = ACCEPTED_NEW_SHAPE;
    } else {
      diff_count_ = 0;
      last_learn_ = ACCEPTED;
    }
    for (int i = 0; i < N; i++) shape_[i] += beta * (sm[i] - shape_[i]);
    // Re-gauge so the shape stays -1..+1, moving its scale into offset/amplitude.
    float mx = shape_[0], mn = shape_[0];
    for (int i = 1; i < N; i++) { if (shape_[i] > mx) mx = shape_[i]; if (shape_[i] < mn) mn = shape_[i]; }
    float mid = (mx + mn) / 2, half = (mx - mn) / 2;
    if (half < 0.2f) { seed_(); last_learn_ = OUT_OF_RANGE; return; }  // degenerate: start over
    for (int i = 0; i < N; i++) shape_[i] = (shape_[i] - mid) / half;
    off_ += amp_ * mid; amp_ *= half;
    build_();
    updates_++;
    save_now_ = true;
#ifdef LF_DEBUG
    fprintf(stderr, "LEARN %u rms=%.3f s0=%.3f pmax=%.3f pmin=%.3f off=%.2f amp=%.2f shape:", (unsigned) updates_, rms, shape_[0], pmax_, pmin_, off_, amp_);
    for (int i = 0; i < N; i += 8) fprintf(stderr, " %.2f", shape_[i]);
    fprintf(stderr, "\n");
#endif
    dirty_ = true;
  }

  // ---- flow fit ----------------------------------------------------------
  struct Pt { double t, phi; float w; bool inf; };   // w: fit weight; inf: the field gave a position
  Pt h_[HMAX];
  int hs_ = 0, hn_ = 0;
  // Current bin: weighted means of time and phase of same-kind samples.
  int bn_ = 0;
  bool binf_ = true;
  double bw_ = 0, bt_ = 0, bp_ = 0, bt0_ = 0, bp0_ = 0;
  void add_(double t, double phi, double w, bool inf) {
    if (bn_ > 0 && inf != binf_) flush_();
    if (bn_ == 0) { binf_ = inf; bw_ = bt_ = bp_ = 0; bt0_ = t; bp0_ = phi; }
    bw_ += w; bt_ += w * (t - bt0_); bp_ += w * (phi - bp0_); bn_++;
  }
  bool bin_due_() const {
    if (bn_ == 0) return false;
    double width = r_ > 1e-6 ? 1.0 / r_ / BIN_PER_CYCLE : BIN_MAX_S;
    if (width > BIN_MAX_S) width = BIN_MAX_S;
    return bn_ * dt_avg_ >= width * 0.999;
  }
  void flush_() {
    if (bn_ == 0) return;
    push_(bt0_ + bt_ / bw_, bp0_ + bp_ / bw_, (float) bw_, binf_);
    bn_ = 0;
  }
  void push_(double t, double phi, float w, bool inf) {
    while (hn_ > 0 && h_[hs_].t < t - WINS[NW - 1]) { hs_ = (hs_ + 1) % HMAX; hn_--; }
    if (hn_ == HMAX) { hs_ = (hs_ + 1) % HMAX; hn_--; }
    h_[(hs_ + hn_) % HMAX] = {t, phi, w, inf};
    hn_++;
  }
  // Longest window over which a straight line (constant flow) still fits. One
  // pass from the newest sample backwards; times and phases relative to now so
  // the sums stay small (the C6 has no FPU; this is ~1 ms per sample).
  void fit_(double t) {
    bool have = false;
    double rsel = 0, sw = 0, st = 0, sp2 = 0, stt = 0, stp = 0, spp = 0, tfirst = t;
    int n = 0, k = 0, n_inf = 0, sel_inf = 0;
    for (int i = hn_ - 1; i >= -1 && k < NW; i--) {
      bool end = (i < 0) || (h_[(hs_ + i) % HMAX].t < t - WINS[k]);
      while (end && k < NW) {
        if (n >= 5 && (t - tfirst) >= WINS[k] * MINW) {
          double tm = st / sw, pm = sp2 / sw;
          double den = stt - sw * tm * tm, num = stp - sw * tm * pm;
          if (den > 0) {
            double rr = num / den;
            double c2 = ((spp - sw * pm * pm) - rr * num) / (n > 2 ? n - 2 : 1);

            if (have && c2 > CHI) { k = NW; break; }
            rsel = rr > 0 ? rr : 0;
            have = true;
            sel_inf = n_inf;
          }
        }
        k++;
        if (k < NW && i >= 0) end = h_[(hs_ + i) % HMAX].t < t - WINS[k];
      }
      if (i < 0 || k >= NW) break;
      const Pt &q = h_[(hs_ + i) % HMAX];
      double w = q.w, tt = q.t - t, pp = q.phi - phi_;
      sw += w; st += w * tt; sp2 += w * pp; stt += w * tt * tt; stp += w * tt * pp; spp += w * pp * pp;
      n++;
      if (q.inf) n_inf++;
      tfirst = q.t;
    }
    if (have) { r_ = rsel; r_known_ = true; }
    // If the chosen window holds nothing but flat-stretch samples (typically
    // right after a big flow stopped near a turn), the fit has no information.
    // What is known: the meter has not left the stretch since the last real
    // position. With no other assumption, the rate is anywhere from zero up to
    // what would just have reached the end of the stretch; report the middle
    // of that range (never more than the rate before). It converges as soon as
    // the field moves again.
    if (flat_now_ && have && sel_inf == 0 && t > t_inf_) {
      double r_hi = (p_exit_ - p_inf_) / (t - t_inf_);
      double r_mid = 0.5 * (r_hi > 0 ? r_hi : 0);
      r_ = r_mid < r_inf_ ? r_mid : r_inf_;
    }
    // Cycle-locked estimate: the time the meter took for its last full cycle.
    // The shape's small errors repeat every cycle, so they cancel over exactly
    // one cycle; this removes the ripple at higher flows. Used only while it
    // agrees with the windowed fit (i.e. the flow is steady) and the cycle is
    // short enough to be recent.
    r_out_ = r_;
    double target = phi_ - 1.0;
    for (int i = hn_ - 1; i > 0; i--) {
      const Pt &a = h_[(hs_ + i - 1) % HMAX], &b = h_[(hs_ + i) % HMAX];
      if (a.phi <= target && b.phi >= target) {
        double tc = (b.phi > a.phi) ? a.t + (target - a.phi) / (b.phi - a.phi) * (b.t - a.t) : b.t;
        double span = t - tc;
        if (span > 0 && span <= WINS[NW - 1]) {
          // weighted straight-line fit over exactly that last cycle
          double sw = 0, st = 0, sp = 0, stt = 0, stp = 0;
          for (int j = i; j < hn_; j++) {
            const Pt &q = h_[(hs_ + j) % HMAX];
            double w = q.w, tt = q.t - t, pp = q.phi - phi_;
            sw += w; st += w * tt; sp += w * pp; stt += w * tt * tt; stp += w * tt * pp;
          }
          double den = stt - st * st / sw;
          if (den > 0) {
            double rc = (stp - st * sp / sw) / den;
            if (rc > 0 && r_ > 0 && std::fabs(rc / r_ - 1) < CYCLE_LOCK_AGREE) r_out_ = rc;
          }
        }
        break;
      }
      if (a.phi < target) break;
    }
  }

  // ---- persistence -------------------------------------------------------
  bool loaded_ = false, dirty_ = false, save_now_ = false;
  double last_save_ = -1e9;
#ifdef LIVE_FLOW_PERSIST
  esphome::ESPPreferenceObject pref_;
#endif
  void load_() {
    loaded_ = true;
#ifdef LIVE_FLOW_PERSIST
    using namespace esphome;  // for the ESP_LOG macros
    pref_ = esphome::global_preferences->make_preference<Learned>(MAGIC);
    Learned l;
    if (pref_.load(&l) && l.magic == MAGIC && l.amplitude > 1 && l.amplitude < 1000) {
      std::memcpy(shape_, l.shape, sizeof(shape_));
      off_ = l.offset; amp_ = l.amplitude; updates_ = l.shape_updates;
      build_();
      ESP_LOGI("live_flow", "Loaded learned shape (%u updates), offset %.1f, amplitude %.1f",
               (unsigned) updates_, off_, amp_);
    } else {
      ESP_LOGI("live_flow", "Using recorded seed shape");
    }
#endif
  }
  void save_(double t) {
    last_save_ = t;
    dirty_ = false;
#ifdef LIVE_FLOW_PERSIST
    Learned l;
    l.magic = MAGIC; l.offset = off_; l.amplitude = amp_; l.shape_updates = updates_;
    std::memcpy(l.shape, shape_, sizeof(shape_));
    pref_.save(&l);
#endif
  }
};

// One shared tracker for the device.
inline Tracker &tracker() {
  static Tracker t;
  return t;
}

}  // namespace live_flow
