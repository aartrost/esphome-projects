// Live gas flow from the magnetometer waveform: one state estimator.
//
// A diaphragm meter is positive displacement: the field depends only on how
// far the mechanism has turned, so one cycle of the field (the SHAPE) maps the
// field back to the meter's position within its cycle, and flow is how fast
// that position advances. Here that is a single extended Kalman filter over
//   phase (cycles), rate (cycles/s), offset (uT), amplitude (uT)
// with the measurement  z = offset + amplitude * shape(phase).
//   - Where the shape is steep the measurement pins the phase; where it is
//     flat (around each turn) it only measures offset/amplitude, and the
//     phase coasts at the current rate with growing uncertainty. No special
//     cases for flat stretches.
//   - The rate may wander in proportion to itself, so the estimate averages
//     over minutes at the pilot and over seconds at a burner.
//   - A flow change shows as measurements the prediction cannot explain:
//     then the phase is re-read directly from the shape (which is exact where
//     the filter's linearisation is not) and the rate is allowed to move.
//   - The shape itself is re-learned from steady slow cycles, timed by the
//     phase crossing whole cycles, and saved to flash with offset/amplitude.
// Fed at 2 Hz (each sample a 0.5 s average); nothing assumes the rate.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>
#ifdef LF_DEBUG
#include <cstdio>
#endif
#ifdef ESP_PLATFORM
#include "esp_timer.h"
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

// ---- tuning ---------------------------------------------------------------
#ifndef LF_SHAPE_TOL
#define LF_SHAPE_TOL 0.02
#endif
#ifndef LF_RATE_WALK
#define LF_RATE_WALK 0.01
#endif
#ifndef LF_GATE
#define LF_GATE 4.0
#endif
#ifndef LF_GATE_ALPHA
#define LF_GATE_ALPHA 0.2
#endif
static const double GATE_ALPHA = LF_GATE_ALPHA;  // memory of the gate's running mean of innovation^2/S
#ifndef LF_LEARN_SMOOTHINGS
#define LF_LEARN_SMOOTHINGS 150
#endif
#ifndef LF_LEARN_TAU
#define LF_LEARN_TAU 2400.0
#endif
static const double SHAPE_TOL = LF_SHAPE_TOL;   // shape error allowed for, as a fraction of amplitude
static const double RATE_WALK = LF_RATE_WALK;   // the rate may wander by this fraction of itself per sqrt(s)
#ifndef LF_RATE_FLOOR
#define LF_RATE_FLOOR 5e-6
#endif
static const double RATE_FLOOR = LF_RATE_FLOOR;          // ...and at least this much (cycles/s per sqrt(s)), so zero flow can restart
static const double PHASE_WALK = 0.003;         // within-cycle unevenness of the meter (cycles per sqrt(s))
static const double DRIFT_WALK = 0.01;          // offset/amplitude drift (uT per sqrt(s); ~0.6 uT/h, seen after burns)
static const double GATE = LF_GATE;             // innovation^2/S above which the prediction is not believed
#ifndef LF_EDGE_PAD
#define LF_EDGE_PAD 0.3
#endif
static const double EDGE_PAD = LF_EDGE_PAD;     // flat-stretch edges padded by this fraction of the stretch
#ifndef LF_TURN_ZONE
#define LF_TURN_ZONE 0.15
#endif
static const double TURN_ZONE = LF_TURN_ZONE;   // cycles past a turn within which the detector may still lag
static const double REACQ_SIGMAS = 6.0;         // ...and the phase is re-read from the shape
static const double RATE_CHECK_S = 20.0;        // the measured phase advance over at least this long is checked against the rate...
#ifndef LF_RCK
#define LF_RCK 4.0
#endif
static const double RATE_CHECK_K = LF_RCK;         // ...and the rate freed when they disagree by this many sigmas
static const double NEAR_EXT = 2.0;                // readings closer than this many turn hysteresis widths to the last extreme count as "near it"
static const double OUTLIER_K = 1.5;               // a lone reading this many half-spans from the centre is a glitch...
static const double GLITCH_SIGMAS = 3.0;           // ...as is one this many sigmas from where the model expects it
static const double GLITCH_RATE_TOL = 0.05;        // the rate may be off by this fraction between two readings
static const double BAND_K = 1.15;                 // readings beyond this many half-spans from the centre are out of band...
static const double BAND_TAU = 60.0;               // ...and when they have been for most of this long (s) the calibration is dead
static const double BAND_RECAL = 0.95;             // recalibrate once the new swing covers this much of the old one
static const float NOISE_TAU = 50;              // s over which the raw noise is averaged
static const float NOISE_INIT = 0.3f;           // starting guess for the raw noise (uT)
static const float MIN_SAMPLES_NOISE = 200;     // noise measured only when a cycle has this many samples
static const float START_S = 20;                // seconds of input used to pick the starting branch
static const double CYCLE_AGREE = 0.08;         // last-cycle time is used while it agrees with the filter
static const int RING_PER_CYCLE = 32;           // (t, phase) points kept per cycle for that
static const int RING = 200;            // covers a pilot cycle at a point every 5 s
// shape learning (as before)
static const float STEADY_TOL = 0.03f;
static const float BIN_FILL = 1.0f;
static const double LEARN_TAU_S = LF_LEARN_TAU;
static const float LEARN_SMOOTHINGS = LF_LEARN_SMOOTHINGS;
static const float SHAPE_BIG = 0.25f;
static const int REC_MAX = 2500;
static const uint32_t MAGIC = 0x4C46310Bu;   // same record as the regression tracker: the calibration and learned shape carry over either way
static const double SAVE_EVERY_S = 1800;

struct Learned {
  uint32_t magic;
  float offset, amplitude;
  float shape[N];
  uint32_t shape_updates;
};

class Tracker {
 public:
  Tracker() { seed_(); }

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
  float take_max_update_ms() { float v = max_us_ / 1000.0f; max_us_ = 0; return v; }

  bool ready() const { return started_; }
  double cycles_per_s() const { return r_out_; }
  double phase() const { return phi_; }
  float offset() const { return (float) off_; }
  float amplitude() const { return (float) amp_; }
  uint32_t shape_updates() const { return updates_; }
  int last_learn() const { return last_learn_; }
  enum LearnResult { NONE = 0, ACCEPTED = 1, NOT_STEADY = 2, OUT_OF_RANGE = 3, DIFFERENT_WAITING = 4, ACCEPTED_NEW_SHAPE = 5, FIRST_CYCLE = 6, COLLECTING = 7, RECALIBRATED = 8 };
  float noise() const { return std::sqrt((float) nvar_); }
#ifdef LF_DEBUG
  double dbg_rm = 0, dbg_sr = 0; int dbg_rc = 0;
  bool dbg_was_flat = false, dbg_flat_exit = false; double dbg_flat_t0 = 0, dbg_flat_phi0 = 0, dbg_hi_over = 0; int dbg_flat_wind = 0, dbg_flat_n = 0;
  double dbg_nis = 0; int dbg_reacq = 0; double dbg_sig_r = 0; int dbg_pm = 0; int dbg_rising = 0; double dbg_ext = 0, dbg_H = 0, dbg_zx = 0;
#endif

  void recalibrate(float zmin, float zmax, bool new_axis, float span_ratio) {
    if (!loaded_) load_();
    if (!(zmax - zmin >= 2.0f) || !std::isfinite(zmin) || !std::isfinite(zmax)) { last_learn_ = OUT_OF_RANGE; return; }   // not a usable span
    if (new_axis) seed_();
    amp_ = span_ratio * (zmax - zmin) / (smax_ - smin_);
    off_ = (zmax + zmin) / 2 - amp_ * (smax_ + smin_) / 2;
    restart_();
    last_learn_ = RECALIBRATED;
    dirty_ = true; save_now_ = true;
  }
  void reset() {
    if (!loaded_) load_();
    // Drop the learned shape but keep the calibration: the Z range the current
    // shape maps to is remapped onto the seed shape.
    float mx = shape_[0], mn = shape_[0];
    for (int i = 1; i < N; i++) { if (shape_[i] > mx) mx = shape_[i]; if (shape_[i] < mn) mn = shape_[i]; }
    float zmax = (float) (off_ + amp_ * mx), zmin = (float) (off_ + amp_ * mn);
    seed_();
    amp_ = (zmax - zmin) / (smax_ - smin_);
    off_ = (zmax + zmin) / 2 - amp_ * (smax_ + smin_) / 2;
    restart_();
    dirty_ = true; last_save_ = -1e9;
#ifdef LIVE_FLOW_PERSIST
    save_(0);
#endif
  }

 private:
  int64_t max_us_ = 0;
  // ---- shape model ---------------------------------------------------------
  float shape_[N];
  double off_ = SEED_OFFSET, amp_ = SEED_AMPLITUDE;
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
  // slope of the shape per cycle, smoothed over about a bin either side
  float dsh_(double p) const { return (sh_(p + 1.0 / N) - sh_(p - 1.0 / N)) * (N / 2.0f); }
  void build_() {
    int imax = 0, imin = 0;
    for (int i = 1; i < N; i++) {
      if (shape_[i] > shape_[imax]) imax = i;
      if (shape_[i] < shape_[imin]) imin = i;
    }
    pmax_ = (double) imax / N; pmin_ = (double) imin / N;
    smax_ = shape_[imax]; smin_ = shape_[imin];
    double pmin_r = pmin_ < pmax_ ? pmin_ + 1 : pmin_;
    double pmax_r = pmax_ < pmin_ ? pmax_ + 1 : pmax_;
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
  // Phase offsets (cycles) before and after a turn within which the field
  // stays within H of the extreme, from the current shape.
  void flat_span_(bool peak, double H, double &da, double &db) const {
    int i0 = 0;
    for (int i = 1; i < N; i++) if (peak ? shape_[i] > shape_[i0] : shape_[i] < shape_[i0]) i0 = i;
    float thr = (float) (H / amp_);
    int a = 1, b = 1;
    while (a < N / 2 && std::fabs(shape_[(i0 - a + N) % N] - shape_[i0]) < thr) a++;
    while (b < N / 2 && std::fabs(shape_[(i0 + b) % N] - shape_[i0]) < thr) b++;
    da = -(double) a / N; db = (double) b / N;
  }
  // Position within the cycle for field z on the given branch (exact inverse).
  double phase_(float z, bool rising) const {
    float u = (float) ((z - off_) / amp_);
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

  // ---- filter state ----------------------------------------------------------
  bool started_ = false;
  double phi_ = 0, r_ = 0, r_out_ = 0;
  // covariance of (phi, r, off, amp), symmetric
  double P_[4][4];
  double tp_ = 0, dt_avg_ = 0.5;
  double nvar_ = NOISE_INIT * NOISE_INIT, z1_ = 0, z2_ = 0;
  int n2_ = 0;
  int in_run_ = 10;       // consecutive readings in line with the model (saturates at 10)
  double out_frac_ = 0, wmin_ = 0, wmax_ = 0; bool lost_ = false, wlast_max_ = false;   // out-of-band watchdog
  double off_ref_ = SEED_OFFSET, amp_ref_ = SEED_AMPLITUDE;         // calibration as of the last believed turn
  double nis_ema_ = 0, t_inf_ = -1e9, p_inf_ = 0, t_gate_ = -1e9, pos_R_ = 1e-4;
  int n0_ = 0; float z0_ = 0; double t0_ = 0;
  void restart_() {
    off_ref_ = off_; amp_ref_ = amp_;
    started_ = false; n0_ = 0; r_ = 0; r_out_ = 0; rec_on_ = false; rn_ = 0;
    prev_period_ = 0; prev_split_ = 0; diff_count_ = 0; acc_reset_();
  }
  void init_cov_() {
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) P_[i][j] = 0;
    P_[0][0] = 0.01 * 0.01; P_[1][1] = 1e-3 * 1e-3; P_[2][2] = 1.0; P_[3][3] = 0.5 * 0.5;   // a calibration is good to ~1 uT
  }
  // Keep the covariance a covariance: non-negative variances and no
  // correlation beyond what the variances allow.
  void fix_cov_() {
    for (int i = 0; i < 4; i++) if (!(P_[i][i] > 1e-12)) P_[i][i] = 1e-12;
    for (int i = 0; i < 4; i++) for (int j = i + 1; j < 4; j++) {
      double lim = 0.999 * std::sqrt(P_[i][i] * P_[j][j]);
      if (P_[i][j] > lim) P_[i][j] = lim; else if (P_[i][j] < -lim) P_[i][j] = -lim;
      P_[j][i] = P_[i][j];
    }
  }
  bool start_(double t, float z) {
    if (n0_ == 0 || t < t0_ || t - t0_ > 10 * START_S) { z0_ = z; t0_ = t; n0_ = 0; }   // time jumped: start the window over
    ++n0_;
    if (t - t0_ < START_S || n0_ < 3) return false;
    dt_avg_ = (t - t0_) / (n0_ - 1);
    rising_ = z > z0_; ext_ = ext0_ = ext4_ = z; rev_n_ = 0; mn_ = 0; mi_ = 0; m_cur_ = 1;
    phi_ = phase_(z, rising_);
    r_ = 0; init_cov_();
    P_[1][1] = 1e-3 * 1e-3;   // rate unknown but small: it will be measured in seconds
    tp_ = t; started_ = true; rn_ = 0; nis_ema_ = 0;
    t_inf_ = -1e9; p_inf_ = 0; dz_avg_ = 0; n2_ = 0; in_run_ = 10; have_turn_ = false; prev_disagree_ = false; out_n_ = 0; have_pk_ = have_tr_ = false; out_frac_ = 0; lost_ = false;
    ring_push_(t);
    last_cross_ = std::floor(phi_);
    return false;
  }

  bool update_(double t, float z) {
    if (!loaded_) load_();
    if (!std::isfinite(z) || !std::isfinite(t)) return started_;   // a bad reading changes nothing
    if (!started_) return start_(t, z);
    // A lone reading far from where the model expects the field (or outside
    // the range the magnet can produce at all) is a glitch, a bad I2C read
    // say, and is dropped: acting on it would throw the phase. If the next
    // reading is out there too the field really has moved, and from then on
    // readings are taken as they come so the filter can re-acquire (a big
    // flow step is seen one reading late) or the watchdog below can act.
    {
      double half = amp_ * (smax_ - smin_) / 2, mid = off_ + amp_ * (smax_ + smin_) / 2;
      double dtp = std::max(0.0, std::min(t - tp_, 30 * dt_avg_)), pp = phi_ + r_ * dtp;
      double zp = off_ + amp_ * sh_(pp), sl = amp_ * dsh_(pp);
      // what one reading may differ from the prediction by: its own noise and
      // shape tolerance, and the prediction's phase uncertainty seen through
      // the local slope (steep: a small field error is a big phase error)
      double p00 = P_[0][0] + dtp * (2 * P_[0][1] + dtp * P_[1][1]);
      // ...and near a turn the slope says nothing about how far the field moves
      // for a phase error, so the tolerance covers the field's change over
      // that phase error directly
      double dph = std::sqrt(p00) + GLITCH_RATE_TOL * r_ * dt_avg_;
      double curve = std::fabs(sh_(pp + dph) - sh_(pp - dph)) * amp_ / 2;
      double tol = GLITCH_SIGMAS * std::sqrt(nvar_ + (SHAPE_TOL * amp_) * (SHAPE_TOL * amp_) + p00 * sl * sl + curve * curve);
      if (std::fabs(z - mid) > OUTLIER_K * half || std::fabs(z - zp) > tol) {
        if (in_run_ >= 10) { in_run_ = 0; return true; }   // the first odd reading after a settled stretch
        in_run_ = 0;
      } else if (in_run_ < 10) {
        in_run_++;
      }
    }
    double dt = t - tp_;
    tp_ = t;
    if (dt <= 0) return true;
    if (dt > 30 * dt_avg_) dt = 30 * dt_avg_;
    dt_avg_ += 0.01 * (dt - dt_avg_);
    // Readings that mostly sit outside the band the calibration allows mean
    // the sensor or the magnet has moved: the offset and amplitude states can
    // follow a shift within the band, but not one that takes the whole swing
    // elsewhere. Then the calibration is dead, the flow reads zero, and the
    // new extremes are collected until a full swing has been seen, which
    // recalibrates exactly as the button would.
    {
      // judged against the calibration as of the last believed turn, which a
      // runaway of the level states after the move cannot have touched
      double half = amp_ref_ * (smax_ - smin_) / 2, mid = off_ref_ + amp_ref_ * (smax_ + smin_) / 2;
      bool out = std::fabs(z - mid) > BAND_K * half;
      out_frac_ += std::min(1.0, dt / BAND_TAU) * ((out ? 1.0 : 0.0) - out_frac_);
      if (!lost_ && out_frac_ > 0.5) {
        lost_ = true; wmin_ = wmax_ = z;
#ifdef LF_DEBUG
        fprintf(stderr, "t=%.0f LOST z=%.2f ref off=%.2f amp=%.2f band %.2f..%.2f\n", t, z, off_ref_, amp_ref_, mid - BAND_K * half, mid + BAND_K * half);
#endif
      }
      if (lost_) {
        if (z < wmin_) { wmin_ = z; wlast_max_ = false; }
        if (z > wmax_) { wmax_ = z; wlast_max_ = true; }
        r_ = 0; r_out_ = 0;
        // a swing as big as the old one, and the field has turned back from
        // the extreme it reached last, so both ends are real turning points
        double swing = wmax_ - wmin_;
        bool turned_back = wlast_max_ ? z < wmax_ - 0.1 * swing : z > wmin_ + 0.1 * swing;
        if (swing > BAND_RECAL * 2 * half && turned_back) {
#ifdef LF_DEBUG
          fprintf(stderr, "t=%.0f AUTO-RECAL %.2f..%.2f (old half %.2f)\n", t, wmin_, wmax_, half);
#endif
          recalibrate((float) wmin_, (float) wmax_, false, 1.0f);
          lost_ = false; out_frac_ = 0;
        }
        return true;
      }
    }
    // Raw noise from second differences, only while the field barely moves
    // between samples (judged from the samples themselves, not from the rate
    // estimate, which lags a flow step).
    if (n2_ >= 1) dz_avg_ += 0.3 * (std::fabs(z - z1_) - dz_avg_);   // mean size of a sample step
    if (r_known_ && n2_ >= 2 && (dz_avg_ < 4.0 * std::sqrt(nvar_) || r_ * dt_avg_ < 0.005)) {
      double d2 = z - 2 * z1_ + z2_;
      double v = d2 * d2 / 6.0;
      if (v > 25 * nvar_) v = 25 * nvar_;   // one wild sample cannot blow it up
      nvar_ += std::min(1.0, dt_avg_ / NOISE_TAU) * (v - nvar_);
      if (nvar_ < 4e-4) nvar_ = 4e-4;   // 0.02 uT: quieter than the sensor gets
    }
    z2_ = z1_; z1_ = z; if (n2_ < 2) n2_++;
    double sig = std::sqrt(nvar_ + (SHAPE_TOL * amp_) * (SHAPE_TOL * amp_));  // one reading's uncertainty, uT

    // ---- which way the field is going (rising/falling branch of the shape):
    // a moving average over ~1/40 cycle with hysteresis of a few noise sigmas,
    // and a turn is only believed near the top (peak) or bottom (trough).
    double period = r_ > 1e-6 ? 1.0 / r_ : 1e9;
    int m = (int) std::min((double) MA, period / 40.0 / dt_avg_);
    // ...and never so long that the field moves more than the hysteresis
    // within it: right after a flow step the rate estimate has not caught
    // up, and the detector would otherwise confirm the turn seconds late.
    if (dz_avg_ > 1e-6) { int ms = (int) (H_prev_ / dz_avg_); if (ms < m) m = ms; }
    if (m < 1) m = 1;
    if (m > MA) m = MA;
    if (m < m_cur_) m_cur_ = m; else if (m > m_cur_) m_cur_++;
    ma_[mi_] = z; mi_ = (mi_ + 1) % MA; if (mn_ < MA) mn_++;
    int mm = std::min(m_cur_, mn_);
    double zx = 0;
    for (int k = 1; k <= mm; k++) zx += ma_[(mi_ - k + MA) % MA];
    zx /= mm;
    double H = std::max(5.0 * std::sqrt(nvar_ / mm), 3.0 * SHAPE_TOL * amp_);
    if (H > 0.15 * amp_ * (smax_ - smin_)) H = 0.15 * amp_ * (smax_ - smin_);
    H_prev_ = H;
    double umid = off_ + amp_ * (smax_ + smin_) / 2;
#ifdef LF_DEBUG
    dbg_H = H; dbg_zx = zx;
#endif
    // A reversal of half the swing is a turn wherever the calibrated middle
    // says it is: after the sensor is moved the middle can end up on the
    // wrong side of every trough, and without this the detector would never
    // see a turn again and the level would never be corrected.
    // That reversal is judged on a short fixed average with its own running
    // extreme, so that one glitched reading (which the adaptive average above
    // may consist of entirely) cannot fake it.
    double big = 0.5 * amp_ * (smax_ - smin_) / 2;
    int m4 = std::min(4, mn_);
    double zx4 = 0;
    for (int k = 1; k <= m4; k++) zx4 += ma_[(mi_ - k + MA) % MA];
    zx4 /= m4;
    bool turned = false; double turn_val = 0; bool turn_peak = false;
    if (rising_) {
      if (zx > ext_) ext_ = zx;
      if (zx4 > ext4_) ext4_ = zx4;
      rev_n_ = zx4 < ext4_ - big ? rev_n_ + 1 : 0;
      if (zx < ext_ - H && (ext_ > umid || rev_n_ >= 2)) { turned = true; turn_val = ext_; turn_peak = true; rising_ = false; }
    } else {
      if (zx < ext_) ext_ = zx;
      if (zx4 < ext4_) ext4_ = zx4;
      rev_n_ = zx4 > ext4_ + big ? rev_n_ + 1 : 0;
      if (zx > ext_ + H && (ext_ < umid || rev_n_ >= 2)) { turned = true; turn_val = ext_; turn_peak = false; rising_ = true; }
    }
    // The extreme is only a measured one if the field travelled to it: after
    // a (re)start the branch is guessed from the first readings, and if the
    // guess was wrong the first "turn" is just the starting level.
    bool real_ext = turned && std::fabs(ext_ - ext0_) > H;
    if (turned) {
      if (turn_peak) { last_pk_ = turn_val; have_pk_ = true; } else { last_tr_ = turn_val; have_tr_ = true; }
      ext_ = ext0_ = zx; ext4_ = zx4; rev_n_ = 0; have_turn_ = true;
      // the reference calibration follows the level only in small steps, one
      // per turn: a runaway of the level states after the sensor moved must
      // not drag the out-of-band watchdog's reference along with it
      if (std::fabs(amp_ / amp_ref_ - 1) < 0.1 && std::fabs(off_ - off_ref_) < 0.1 * amp_ref_) { off_ref_ = off_; amp_ref_ = amp_; }
    }

    // ---- predict: phi += r dt; the rate may wander in proportion to itself ----
    phi_ += r_ * dt;
    double p00 = P_[0][0] + dt * (P_[0][1] + P_[1][0]) + dt * dt * P_[1][1];
    double p01 = P_[0][1] + dt * P_[1][1];
    double p02 = P_[0][2] + dt * P_[1][2], p03 = P_[0][3] + dt * P_[1][3];
    P_[0][0] = p00; P_[0][1] = P_[1][0] = p01; P_[0][2] = P_[2][0] = p02; P_[0][3] = P_[3][0] = p03;
    double qr = RATE_WALK * r_ + RATE_FLOOR;
    P_[1][1] += qr * qr * dt;
    P_[0][0] += PHASE_WALK * PHASE_WALK * dt;
    P_[2][2] += DRIFT_WALK * DRIFT_WALK * dt;
    P_[3][3] += DRIFT_WALK * DRIFT_WALK * dt;

    // ---- measure ----
    // Three cases. In the flat stretch around the turn the branch is heading
    // to, the field says only that the meter is somewhere in that stretch, and
    // measures its own level (offset/amplitude). Elsewhere, if the predicted
    // position is on the same branch as the field's direction, the shape's
    // exact inverse reads the position. Otherwise (just past a turn the
    // detector has not confirmed) nothing is known and the filter coasts.
    double zmax = off_ + amp_ * smax_, zmin = off_ + amp_ * smin_;
    // "Turning": already heading back from a peak/trough the detector has not
    // confirmed yet (it needs a full hysteresis); the field is not readable
    // on either branch then.
    bool turning = rising_ ? (ext_ > umid && ext_ - zx > 0.5 * H) : (ext_ < umid && zx - ext_ > 0.5 * H);
    bool flat = (rising_ ? (z > zmax - H) : (z < zmin + H)) || turning;
#ifdef LF_DEBUG
    if (flat && !dbg_was_flat) { dbg_flat_t0 = t; dbg_flat_phi0 = phi_; dbg_flat_wind = 0; dbg_flat_n = 0; }
    if (flat) dbg_flat_n++;
    dbg_flat_exit = !flat && dbg_was_flat;
    dbg_was_flat = flat;
#endif
    int n_meas = 0;
    double Hm[3][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, nu[3] = {0, 0, 0}, Rm[3] = {1, 1, 1};
    bool gated = false, near_ext = false;
    if (turned && real_ext && period <= 40 * dt_avg_ && period >= 16 * dt_avg_ && have_pk_ && have_tr_) {
      // A fast cycle: each reading averages half a second of field, which
      // pulls both extremes toward the middle, so neither measures the
      // amplitude - but their midpoint still measures the offset, and the
      // magnet's field drifts as gas flows (it cools), so that matters over
      // a long burn. Not on very fast cycles, where the flattening is large
      // and the two extremes (of different widths) no longer flatten alike.
      Hm[0][2] = 1; nu[0] = 0.5 * (last_pk_ + last_tr_) - (off_ + amp_ * 0.5 * (smax_ + smin_));
      Rm[0] = nvar_ + (0.02 * amp_) * (0.02 * amp_);
      n_meas = 1;
    }
    if (turned && real_ext && period > 40 * dt_avg_) {
      // A confirmed turn of a cycle with enough samples to catch the extreme:
      // its extreme (the peak or trough of the smoothed field) measures the
      // level there, wherever the model thought it was. An extreme more than
      // an amplitude off is not a level shift the model can absorb (the
      // out-of-band watchdog deals with those) and is left alone.
      double sext = turn_peak ? smax_ : smin_;
      double nu0 = turn_val - (off_ + amp_ * sext);
      if (std::fabs(nu0) < amp_) {
        Hm[0][2] = 1; Hm[0][3] = sext; nu[0] = nu0;
        Rm[0] = nvar_ / std::max(1, m_cur_) + (0.01 * amp_) * (0.01 * amp_);   // noise of the average, and its rounding of the extreme
        n_meas = 1;
      }
    }
    if (flat) {
      double da, db;
      flat_span_(rising_, H, da, db);
      double lim = rising_ ? pmax_ : pmin_;
      lim += std::floor(phi_ - lim);
      if (lim < phi_ - 0.5) lim += 1;
      // The meter is somewhere in the flat stretch: inside it the field says
      // nothing more (the phase coasts), and a prediction that has run out of
      // it is held back to its edge (the meter cannot have left: the field
      // would show it). The level of the field measures offset/amplitude.
      // The stretch's edges are only known to the shape tolerance (the real
      // bottom was 20% longer than the shape's in the logs), so the edges are
      // padded before a prediction is held back by them.
      double pad = EDGE_PAD * (db - da);
      double lo = lim + da - pad, hi = lim + db + pad, sf = (db - da) / std::sqrt(12.0);
      // The level of the field measures offset/amplitude, from the readings
      // nearest the extreme (the whole band lies to one side of it, so its
      // average would read the extreme a third of a band too low).
      double sext = rising_ ? smax_ : smin_;
      double zext = off_ + amp_ * sext;
      if (std::fabs(z - zext) < H / 3 || (rising_ ? z > zext : z < zext)) {
        Hm[n_meas][2] = 1; Hm[n_meas][3] = sext; nu[n_meas] = z - zext; Rm[n_meas] = sig * sig + H * H / 9;
        n_meas++;
      }
#ifdef LF_DEBUG
      dbg_hi_over = phi_ - hi;
#endif
      if (phi_ > hi || phi_ < lo) {
        Hm[n_meas][0] = 1; nu[n_meas] = (phi_ > hi ? hi : lo) - phi_; Rm[n_meas] = sf * sf;
        n_meas++;
      }
      // Once the prediction has reached the far edge, the meter has been in
      // the stretch longer than the rate allows: since the last readable
      // position it has not left, so the rate is anywhere from zero up to
      // what would just reach the edge now. Report the middle of that range
      // (a flow that stopped in here is then seen to stop within seconds).
      if (phi_ > hi && t > t_inf_ && hi > p_inf_) {
        double r_mid = 0.5 * (hi - p_inf_) / (t - t_inf_);
        if (r_ > r_mid) {
#ifdef LF_DEBUG
          dbg_flat_wind++;
#endif
          r_ = r_mid;
          if (P_[1][1] > r_mid * r_mid) {   // shrink the rate's covariance with its cross-term, or P goes indefinite
            double sc = r_mid * r_mid / P_[1][1];
            P_[1][1] *= sc; P_[0][1] = P_[1][0] = P_[0][1] * std::sqrt(sc);
          }
        }
      }
    } else {
      // The shape's exact inverse on the branch the field is moving along,
      // unwrapped to the nearest cycle of the prediction (also when the
      // prediction sits on the other branch: then the residual is large and
      // the gate below re-acquires).
      double y = phase_(z, rising_);
      y += std::floor(phi_ - y);
      if (y < phi_ - 0.5) y += 1;
      // A reading still close to the extreme just passed sits where the wave
      // is nearly flat: its position is poorly determined and easily biased
      // by a little shape error. One such reading on its own is not evidence
      // of a flow change (at fast flow the first reading after every turn is
      // one), so it may only trip the gate if the last well-determined reading
      // already disagreed. A meter that stopped or stepped keeps disagreeing.
      // Only at fast flow, judged by the field's step between readings: at
      // pilot speed a reading near the extreme moves the prediction little.
      near_ext = dz_avg_ > 0.5 * H && (rising_ ? (z < zmin + NEAR_EXT * H) : (z > zmax - NEAR_EXT * H));
      gated = true;
      double sl = amp_ * dsh_(y);
      if (std::fabs(sl) < 1e-3) sl = 1e-3;
      int q = n_meas;
      Hm[q][0] = 1; nu[q] = y - phi_; Rm[q] = (sig / sl) * (sig / sl);
      n_meas++;
#ifdef LF_DEBUG
      if (dbg_flat_exit) fprintf(stderr, "t=%.1f FLATEXIT dur=%.1f n=%d dphi=%.4f wind=%d hi_over=%.4f nu=%.4f sigma=%.4f r=%.4f\n",
                                 t, t - dbg_flat_t0, dbg_flat_n, phi_ - dbg_flat_phi0, dbg_flat_wind, dbg_hi_over, nu[q], std::sqrt(Rm[q] + P_[0][0]), r_);
#endif
      // The detector confirms a turn only once the field has moved a
      // hysteresis past it; until then a reading just past the turn comes
      // out behind the prediction, on the old branch. While the prediction
      // is only a little past the turn that is all it can be, and the filter
      // coasts. A stopped meter's prediction leaves that zone within seconds,
      // after which a reading behind the prediction is what it looks like.
      if (nu[q] < -3.0 * std::sqrt(Rm[q]) && nu[q] < -0.01) {
        double pf = phi_ - std::floor(phi_);
        double past = pf - (rising_ ? pmax_ : pmin_);
        past -= std::floor(past);
        if (past < TURN_ZONE) n_meas = q;
      }
      if (n_meas > q) { t_inf_ = t; p_inf_ = phi_; pos_R_ = Rm[q]; }   // the last readable position
    }
#ifdef LF_DEBUG
    dbg_reacq = 0; dbg_pm = flat ? 2 : 1; dbg_nis = 0; dbg_rising = rising_; dbg_ext = ext_;
#endif
    for (int q = 0; q < n_meas; q++) {
      double *Hq = Hm[q];
      double PH[4];
      for (int i = 0; i < 4; i++) PH[i] = P_[i][0] * Hq[0] + P_[i][2] * Hq[2] + P_[i][3] * Hq[3];
      double S = Hq[0] * PH[0] + Hq[2] * PH[2] + Hq[3] * PH[3] + Rm[q];
      if (!(S >= Rm[q])) { init_cov_(); fix_cov_(); continue; }   // a broken covariance: start it over rather than use it
      double nis = nu[q] * nu[q] / S;
      if (gated && Hq[0] == 1) {
#ifdef LF_DEBUG
        dbg_nis = nis;
#endif
        if (!near_ext) { nis_ema_ += GATE_ALPHA * (nis - nis_ema_); prev_disagree_ = nis > GATE; }   // only a well-determined reading counts as evidence
        bool fire = nis_ema_ > GATE && (!near_ext || prev_disagree_);
        if (fire) {
          // The prediction does not explain the position: the flow changed.
          // Let the rate move freely; if the position is far off, take it.
          double sig_r = std::sqrt(P_[1][1]);
          double need = std::max(std::fabs(nu[q]) / dt_avg_, 0.3 * r_);   // the rate change that would explain it
          if (sig_r < need) { P_[1][1] = need * need; P_[0][1] = P_[1][0] = 0; }
          if (nis > REACQ_SIGMAS * REACQ_SIGMAS && nu[q] > 0) {
            phi_ += nu[q]; nu[q] = 0;
            P_[0][0] = Rm[q]; P_[0][1] = P_[1][0] = 0; P_[0][2] = P_[2][0] = 0; P_[0][3] = P_[3][0] = 0;
#ifdef LF_DEBUG
            dbg_reacq = 1;
#endif
          }
          for (int i = 0; i < 4; i++) PH[i] = P_[i][0] * Hq[0] + P_[i][2] * Hq[2] + P_[i][3] * Hq[3];
          S = Hq[0] * PH[0] + Hq[2] * PH[2] + Hq[3] * PH[3] + Rm[q];
          nis_ema_ = 0;
          t_gate_ = t;
        }
      }
      double K[4];
      for (int i = 0; i < 4; i++) K[i] = PH[i] / S;
      phi_ += K[0] * nu[q]; r_ += K[1] * nu[q]; off_ += K[2] * nu[q]; amp_ += K[3] * nu[q];
      if (r_ < 0) r_ = 0;
      if (r_ > 0.5 / dt_avg_) r_ = 0.5 / dt_avg_;   // more than half a cycle per reading cannot be told from less
      if (amp_ < 1.0) amp_ = 1.0;   // a field this weak is unusable anyway; never let the model collapse
      for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) P_[i][j] -= K[i] * PH[j];
      for (int i = 0; i < 4; i++) for (int j = i + 1; j < 4; j++) P_[i][j] = P_[j][i] = 0.5 * (P_[i][j] + P_[j][i]);
      fix_cov_();
    }
    r_known_ = true;
    // A slow change (the pilot going out: a step of 0.0007 cycles per sample)
    // is absorbed sample by sample by the phase and never trips the gate.
    // Over a longer stretch it is plain: the rate the measured positions
    // advanced at, against the filter's rate. Only a readable position is a
    // measured one (in a flat stretch the phase is a prediction).
    if (!flat && n_meas > 0 && t - t_gate_ > RATE_CHECK_S) {
      double ta, pa;
      if (ring_back_(t, RATE_CHECK_S, ta, pa) && ta > t_gate_) {
        double rm = (phi_ - pa) / (t - ta);
        double sr = 1.5 * std::sqrt(pos_R_ + P_[0][0]) / (t - ta);   // uncertainty of that, from the positions'
#ifdef LF_DEBUG
        dbg_rm = rm; dbg_sr = sr; dbg_rc = 0;
#endif
        if (std::fabs(rm - r_) > RATE_CHECK_K * sr) {
#ifdef LF_DEBUG
          dbg_rc = 1;
#endif
          // Take the measured rate, with its uncertainty.
          r_ = rm > 0 ? rm : 0;
          P_[1][1] = sr * sr; P_[0][1] = P_[1][0] = 0; P_[1][2] = P_[2][1] = 0; P_[1][3] = P_[3][1] = 0;
          t_gate_ = t;
        }
      }
    }
#ifdef LF_DEBUG
    dbg_sig_r = std::sqrt(P_[1][1]);
#endif
    // ---- output: the filter's rate, or the time of the last full cycle when
    // the two agree (the shape's small repeating errors cancel over a cycle) ----
    ring_push_(t, !flat && n_meas > 0);
    double ro = r_;
    double tc;
    if (cycle_ago_(t, tc)) {
      double rc = 1.0 / (t - tc);
      if (r_ > 0 && std::fabs(rc / r_ - 1) < CYCLE_AGREE) ro = rc;
    }
    // The median of the last three: one reading's excursion never reaches
    // the output, a real change shows one reading later.
    out2_ = out1_; out1_ = out0_; out0_ = ro;
    if (out_n_ < 3) out_n_++;
    r_out_ = out_n_ < 3 ? ro : std::max(std::min(out0_, out1_), std::min(std::max(out0_, out1_), out2_));

    // ---- shape learning, timed by the phase crossing whole cycles ----
    rec_sample_(t, z);
    double fl = std::floor(phi_);
    if (fl > last_cross_) {
      double tcr = t - (phi_ - fl) / (r_ > 1e-9 ? r_ : 1e-9);
      if (tcr < t - dt) tcr = t - dt;
      on_cross_(tcr);
      last_cross_ = fl;
    }
    if (dirty_ && (save_now_ || (t - last_save_) >= SAVE_EVERY_S)) { save_(t); save_now_ = false; }
    return true;
  }
  static const int MA = 64;
  double ma_[MA]; int mi_ = 0, mn_ = 0, m_cur_ = 1;
  bool rising_ = true; double ext_ = 0, ext0_ = 0, ext4_ = 0, H_prev_ = 1.0, dz_avg_ = 0; int rev_n_ = 0;
  bool have_turn_ = false;   // a turn has been believed since the (re)start
  double last_pk_ = 0, last_tr_ = 0; bool have_pk_ = false, have_tr_ = false;   // the last believed peak and trough
  double out0_ = 0, out1_ = 0, out2_ = 0; int out_n_ = 0;   // the last three output candidates
  bool prev_disagree_ = false;   // the previous position reading disagreed with the prediction
  bool r_known_ = false;

  // ---- (t, phase) ring for the last-cycle time ----
  double ring_t_[RING], ring_p_[RING]; bool ring_m_[RING];
  int ri_ = 0, rn_ = 0;
  double ring_last_p_ = -1e9, ring_last_t_ = -1e9;
  void ring_push_(double t, bool measured = false) {
    // a point every 1/32 cycle, and at least every few seconds (a stopped
    // meter adds no phase, but its standstill must still be on record)
    if (rn_ > 0 && phi_ - ring_last_p_ < 1.0 / RING_PER_CYCLE && t - ring_last_t_ < RATE_CHECK_S / 4) return;
    ring_t_[ri_] = t; ring_p_[ri_] = phi_; ring_m_[ri_] = measured; ring_last_p_ = phi_; ring_last_t_ = t;
    ri_ = (ri_ + 1) % RING; if (rn_ < RING) rn_++;
  }
  // The newest ring entry from a measured position at least `back` seconds old.
  bool ring_back_(double t, double back, double &ta, double &pa) const {
    for (int k = 1; k < rn_; k++) {
      int i = (ri_ - 1 - k + RING) % RING;
      if (t - ring_t_[i] > 4 * back) break;
      if (t - ring_t_[i] >= back && ring_m_[i]) { ta = ring_t_[i]; pa = ring_p_[i]; return true; }
    }
    return false;
  }
  bool cycle_ago_(double t, double &tc) const {
    double target = phi_ - 1.0;
    for (int k = 1; k < rn_; k++) {
      int ib = (ri_ - k + RING) % RING, ia = (ri_ - k - 1 + RING) % RING;
      if (ring_p_[ia] <= target && ring_p_[ib] >= target) {
        double f = (ring_p_[ib] > ring_p_[ia]) ? (target - ring_p_[ia]) / (ring_p_[ib] - ring_p_[ia]) : 1;
        tc = ring_t_[ia] + f * (ring_t_[ib] - ring_t_[ia]);
        return t - tc > 0;
      }
      if (ring_p_[ia] < target) break;
    }
    (void) t;
    return false;
  }

  // ---- shape learning (recording and folding as before) ----
  bool rec_on_ = false;
  double rec_t0_ = 0, rec_half_ = -1, last_cross_ = 0;
  int rec_n_ = 0, rec_step_ = 1, rec_skip_ = 0;
  double rec_sz_ = 0, rec_st_ = 0;
  float rec_t_[REC_MAX], rec_z_[REC_MAX];
  float prev_period_ = 0, prev_split_ = 0;
  int diff_count_ = 0, last_learn_ = NONE;
  float acc_sum_[N], acc_cnt_[N];
  int acc_cycles_ = 0;
  double acc_span_ = 0;
  void acc_reset_() { for (int i = 0; i < N; i++) { acc_sum_[i] = 0; acc_cnt_[i] = 0; } acc_cycles_ = 0; acc_span_ = 0; }
  void rec_sample_(double t, float z) {
    if (!rec_on_) return;
    if (rec_half_ < 0 && phi_ - std::floor(phi_) >= 0.5 && phi_ - last_cross_ >= 0.5) rec_half_ = t - rec_t0_;
    rec_sz_ += z; rec_st_ += t - rec_t0_;
    if (++rec_skip_ >= rec_step_) {
      if (rec_n_ == REC_MAX) {
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
  void on_cross_(double tc) {
    if (rec_on_) finish_cycle_(tc);
    rec_on_ = true; rec_t0_ = tc; rec_n_ = 0; rec_half_ = -1; rec_step_ = 1; rec_skip_ = 0; rec_sz_ = 0; rec_st_ = 0;
  }
  void finish_cycle_(double tc) {
    float period = (float) (tc - rec_t0_);
    float split = rec_half_ > 0 ? (float) rec_half_ / period : -1;
    float tol = std::max(STEADY_TOL, (float) (2 * dt_avg_ / period));
    bool steady = prev_period_ > 0 && split > 0 && prev_split_ > 0 &&
                  std::fabs(period / prev_period_ - 1) < tol &&
                  std::fabs(split - prev_split_) < tol;
    bool range = rec_n_ >= 8;
    bool first = prev_period_ <= 0;
    prev_period_ = period; prev_split_ = split;
#ifdef LF_DEBUG
    fprintf(stderr, "t=%.0f CYCLE period=%.1f split=%.3f steady=%d n=%d acc=%d\n", tc, period, split, steady, rec_n_, acc_cycles_);
#endif
    if (period < LEARN_SMOOTHINGS * 0.5 * dt_avg_) range = false;
    if (!range) { acc_reset_(); diff_count_ = 0; last_learn_ = OUT_OF_RANGE; return; }
    if (first) { acc_reset_(); diff_count_ = 0; last_learn_ = FIRST_CYCLE; return; }
    if (!steady) { acc_reset_(); diff_count_ = 0; last_learn_ = NOT_STEADY; return; }
    for (int k = 0; k < rec_n_; k++) {
      float x = rec_t_[k] / period * N;
      if (x >= N && x < 2 * N) x -= N;
      if (x < 0 || x >= N) continue;
      int i0 = (int) x;
      float f = x - i0;
      int i1 = (i0 + 1) % N;
      float u = (float) ((rec_z_[k] - off_) / amp_);
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
      if (++diff_count_ < 2) { last_learn_ = DIFFERENT_WAITING; return; }
      beta = 0.75f;
      last_learn_ = ACCEPTED_NEW_SHAPE;
    } else {
      diff_count_ = 0;
      last_learn_ = ACCEPTED;
    }
    float ns[N];
    for (int i = 0; i < N; i++) ns[i] = shape_[i] + beta * (sm[i] - shape_[i]);
    float mx = ns[0], mn = ns[0];
    for (int i = 1; i < N; i++) { if (ns[i] > mx) mx = ns[i]; if (ns[i] < mn) mn = ns[i]; }
    float mid = (mx + mn) / 2, half = (mx - mn) / 2;
    if (half < 0.2f) { last_learn_ = OUT_OF_RANGE; return; }   // a degenerate result: keep the shape in use
    for (int i = 0; i < N; i++) shape_[i] = (ns[i] - mid) / half;
    off_ += amp_ * mid; amp_ *= half;
    build_();
    updates_++;
    save_now_ = true;
#ifdef LF_DEBUG
    fprintf(stderr, "LEARN %u rms=%.3f off=%.2f amp=%.2f\n", (unsigned) updates_, rms, off_, amp_);
#endif
    dirty_ = true;
  }

  // ---- persistence ----
  bool loaded_ = false, dirty_ = false, save_now_ = false;
  double last_save_ = -1e9;
#ifdef LIVE_FLOW_PERSIST
  esphome::ESPPreferenceObject pref_;
#endif
  void load_() {
    loaded_ = true;
#ifdef LIVE_FLOW_PERSIST
    using namespace esphome;
    pref_ = esphome::global_preferences->make_preference<Learned>(MAGIC);
    Learned l;
    if (pref_.load(&l) && l.magic == MAGIC && l.amplitude > 1 && l.amplitude < 1000) {
      std::memcpy(shape_, l.shape, sizeof(shape_));
      off_ = l.offset; amp_ = l.amplitude; updates_ = l.shape_updates;
      off_ref_ = off_; amp_ref_ = amp_;
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
    l.magic = MAGIC; l.offset = (float) off_; l.amplitude = (float) amp_; l.shape_updates = updates_;
    std::memcpy(l.shape, shape_, sizeof(shape_));
    pref_.save(&l);
#endif
  }
};

inline Tracker &tracker() {
  static Tracker t;
  return t;
}

}  // namespace live_flow
