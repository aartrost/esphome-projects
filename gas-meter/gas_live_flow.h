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
//      cycles (> 60 s per count), which tracks temperature drift and gain;
//   2. the shape itself, re-learned from each full cycle of steady flow (the
//      water heater pilot provides one every ~11 min), timed purely from the
//      field's own crossings so a wrong shape cannot block its own correction;
//   3. both are saved to flash, so a restart resumes with what was learned.
// The seed shape was recorded from the pilot on 2026-10-24.
//
// Phase 0 = the field rising through shape(0); one cycle = one count.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>
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
static const float H = 1.5f;      // uT past a peak/trough before switching branch
static const float EMA = 0.8f;    // extra smoothing on the 2 Hz input
static const float SZ = 0.3f;     // field noise after smoothing (uT)
static const float CHI = 2.0f;    // fit-consistency limit that marks a flow change
static const float MINW = 0.15f;  // a window must be this full before it is used
static const int NW = 7;
static const float WINS[NW] = {5, 10, 20, 40, 80, 160, 320};  // seconds
static const int HMAX = 700;      // > 320 s at 2 Hz

// Adaptation.
static const float SLOW_CYCLE_S = 60;     // learn offset/amplitude only from cycles slower than this
static const float EXT_ALPHA = 0.5f;      // weight of each new peak/trough pair
static const float LEARN_MIN_S = 200;     // learn shape only from cycles this slow ...
static const float LEARN_MAX_S = 2400;    // ... and no slower (recording buffer size)
static const float STEADY_TOL = 0.03f;    // consecutive cycles must agree this closely
static const float SHAPE_BETA = 0.25f;    // weight of each accepted cycle
static const float SHAPE_BIG = 0.25f;     // RMS change that counts as "different"
static const int REC_MAX = 2500;          // 1 Hz recording of one cycle
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

  // Feed one Z sample (uT, ~2 Hz) at time t (s). Returns true once tracking.
  bool update(double t, float z) {
    if (!loaded_) load_();
    if (!started_) return start_(t, z);
    double dt = t - tp_;
    tp_ = t;
    if (dt <= 0) return true;
    zf_ += EMA * (z - zf_);
    ring_[ri_] = zf_;
    ri_ = (ri_ + 1) % RING;
    if (rn_ < RING) rn_++;
    float zx = smoothed_();

    // Branch (rising/falling) with hysteresis on the cycle-sized moving average,
    // and only where a turn is plausible (a peak above the midline, a trough
    // below it), so noise on the long flat bottom cannot flip the branch.
    float umid = off_ + amp_ * (smax_ + smin_) / 2;
    if (rising_) {
      if (zx > ext_) ext_ = zx;
      if (zx < ext_ - H && ext_ > umid) { on_extreme_(true, ext_, t); rising_ = false; ext_ = zx; }
    } else {
      if (zx < ext_) ext_ = zx;
      if (zx > ext_ + H && ext_ < umid) { on_extreme_(false, ext_, t); rising_ = true; ext_ = zx; }
    }

    // Reference-level crossings (alternating up/down) time each cycle for shape
    // learning.
    float lref = off_ + amp_ * shape_[0];
    if (zxprev_ < lref && zx >= lref && rising_ && !last_up_) {
      on_cross_(t - dt * (zx - lref) / (zx - zxprev_), true);
      last_up_ = true;
    }
    if (zxprev_ > lref && zx <= lref && !rising_ && last_up_) {
      on_cross_(t - dt * (lref - zx) / (zxprev_ - zx), false);
      last_up_ = false;
    }
    zxprev_ = zx;
    if (rec_on_ && (t - rec_last_) >= 1.0 && rec_n_ < REC_MAX) {
      rec_t_[rec_n_] = (float) (t - rec_t0_);
      rec_z_[rec_n_] = zx;  // same signal as the crossings, so the phase reference holds still
      rec_n_++;
      rec_last_ = t;
    }

    // Position within the cycle on the current branch, forward only.
    double base = phase_(zf_, rising_);
    double c = base + std::floor(phi_ - base);
    if (c < phi_ - 0.02) c += 1;
    if (c - phi_ > 0.5) c -= 1;
    if ((rising_ && zf_ > zmax_() - H) || (!rising_ && zf_ < zmin_() + H)) {
      // Flat top/bottom: coast at the current rate, never past the turning point.
      double lim = rising_ ? pmax_ : pmin_;
      lim += std::floor(phi_ - lim);
      if (lim < phi_ - 0.02) lim += 1;
      double coast = phi_ + r_ * dt;
      if (coast > lim) coast = lim;
      if (coast > c) c = coast;
    }
    if (c > phi_) phi_ = c;

    double p = phi_ - std::floor(phi_);
    float dzdp = amp_ * std::fabs(sh_(p + 0.004) - sh_(p - 0.004)) / 0.008f;
    float sp = SZ / (dzdp > 1e-3f ? dzdp : 1e-3f);
    push_(t, phi_, sp);
    fit_(t);
    if (dirty_ && (t - last_save_) >= SAVE_EVERY_S) save_(t);
    return true;
  }

  bool ready() const { return started_; }
  double cycles_per_s() const { return r_; }
  double phase() const { return phi_; }  // unwrapped, in counts
  float offset() const { return off_; }
  float amplitude() const { return amp_; }
  uint32_t shape_updates() const { return updates_; }
  int last_learn() const { return last_learn_; }  // see LearnResult

  enum LearnResult { NONE = 0, ACCEPTED = 1, NOT_STEADY = 2, OUT_OF_RANGE = 3, DIFFERENT_WAITING = 4, ACCEPTED_NEW_SHAPE = 5, FIRST_CYCLE = 6 };

  // Forget everything learned and go back to the recorded seed shape.
  void reset() {
    seed_();
    started_ = false; n0_ = 0; rec_on_ = false; hn_ = 0; hs_ = 0; r_ = 0;
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
  bool start_(double t, float z) {
    // Decide the initial branch from the trend of the first ~20 s.
    if (n0_ == 0) z0_ = z;
    zf_ = z;
    if (++n0_ < 40) return false;
    rising_ = z > z0_;
    ext_ = zf_ = zxprev_ = z;
    // the last crossing was upward if we are above the reference level now
    last_up_ = z > off_ + amp_ * shape_[0];
    rn_ = 0; ri_ = 0;
    phi_ = phase_(zf_, rising_);
    tp_ = t; started_ = true; r_ = 0;
    return false;
  }

  // ---- branch / extremes -------------------------------------------------
  bool rising_ = true;
  float zf_ = 0, ext_ = 0, zxprev_ = 0;
  bool last_up_ = false;
  double tp_ = 0, phi_ = 0, r_ = 0;
  static const int RING = 30;  // 15 s at 2 Hz
  float ring_[RING];
  int ri_ = 0, rn_ = 0;
  // Peaks/troughs are taken from a moving average sized to the cycle so noise
  // does not inflate them; a few seconds at pilot speed.
  float smoothed_() const {
    double period = r_ > 1e-6 ? 1.0 / r_ : 1e9;
    int m = (int) (period / 40.0 * 2.0);
    if (m < 1) m = 1;
    if (m > rn_) m = rn_;
    if (m > RING) m = RING;
    double s = 0;
    for (int k = 1; k <= m; k++) s += ring_[(ri_ - k + RING) % RING];
    return (float) (s / m);
  }
  bool have_pk_ = false, have_tr_ = false;
  float pk_ = 0, tr_ = 0;
  double t_pk_ = 0, t_tr_ = 0, r_pk_ = 0, r_tr_ = 0;
  void on_extreme_(bool peak, float value, double t) {
    double period = r_ > 1e-6 ? 1.0 / r_ : 1e9;
    bool slow = period > SLOW_CYCLE_S && period < 1e8;
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
  double rec_t0_ = 0, rec_last_ = 0, rec_fall_ = -1;
  int rec_n_ = 0;
  float rec_t_[REC_MAX], rec_z_[REC_MAX];
  float prev_period_ = 0, prev_split_ = 0;
  int diff_count_ = 0, last_learn_ = NONE;
  void on_cross_(double tc, bool up) {
    if (!up) { if (rec_on_ && rec_fall_ < 0) rec_fall_ = tc - rec_t0_; return; }
    if (rec_on_) finish_cycle_(tc);
    rec_on_ = true; rec_t0_ = tc; rec_last_ = tc - 1; rec_n_ = 0; rec_fall_ = -1;
  }
  void finish_cycle_(double tc) {
    float period = (float) (tc - rec_t0_);
    float split = rec_fall_ > 0 ? (float) rec_fall_ / period : -1;
    bool steady = prev_period_ > 0 && split > 0 && prev_split_ > 0 &&
                  std::fabs(period / prev_period_ - 1) < STEADY_TOL &&
                  std::fabs(split - prev_split_) < STEADY_TOL;
    bool range = period >= LEARN_MIN_S && period <= LEARN_MAX_S && rec_n_ >= N;
    bool first = prev_period_ <= 0;
    prev_period_ = period; prev_split_ = split;
#ifdef LF_DEBUG
    fprintf(stderr, "t=%.0f CYCLE period=%.1f split=%.3f steady=%d range=%d n=%d\n", tc, period, split, steady, range, rec_n_);
#endif
    if (!range) { last_learn_ = OUT_OF_RANGE; return; }
    if (first) { last_learn_ = FIRST_CYCLE; return; }  // nothing to compare with yet
    if (!steady) { last_learn_ = NOT_STEADY; return; }
    // Average the recording into N phase bins (time is proportional to volume).
    // Shape index i is phase i/N, so each sample is shared between the two
    // nearest indices (plain bucketing would shift the shape half a bin per
    // update and walk it around the cycle).
    float sum[N], cnt[N];
    for (int i = 0; i < N; i++) { sum[i] = 0; cnt[i] = 0; }
    for (int k = 0; k < rec_n_; k++) {
      float x = rec_t_[k] / period * N;
      if (x < 0 || x >= N) continue;
      int i0 = (int) x;
      float f = x - i0;
      int i1 = (i0 + 1) % N;
      float u = (rec_z_[k] - off_) / amp_;
      sum[i0] += (1 - f) * u; cnt[i0] += 1 - f;
      sum[i1] += f * u; cnt[i1] += f;
    }
    float nw[N];
    for (int i = 0; i < N; i++) {
      if (cnt[i] < 0.3f) { last_learn_ = OUT_OF_RANGE; return; }
      nw[i] = sum[i] / cnt[i];
    }
    float sm[N];
    for (int i = 0; i < N; i++) {
      float s = 0;
      for (int d = -2; d <= 2; d++) s += nw[(i + d + N) % N];
      sm[i] = s / 5;
    }
    float rms = 0;
    for (int i = 0; i < N; i++) rms += (sm[i] - shape_[i]) * (sm[i] - shape_[i]);
    rms = std::sqrt(rms / N);
    float beta = SHAPE_BETA;
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
#ifdef LF_DEBUG
    fprintf(stderr, "LEARN %u rms=%.3f s0=%.3f pmax=%.3f pmin=%.3f off=%.2f amp=%.2f shape:", (unsigned) updates_, rms, shape_[0], pmax_, pmin_, off_, amp_);
    for (int i = 0; i < N; i += 8) fprintf(stderr, " %.2f", shape_[i]);
    fprintf(stderr, "\n");
#endif
    dirty_ = true;
  }

  // ---- flow fit ----------------------------------------------------------
  struct Pt { double t, phi; float s; };
  Pt h_[HMAX];
  int hs_ = 0, hn_ = 0;
  void push_(double t, double phi, float s) {
    while (hn_ > 0 && h_[hs_].t < t - WINS[NW - 1]) { hs_ = (hs_ + 1) % HMAX; hn_--; }
    if (hn_ == HMAX) { hs_ = (hs_ + 1) % HMAX; hn_--; }
    h_[(hs_ + hn_) % HMAX] = {t, phi, s};
    hn_++;
  }
  // Longest window over which a straight line (constant flow) still fits. One
  // pass from the newest sample backwards; times and phases relative to now so
  // the sums stay small (the C6 has no FPU; this is ~1 ms per sample).
  void fit_(double t) {
    bool have = false;
    double rsel = 0, sw = 0, st = 0, sp2 = 0, stt = 0, stp = 0, spp = 0, tfirst = t;
    int n = 0, k = 0;
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
          }
        }
        k++;
        if (k < NW && i >= 0) end = h_[(hs_ + i) % HMAX].t < t - WINS[k];
      }
      if (i < 0 || k >= NW) break;
      const Pt &q = h_[(hs_ + i) % HMAX];
      double w = 1.0 / ((double) q.s * q.s), tt = q.t - t, pp = q.phi - phi_;
      sw += w; st += w * tt; sp2 += w * pp; stt += w * tt * tt; stp += w * tt * pp; spp += w * pp * pp;
      n++;
      tfirst = q.t;
    }
    if (have) r_ = rsel;
  }

  // ---- persistence -------------------------------------------------------
  bool loaded_ = false, dirty_ = false;
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
