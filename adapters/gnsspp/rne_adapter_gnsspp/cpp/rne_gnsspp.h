// C ABI between Robot Native Engine and libgnss++ (C++20).
// Every function catches C++ exceptions and reports failure by return code.
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// system: 1 = GPS, 4 = Galileo (libgnss++ GNSSSystem values).
typedef struct {
    uint8_t system;
    uint8_t prn;
    uint16_t iode;
    uint32_t toe_week;
    double toe_tow;
    double sqrt_a, e, i0, omega0, omega, m0, delta_n, omega_dot, idot;
    double af0, af1;
} rne_gnsspp_ephemeris;

// signal: 0 = GPS L1 C/A, 1 = GPS L2C, 2 = Galileo E1, 3 = Galileo E5a.
typedef struct {
    uint8_t system;
    uint8_t prn;
    uint8_t signal;
    uint8_t has_pseudorange;
    uint8_t has_carrier_phase;
    uint8_t has_doppler;
    uint8_t loss_of_lock;
    double pseudorange_m;
    double carrier_phase_cycles;
    double doppler_hz;
    double cn0_dbhz;
} rne_gnsspp_observation;

typedef struct {
    uint32_t week;
    double tow;
    double approx_ecef_m[3]; // zero = unknown
    const rne_gnsspp_observation* observations;
    size_t count;
} rne_gnsspp_epoch;

// status: 0 none, 1 single, 2 DGPS, 3 float, 4 fixed, other values as libgnss++.
typedef struct {
    int32_t status;
    double ecef_m[3];
    double covariance_m2[9];
    int32_t satellites;
    int32_t fixed_ambiguities;
    double ratio;
    double hdop;
    double baseline_length_m;
} rne_gnsspp_solution;

void* rne_gnsspp_nav_new(void);
void rne_gnsspp_nav_free(void* nav);
int rne_gnsspp_nav_add_ephemeris(void* nav, const rne_gnsspp_ephemeris* eph);
int rne_gnsspp_nav_set_klobuchar(void* nav, const double alpha[4], const double beta[4]);

void* rne_gnsspp_spp_new(double elevation_mask_deg);
void rne_gnsspp_spp_free(void* spp);
int rne_gnsspp_spp_process(void* spp, const rne_gnsspp_epoch* epoch, const void* nav, rne_gnsspp_solution* out);

// position_mode: 0 static, 1 kinematic, 2 moving base.
// preset: a libgnss++ RTK preset name ("", "survey", "low-cost", "odaiba", "moving-base");
// an unknown name fails. max_nonfix_for_reset: reset ambiguities after this
// many consecutive non-fixed epochs (0 disables).
void* rne_gnsspp_rtk_new(int position_mode, double elevation_mask_deg, double ratio_threshold, const char* preset,
                         int max_nonfix_for_reset);
void rne_gnsspp_rtk_free(void* rtk);
int rne_gnsspp_rtk_set_base_position(void* rtk, const double ecef_m[3]);
int rne_gnsspp_rtk_process(void* rtk, const rne_gnsspp_epoch* rover, const rne_gnsspp_epoch* base, const void* nav,
                           rne_gnsspp_solution* out);

typedef struct {
    int32_t ar_attempted;
    int32_t ar_skip_reason;
    int32_t retained_code_rows;
    int32_t retained_phase_rows;
    int32_t pair_count;
    double max_ambiguity_variance;
} rne_gnsspp_rtk_diagnostics;

int rne_gnsspp_rtk_diagnostics_get(const void* rtk, rne_gnsspp_rtk_diagnostics* out);

#ifdef __cplusplus
}
#endif
