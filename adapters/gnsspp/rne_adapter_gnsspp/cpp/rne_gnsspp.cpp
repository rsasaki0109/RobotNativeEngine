// libgnss++ shim for Robot Native Engine. See rne_gnsspp.h.
#include "rne_gnsspp.h"

#include <libgnss++/gnss.hpp>
#include <libgnss++/algorithms/rtk_presets.hpp>

#include <cmath>
#include <exception>
#include <memory>

namespace {

libgnss::GNSSSystem toSystem(uint8_t system) {
    switch (system) {
        case 4: return libgnss::GNSSSystem::Galileo;
        case 0x10: return libgnss::GNSSSystem::QZSS;
        case 8: return libgnss::GNSSSystem::BeiDou;
        default: return libgnss::GNSSSystem::GPS;
    }
}

struct SignalInfo {
    libgnss::SignalType type;
    const char* code;
    const char* phase;
};

SignalInfo toSignal(uint8_t signal) {
    switch (signal) {
        case 1: return {libgnss::SignalType::GPS_L2C, "C2L", "L2L"};
        case 2: return {libgnss::SignalType::GAL_E1, "C1C", "L1C"};
        case 3: return {libgnss::SignalType::GAL_E5A, "C5Q", "L5Q"};
        case 4: return {libgnss::SignalType::QZS_L1CA, "C1C", "L1C"};
        case 5: return {libgnss::SignalType::QZS_L2C, "C2L", "L2L"};
        case 6: return {libgnss::SignalType::BDS_B1I, "C2I", "L2I"};
        case 7: return {libgnss::SignalType::BDS_B3I, "C6I", "L6I"};
        default: return {libgnss::SignalType::GPS_L1CA, "C1C", "L1C"};
    }
}

libgnss::ObservationData toObservationData(const rne_gnsspp_epoch* epoch) {
    libgnss::ObservationData data(libgnss::GNSSTime(static_cast<int>(epoch->week), epoch->tow));
    data.receiver_position = libgnss::Vector3d(epoch->approx_ecef_m[0], epoch->approx_ecef_m[1], epoch->approx_ecef_m[2]);
    for (size_t i = 0; i < epoch->count; ++i) {
        const rne_gnsspp_observation& in = epoch->observations[i];
        const SignalInfo info = toSignal(in.signal);
        libgnss::Observation obs(libgnss::SatelliteId(toSystem(in.system), in.prn), info.type);
        obs.pseudorange = in.pseudorange_m;
        obs.has_pseudorange = in.has_pseudorange != 0;
        obs.carrier_phase = in.carrier_phase_cycles;
        obs.has_carrier_phase = in.has_carrier_phase != 0;
        obs.doppler = in.doppler_hz;
        obs.has_doppler = in.has_doppler != 0;
        obs.snr = in.cn0_dbhz;
        obs.signal_strength = std::max(1, std::min(9, static_cast<int>(in.cn0_dbhz / 6.0)));
        obs.loss_of_lock = in.loss_of_lock != 0;
        obs.lli = in.loss_of_lock != 0 ? 1 : 0;
        obs.pseudorange_observation_type = info.code;
        obs.carrier_phase_observation_type = info.phase;
        obs.valid = obs.has_pseudorange;
        data.addObservation(obs);
    }
    return data;
}

void toSolution(const libgnss::PositionSolution& s, rne_gnsspp_solution* out) {
    out->status = static_cast<int32_t>(s.status);
    for (int i = 0; i < 3; ++i) {
        out->ecef_m[i] = s.position_ecef(i);
        for (int j = 0; j < 3; ++j) {
            out->covariance_m2[i * 3 + j] = s.position_covariance(i, j);
        }
    }
    out->satellites = s.num_satellites;
    out->fixed_ambiguities = s.num_fixed_ambiguities;
    out->ratio = s.ratio;
    out->hdop = s.hdop;
    out->baseline_length_m = s.baseline_length;
}

struct Spp {
    libgnss::SPPProcessor processor;
};

struct Rtk {
    libgnss::RTKProcessor processor;
};

}  // namespace

extern "C" {

void* rne_gnsspp_nav_new(void) {
    try {
        return new libgnss::NavigationData();
    } catch (...) {
        return nullptr;
    }
}

void rne_gnsspp_nav_free(void* nav) { delete static_cast<libgnss::NavigationData*>(nav); }

int rne_gnsspp_nav_add_ephemeris(void* nav, const rne_gnsspp_ephemeris* in) {
    try {
        libgnss::Ephemeris eph;
        eph.satellite = libgnss::SatelliteId(toSystem(in->system), in->prn);
        eph.toe = libgnss::GNSSTime(static_cast<int>(in->toe_week), in->toe_tow);
        eph.toc = eph.toe;
        eph.tof = eph.toe - 30.0;
        eph.toes = in->toe_tow;
        eph.sqrt_a = in->sqrt_a;
        eph.e = in->e;
        eph.i0 = in->i0;
        eph.omega0 = in->omega0;
        eph.omega = in->omega;
        eph.m0 = in->m0;
        eph.delta_n = in->delta_n;
        eph.omega_dot = in->omega_dot;
        eph.idot = in->idot;
        eph.i_dot = in->idot;
        eph.af0 = in->af0;
        eph.af1 = in->af1;
        eph.week = static_cast<uint16_t>(in->toe_week);
        eph.iode = in->iode;
        eph.iodc = in->iode;
        eph.health = 0;
        eph.sv_health = 0.0;
        eph.sv_accuracy = 2.0;
        eph.valid = true;
        if (in->system == 4) {
            // I/NAV E1/E5b clock with both E1 and E5a usable.
            eph.data_source_code = (1 << 0) | (1 << 9);
        }
        static_cast<libgnss::NavigationData*>(nav)->addEphemeris(eph);
        return 0;
    } catch (...) {
        return -1;
    }
}

int rne_gnsspp_nav_set_klobuchar(void* nav, const double alpha[4], const double beta[4]) {
    try {
        auto& model = static_cast<libgnss::NavigationData*>(nav)->ionosphere_model;
        for (int i = 0; i < 4; ++i) {
            model.alpha[i] = alpha[i];
            model.beta[i] = beta[i];
        }
        model.valid = true;
        return 0;
    } catch (...) {
        return -1;
    }
}

void* rne_gnsspp_spp_new(double elevation_mask_deg) {
    try {
        auto spp = std::make_unique<Spp>();
        libgnss::ProcessorConfig config;
        config.mode = libgnss::PositioningMode::SPP;
        config.elevation_mask = elevation_mask_deg;
        spp->processor.initialize(config);
        return spp.release();
    } catch (...) {
        return nullptr;
    }
}

void rne_gnsspp_spp_free(void* spp) { delete static_cast<Spp*>(spp); }

int rne_gnsspp_spp_process(void* spp, const rne_gnsspp_epoch* epoch, const void* nav, rne_gnsspp_solution* out) {
    try {
        const auto data = toObservationData(epoch);
        const auto solution =
            static_cast<Spp*>(spp)->processor.processEpoch(data, *static_cast<const libgnss::NavigationData*>(nav));
        toSolution(solution, out);
        return 0;
    } catch (...) {
        return -1;
    }
}

void* rne_gnsspp_rtk_new(int position_mode, double elevation_mask_deg, double ratio_threshold, const char* preset,
                         int max_nonfix_for_reset) {
    try {
        auto rtk = std::make_unique<Rtk>();
        libgnss::ProcessorConfig config;
        config.mode = libgnss::PositioningMode::RTK_FIXED;
        config.elevation_mask = elevation_mask_deg;
        rtk->processor.initialize(config);
        libgnss::RTKProcessor::RTKConfig rtk_config;
        using Mode = libgnss::RTKProcessor::RTKConfig::PositionMode;
        rtk_config.position_mode = position_mode == 0 ? Mode::STATIC : (position_mode == 2 ? Mode::MOVING_BASE : Mode::KINEMATIC);
        rtk_config.ar_mode = libgnss::RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
        rtk_config.ratio_threshold = ratio_threshold;
        rtk_config.ambiguity_ratio_threshold = ratio_threshold;
        rtk_config.elevation_mask = elevation_mask_deg * M_PI / 180.0;
        if (preset != nullptr && !libgnss::applyRtkPreset(rtk_config, preset)) {
            return nullptr;
        }
        if (preset == nullptr || preset[0] == '\0') {
            rtk_config.ratio_threshold = ratio_threshold;
            rtk_config.ambiguity_ratio_threshold = ratio_threshold;
        }
        rtk_config.max_consecutive_nonfix_for_reset = max_nonfix_for_reset;
        if (position_mode == 2) {
            rtk_config.min_baseline_length = 0.0;
        }
        rtk->processor.setRTKConfig(rtk_config);
        return rtk.release();
    } catch (...) {
        return nullptr;
    }
}

void rne_gnsspp_rtk_free(void* rtk) { delete static_cast<Rtk*>(rtk); }

int rne_gnsspp_rtk_set_base_position(void* rtk, const double ecef_m[3]) {
    try {
        static_cast<Rtk*>(rtk)->processor.setBasePosition(libgnss::Vector3d(ecef_m[0], ecef_m[1], ecef_m[2]));
        return 0;
    } catch (...) {
        return -1;
    }
}

int rne_gnsspp_rtk_process(void* rtk, const rne_gnsspp_epoch* rover, const rne_gnsspp_epoch* base, const void* nav,
                           rne_gnsspp_solution* out) {
    try {
        const auto rover_data = toObservationData(rover);
        const auto base_data = toObservationData(base);
        const auto solution = static_cast<Rtk*>(rtk)->processor.processRTKEpoch(
            rover_data, base_data, *static_cast<const libgnss::NavigationData*>(nav));
        toSolution(solution, out);
        return 0;
    } catch (...) {
        return -1;
    }
}

int rne_gnsspp_rtk_diagnostics_get(const void* rtk, rne_gnsspp_rtk_diagnostics* out) {
    try {
        const auto& t = static_cast<const Rtk*>(rtk)->processor.getLastDebugTelemetry();
        out->ar_attempted = t.ar_attempted ? 1 : 0;
        out->ar_skip_reason = static_cast<int32_t>(t.ar_skip_reason);
        out->retained_code_rows = t.retained_code_rows;
        out->retained_phase_rows = t.retained_phase_rows;
        out->pair_count = t.pair_count;
        out->max_ambiguity_variance = t.max_ambiguity_variance;
        return 0;
    } catch (...) {
        return -1;
    }
}

}  // extern "C"
