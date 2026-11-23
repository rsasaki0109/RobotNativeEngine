#!/usr/bin/env python3
"""Satellite directions and measured C/N0 along a real drive through Shinjuku.

Reads the UrbanNav Tokyo "Shinjuku" run (IPNL, PolyU; MIT licence): the
u-blox F9P rover's RINEX observations, the broadcast ephemerides and the
Applanix POS LV620 ground truth. Writes, once a second, every GPS, Galileo
and QZSS satellite above the elevation mask with its azimuth and elevation
at the true antenna position and the C/N0 the receiver measured on L1 (empty
when it did not track the satellite):

    python3 examples/152_plateau_gnss_navigation/urbannav.py \\
        target/urbannav/Shinjuku target/urbannav/shinjuku_sky.csv

Example 152's `--urbannav` then ray traces the same directions through the
PLATEAU model and compares line of sight, NLOS and blocked predictions with
the measured signals. Fetch the four input files from the dataset's
Tokyo_Data.zip (Tokyo_Data/Shinjuku/{reference.csv, base.nav,
rover_ublox.obs}; see https://github.com/IPNL-POLYU/UrbanNavDataset).
"""

import argparse
import bisect
import csv
import math
import pathlib

MU = {"G": 3.986005e14, "J": 3.986005e14, "E": 3.986004418e14}
OMEGA_E = 7.2921151467e-5
SYSTEMS = "GEJ"
MASK_DEG = 5.0


def number(text):
    text = text.strip().replace("D", "E").replace("d", "e")
    return float(text) if text else 0.0


def read_nav(path):
    """Kepler ephemerides per satellite: list of dicts."""
    lines = pathlib.Path(path).read_text().splitlines()
    start = next(i for i, l in enumerate(lines) if "END OF HEADER" in l) + 1
    ephs = {}
    i = start
    while i < len(lines):
        line = lines[i]
        sys = line[:1]
        if sys not in "GEJCRSI" or len(line) < 23:
            i += 1
            continue
        count = 4 if sys in "RS" else 8
        record = lines[i : i + count]
        i += count
        if sys not in SYSTEMS:
            continue
        values = [number(record[0][23 + 19 * k : 42 + 19 * k]) for k in range(3)]
        for row in record[1:]:
            values += [number(row[4 + 19 * k : 23 + 19 * k]) for k in range(4)]
        names = [
            "af0", "af1", "af2", "iode", "crs", "dn", "m0", "cuc", "e", "cus", "sqrta",
            "toe", "cic", "omega0", "cis", "i0", "crc", "omega", "omegadot", "idot",
            "codes", "week", "l2p", "accuracy", "health", "tgd", "iodc",
        ]
        eph = dict(zip(names, values))
        ephs.setdefault(line[:3].replace(" ", "0"), []).append(eph)
    return ephs


def sat_position(eph, sys, tow):
    """ECEF of the satellite at GPS time of week `tow` (no light-time)."""
    a = eph["sqrta"] ** 2
    tk = tow - eph["toe"]
    if tk > 302400:
        tk -= 604800
    elif tk < -302400:
        tk += 604800
    n = math.sqrt(MU[sys] / a**3) + eph["dn"]
    m = eph["m0"] + n * tk
    e = eph["e"]
    big_e = m
    for _ in range(10):
        big_e = m + e * math.sin(big_e)
    nu = math.atan2(math.sqrt(1 - e * e) * math.sin(big_e), math.cos(big_e) - e)
    phi = nu + eph["omega"]
    du = eph["cus"] * math.sin(2 * phi) + eph["cuc"] * math.cos(2 * phi)
    dr = eph["crs"] * math.sin(2 * phi) + eph["crc"] * math.cos(2 * phi)
    di = eph["cis"] * math.sin(2 * phi) + eph["cic"] * math.cos(2 * phi)
    u = phi + du
    r = a * (1 - e * math.cos(big_e)) + dr
    inc = eph["i0"] + di + eph["idot"] * tk
    x, y = r * math.cos(u), r * math.sin(u)
    omega = eph["omega0"] + (eph["omegadot"] - OMEGA_E) * tk - OMEGA_E * eph["toe"]
    return (
        x * math.cos(omega) - y * math.cos(inc) * math.sin(omega),
        x * math.sin(omega) + y * math.cos(inc) * math.cos(omega),
        y * math.sin(inc),
    )


def best_ephemeris(candidates, tow):
    healthy = [e for e in candidates if e["health"] == 0] or candidates
    return min(healthy, key=lambda e: abs(((tow - e["toe"] + 302400) % 604800) - 302400))


def az_el(rx, lat, lon, sat):
    d = [s - r for s, r in zip(sat, rx)]
    sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    east = -so * d[0] + co * d[1]
    north = -sl * co * d[0] - sl * so * d[1] + cl * d[2]
    up = cl * co * d[0] + cl * so * d[1] + sl * d[2]
    return math.degrees(math.atan2(east, north)) % 360, math.degrees(
        math.atan2(up, math.hypot(east, north))
    )


def read_obs(path):
    """C/N0 on L1 per satellite for whole-second epochs: {tow: {sat: cn0}}."""
    snr_index = {}
    epochs = {}
    with open(path) as f:
        for line in f:
            if "SYS / # / OBS TYPES" in line:
                sys = line[0]
                types = line[7:58].split()
                snr_index[sys] = next(
                    (k for k, t in enumerate(types) if t.startswith("S1")), None
                )
            if "END OF HEADER" in line:
                break
        current = None
        for line in f:
            if line.startswith(">"):
                parts = line[1:].split()
                y, mo, d, h, mi = (int(v) for v in parts[:5])
                sec = float(parts[5])
                # GPS time of week (the run is within one week, 2018-12-19).
                days = (
                    __import__("datetime").date(y, mo, d)
                    - __import__("datetime").date(1980, 1, 6)
                ).days
                tow = (days % 7) * 86400 + h * 3600 + mi * 60 + sec
                current = {} if abs(tow - round(tow)) < 0.01 else None
                if current is not None:
                    epochs[round(tow)] = current
                continue
            if current is None or line[:1] not in SYSTEMS:
                continue
            k = snr_index.get(line[0])
            if k is None:
                continue
            field = line[3 + 16 * k : 3 + 16 * k + 14]
            if field.strip():
                current[line[:3].replace(" ", "0")] = number(field)
    return epochs


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run", type=pathlib.Path)
    parser.add_argument("output", type=pathlib.Path)
    args = parser.parse_args()
    ephs = read_nav(args.run / "base.nav")
    obs = read_obs(args.run / "rover_ublox.obs")
    reference = []
    with open(args.run / "reference.csv") as f:
        rows = csv.reader(f)
        next(rows)
        for r in rows:
            reference.append(tuple(float(v) for v in (r[0], r[2], r[3], r[4], r[5], r[6], r[7])))
    times = [r[0] for r in reference]
    written = 0
    with open(args.output, "w", newline="") as out:
        w = csv.writer(out)
        w.writerow(["tow_s", "lat_deg", "lon_deg", "height_m", "sat", "az_deg", "el_deg", "cn0_dbhz"])
        for tow in sorted(obs):
            k = bisect.bisect_left(times, tow)
            if k >= len(times) or abs(times[k] - tow) > 0.06:
                continue
            _, lat, lon, h, x, y, z = reference[k]
            rx = (x, y, z)
            for sat, candidates in sorted(ephs.items()):
                eph = best_ephemeris(candidates, tow)
                pos = sat_position(eph, sat[0], tow - 0.075)
                # Earth rotation during the signal's flight.
                theta = OMEGA_E * 0.075
                pos = (
                    pos[0] * math.cos(theta) + pos[1] * math.sin(theta),
                    -pos[0] * math.sin(theta) + pos[1] * math.cos(theta),
                    pos[2],
                )
                az, el = az_el(rx, math.radians(lat), math.radians(lon), pos)
                if el < MASK_DEG:
                    continue
                cn0 = obs[tow].get(sat)
                w.writerow([tow, lat, lon, h, sat, f"{az:.2f}", f"{el:.2f}", "" if cn0 is None else cn0])
                written += 1
    print(f"wrote {args.output}: {written} satellite-epochs over {len(obs)} epochs")


if __name__ == "__main__":
    main()
