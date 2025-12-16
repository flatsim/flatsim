#!/usr/bin/env python3
import os
import errno
import termios
import time
from datetime import datetime, timezone
import math
import struct
import mmap
import glob
import argparse
import sys

# Default serial port
DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
DEFAULT_SERIAL_BAUD = 115200

# Base "true" position in degrees (LLH)
BASE_LAT_DEG = 51.987900  # degrees North
BASE_LON_DEG = 5.663000  # degrees East
BASE_ALT_M = 10.0  # meters above mean sea level

# Base velocity and heading for RMC
BASE_SOG_KNOTS = 1.2  # speed over ground in knots
BASE_COG_DEG = 45.0  # course over ground in degrees

# Fix quality (GGA)
GGA_FIX_QUALITY = 4  # use 4 to emulate RTK fixed
GGA_NUM_SATS = 10
GGA_HDOP = 0.8

# GST 1-sigma LLH errors in meters
GST_SIGMA_LAT_M = 0.02  # 2 cm
GST_SIGMA_LON_M = 0.02
GST_SIGMA_ALT_M = 0.05

# GNS mode indicators per constellation
GNS_MODE_GPS = "R"
GNS_MODE_GLONASS = "N"
GNS_MODE_GALILEO = "R"
GNS_MODE_BEIDOU = "N"
GNS_MODE_QZSS = "N"

# PHTG authentication message defaults
PHTG_SYSTEM = "GAL"
PHTG_SERVICE = "HAS"
PHTG_AUTH_RESULT = 0
PHTG_STATUS = 0
PHTG_WARNING = 0

# Talker IDs
TALKER_GPS = "GP"
TALKER_GN = "GN"


def open_serial_nonblocking(path, baud=115200):
    """Open serial port for non-blocking writes that discard data if no reader"""
    fd = os.open(path, os.O_WRONLY | os.O_NONBLOCK | os.O_NOCTTY)

    # Configure baud rate and raw mode
    baud_map = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
    }
    try:
        attrs = termios.tcgetattr(fd)
        attrs[4] = attrs[5] = baud_map.get(baud, termios.B115200)
        attrs[3] &= ~termios.ICANON  # raw mode
        attrs[3] &= ~termios.ECHO  # no echo
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
    except termios.error:
        pass  # PTY may not support all termios operations

    return fd


def write_nonblocking(fd, data: bytes):
    """Write data, silently discard if no reader or would block"""
    try:
        os.write(fd, data)
    except OSError as e:
        if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EIO, errno.ENXIO, errno.EPIPE):
            pass  # No reader or would block - just discard
        else:
            raise


def nmea_checksum(body: str) -> str:
    """Compute NMEA-0183 checksum (two hex digits)"""
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"{cs:02X}"


def format_lat_nmea(lat_deg: float):
    """Convert latitude in degrees to NMEA ddmm.mmmmmmmm format + N/S."""
    hemi = "N" if lat_deg >= 0 else "S"
    lat_abs = abs(lat_deg)
    deg = int(lat_abs)
    minutes = (lat_abs - deg) * 60.0
    return f"{deg:02d}{minutes:011.8f}", hemi


def format_lon_nmea(lon_deg: float):
    """Convert longitude in degrees to NMEA dddmm.mmmmmmmm format + E/W."""
    hemi = "E" if lon_deg >= 0 else "W"
    lon_abs = abs(lon_deg)
    deg = int(lon_abs)
    minutes = (lon_abs - deg) * 60.0
    return f"{deg:03d}{minutes:011.8f}", hemi


def now_utc():
    """Return (time_str, date_str, datetime) for NMEA"""
    now = datetime.now(timezone.utc)
    time_str = now.strftime("%H%M%S") + f".{int(now.microsecond / 1000):03d}"
    date_str = now.strftime("%d%m%y")
    return time_str, date_str, now


def build_gga(lat_deg: float, lon_deg: float, alt_m: float) -> str:
    """Build a GGA sentence"""
    t_str, _, _ = now_utc()
    lat_str, lat_hemi = format_lat_nmea(lat_deg)
    lon_str, lon_hemi = format_lon_nmea(lon_deg)
    geoid_sep = 0.0
    body = (
        f"{TALKER_GPS}GGA,"
        f"{t_str},"
        f"{lat_str},{lat_hemi},"
        f"{lon_str},{lon_hemi},"
        f"{GGA_FIX_QUALITY},"
        f"{GGA_NUM_SATS:02d},"
        f"{GGA_HDOP:.1f},"
        f"{alt_m:.2f},M,"
        f"{geoid_sep:.2f},M,,"
    )
    cs = nmea_checksum(body)
    return f"${body}*{cs}\r\n"


def build_rmc(lat_deg: float, lon_deg: float, sog_knots: float, cog_deg: float) -> str:
    """Build an RMC sentence"""
    t_str, d_str, _ = now_utc()
    lat_str, lat_hemi = format_lat_nmea(lat_deg)
    lon_str, lon_hemi = format_lon_nmea(lon_deg)
    status = "A"
    mag_var = ""
    mag_hemi = ""
    body = (
        f"{TALKER_GPS}RMC,"
        f"{t_str},"
        f"{status},"
        f"{lat_str},{lat_hemi},"
        f"{lon_str},{lon_hemi},"
        f"{sog_knots:.1f},"
        f"{cog_deg:.1f},"
        f"{d_str},"
        f"{mag_var},{mag_hemi}"
    )
    cs = nmea_checksum(body)
    return f"${body}*{cs}\r\n"


def build_gns(lat_deg: float, lon_deg: float, alt_m: float) -> str:
    """Build a GNS sentence"""
    t_str, _, _ = now_utc()
    lat_str, lat_hemi = format_lat_nmea(lat_deg)
    lon_str, lon_hemi = format_lon_nmea(lon_deg)
    mode_str = (
        f"{GNS_MODE_GPS}{GNS_MODE_GLONASS}"
        f"{GNS_MODE_GALILEO}{GNS_MODE_BEIDOU}"
        f"{GNS_MODE_QZSS}"
    )
    geoid_sep = 0.0
    body = (
        f"{TALKER_GN}GNS,"
        f"{t_str},"
        f"{lat_str},{lat_hemi},"
        f"{lon_str},{lon_hemi},"
        f"{mode_str},"
        f"{GGA_HDOP:.1f},"
        f"{alt_m:.2f},"
        f"{geoid_sep:.2f},,,"
    )
    cs = nmea_checksum(body)
    return f"${body}*{cs}\r\n"


def build_gst(lat_deg: float, lon_deg: float) -> str:
    """Build a GST sentence"""
    t_str, _, _ = now_utc()
    sigma_lat = GST_SIGMA_LAT_M
    sigma_lon = GST_SIGMA_LON_M
    sigma_alt = GST_SIGMA_ALT_M
    rms = math.sqrt((sigma_lat**2 + sigma_lon**2) / 2.0)
    sigma_major = sigma_lat
    sigma_minor = sigma_lon
    orientation = 0.0
    body = (
        f"{TALKER_GPS}GST,"
        f"{t_str},"
        f"{rms:.3f},"
        f"{sigma_major:.3f},"
        f"{sigma_minor:.3f},"
        f"{orientation:.3f},"
        f"{sigma_lat:.3f},"
        f"{sigma_lon:.3f},"
        f"{sigma_alt:.3f}"
    )
    cs = nmea_checksum(body)
    return f"${body}*{cs}\r\n"


def build_gsv() -> str:
    """Build a GSV sentence"""
    num_sats_view = max(GGA_NUM_SATS, 1)
    num_msgs = 1
    msg_idx = 1
    sat_id = 11
    elevation = 45
    azimuth = 120
    snr = 40
    body = (
        f"{TALKER_GPS}GSV,"
        f"{num_msgs},"
        f"{msg_idx},"
        f"{num_sats_view},"
        f"{sat_id},{elevation},{azimuth},{snr}"
    )
    cs = nmea_checksum(body)
    return f"${body}*{cs}\r\n"


def build_phtg(now: datetime) -> str:
    """Build the proprietary authentication NMEA message"""
    date_part = now.strftime("%d:%m:%Y")
    time_part = now.strftime("%H:%M:%S") + ".00"
    timetag = f"{date_part},{time_part}"
    body = f"PHTG,{timetag},{PHTG_SYSTEM},{PHTG_SERVICE},{PHTG_AUTH_RESULT},{PHTG_WARNING}"
    cs = nmea_checksum(body)
    return f"${body}*{cs}\r\n"


def read_gps_shm(shm_path):
    """Read GPS NMEA sentences from shared memory"""
    try:
        with open(shm_path, "rb") as f:
            mm = mmap.mmap(f.fileno(), 0, prot=mmap.PROT_READ)
            seq = struct.unpack("<Q", mm[0:8])[0]
            ts_ns = struct.unpack("<Q", mm[8:16])[0]
            data_size = struct.unpack("<I", mm[16:20])[0]
            if data_size > 0 and data_size < 2048:
                nmea_bytes = mm[24 : 24 + data_size]
                nmea = nmea_bytes.decode("ascii", errors="ignore").strip()
                mm.close()
                return seq, ts_ns, nmea
            mm.close()
    except Exception:
        pass
    return None, None, None


def parse_nmea_for_coords(nmea_data):
    """Parse NMEA data to extract lat, lon, alt, speed, heading"""
    lines = [l for l in nmea_data.split("\n") if l.strip()]
    lat = BASE_LAT_DEG
    lon = BASE_LON_DEG
    alt = BASE_ALT_M
    sog = BASE_SOG_KNOTS
    cog = BASE_COG_DEG

    gga_line = next((l for l in lines if "GGA" in l), None)
    if gga_line:
        fields = gga_line.split(",")
        if len(fields) > 6 and fields[2] and fields[4]:
            lat_nmea = fields[2]
            lon_nmea = fields[4]
            lat_hemi = fields[3]
            lon_hemi = fields[5]
            if lat_nmea and len(lat_nmea) > 2:
                lat_deg = float(lat_nmea[:2]) + float(lat_nmea[2:]) / 60.0
                lat = lat_deg if lat_hemi == "N" else -lat_deg
            if lon_nmea and len(lon_nmea) > 3:
                lon_deg = float(lon_nmea[:3]) + float(lon_nmea[3:]) / 60.0
                lon = lon_deg if lon_hemi == "E" else -lon_deg
            if len(fields) > 9 and fields[9]:
                alt = float(fields[9])

    rmc_line = next((l for l in lines if "RMC" in l), None)
    if rmc_line:
        fields = rmc_line.split(",")
        if len(fields) > 7:
            if fields[7]:
                sog = float(fields[7])
            if fields[8]:
                cog = float(fields[8])

    return lat, lon, alt, sog, cog


def main(serial_port, baud, shm_path=None):
    print(f"Opening serial {serial_port} @ {baud} baud (non-blocking, no buffer)...")

    if not os.path.exists(serial_port):
        print(f"Error: {serial_port} not found!")
        sys.exit(1)

    fd = open_serial_nonblocking(serial_port, baud)

    # Determine GPS shared memory path
    if shm_path is None:
        shm_files = sorted(
            glob.glob("/dev/shm/flatsim_*_GPS"),
            key=lambda f: os.path.getmtime(f),
            reverse=True,
        )
        if not shm_files:
            print("No GPS shared memory found!")
            print("Make sure test_gps_nmea is running")
            os.close(fd)
            return
        shm_path = shm_files[0]

    print(f"Using GPS data from: {shm_path}")
    print("Sending live GPS NMEA data at 1 Hz. CTRL+C to stop.")

    last_seq = 0
    fallback_count = 0

    try:
        while True:
            try:
                seq, ts_ns, nmea = read_gps_shm(shm_path)

                if seq is not None and nmea and seq != last_seq:
                    lat, lon, alt, sog, cog = parse_nmea_for_coords(nmea)
                    t_str, d_str, now = now_utc()

                    sentences = [
                        build_gga(lat, lon, alt),
                        build_rmc(lat, lon, sog, cog),
                        build_gns(lat, lon, alt),
                        build_gst(lat, lon),
                        build_gsv(),
                        build_phtg(now),
                    ]

                    for s in sentences:
                        write_nonblocking(fd, s.encode("ascii"))
                        print("TX:", s.strip())
                        time.sleep(0.01)

                    last_seq = seq
                    fallback_count = 0
                else:
                    fallback_count += 1
                    if fallback_count % 5 == 1:
                        print(f"No new GPS data (seq: {seq}), using fallback values...")

                    t_str, d_str, now = now_utc()
                    sentences = [
                        build_gga(BASE_LAT_DEG, BASE_LON_DEG, BASE_ALT_M),
                        build_rmc(BASE_LAT_DEG, BASE_LON_DEG, BASE_SOG_KNOTS, BASE_COG_DEG),
                        build_gns(BASE_LAT_DEG, BASE_LON_DEG, BASE_ALT_M),
                        build_gst(BASE_LAT_DEG, BASE_LON_DEG),
                        build_gsv(),
                        build_phtg(now),
                    ]

                    for s in sentences:
                        write_nonblocking(fd, s.encode("ascii"))
                        time.sleep(0.01)

                time.sleep(0.05)

            except KeyboardInterrupt:
                raise
            except Exception as e:
                print("Error:", e)
                time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nStopping NMEA output.")
    finally:
        os.close(fd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Send NMEA data from GPS simulation",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-p", "--port",
        default=DEFAULT_SERIAL_PORT,
        help="Serial port path",
    )
    parser.add_argument(
        "-b", "--baud",
        type=int,
        default=DEFAULT_SERIAL_BAUD,
        help="Baud rate",
    )
    parser.add_argument(
        "-s", "--shm",
        dest="shm_path",
        help="GPS shared memory path (auto-detected if not specified)",
    )
    parser.add_argument(
        "robot_uuid",
        nargs="?",
        help="Robot UUID (optional, uses most recent if not specified)",
    )
    args = parser.parse_args()

    # Determine shared memory path
    shm_path = args.shm_path
    if shm_path is None and args.robot_uuid:
        shm_path = f"/dev/shm/flatsim_{args.robot_uuid}_GPS"
        if not os.path.exists(shm_path):
            print(f"Error: {shm_path} not found!")
            sys.exit(1)

    main(args.port, args.baud, shm_path)
