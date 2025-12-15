#!/usr/bin/env python3
"""
Read GPS NMEA sentences from shared memory
Usage: python3 read_gps_nmea.py [robot_uuid]

This reads NMEA output from the GPS sensor running at simulation tick rate (60 FPS).
Each update contains all 6 NMEA sentence types (GGA, RMC, GNS, GST, GSV, PHTG).
"""
import struct
import mmap
import sys
import time
import glob
import os
import argparse


def read_gps_shm(shm_path):
    """Read GPS NMEA sentences from shared memory"""
    try:
        with open(shm_path, 'rb') as f:
            mm = mmap.mmap(f.fileno(), 0, prot=mmap.PROT_READ)
            
            # Read header (16 bytes)
            seq = struct.unpack('<Q', mm[0:8])[0]
            ts_ns = struct.unpack('<Q', mm[8:16])[0]
            data_size = struct.unpack('<I', mm[16:20])[0]
            
            if data_size > 0 and data_size < 2048:
                nmea_bytes = mm[24:24+data_size]
                nmea = nmea_bytes.decode('ascii', errors='ignore').strip()
                mm.close()
                return seq, ts_ns, nmea
            
            mm.close()
    except Exception:
        pass
    
    return None, None, None


def monitor_gps(shm_path, show_all=False):
    """Monitor GPS NMEA output"""
    robot_id = shm_path.split('_')[1][:8] if '_' in shm_path else "unknown"
    print(f"Monitoring GPS: {shm_path}")
    print(f"Robot ID: {robot_id}")
    print(f"Mode: {'Full output' if show_all else 'Summary only'}")
    print(f"Press Ctrl+C to stop\n")
    
    last_seq = 0
    frame_count = 0
    start_time = time.time()
    
    while True:
        seq, ts_ns, nmea = read_gps_shm(shm_path)
        
        if seq is not None and nmea and seq != last_seq:
            frame_count += 1
            lines = [l for l in nmea.split('\n') if l.strip()]
            
            # Calculate rate
            elapsed = time.time() - start_time
            rate = frame_count / elapsed if elapsed > 0 else 0
            
            if show_all:
                # Show all NMEA sentences
                print(f"=== Frame {frame_count:5d} | Seq {seq:6d} | {len(lines)} sentences | Rate: {rate:.1f} Hz ===")
                for line in lines:
                    if line.strip():
                        nmea_type = line[1:6] if len(line) > 6 else "?????"
                        print(f"  {nmea_type}: {line}")
                print()
            else:
                # Show summary every 60 frames (~1 second at 60Hz)
                if frame_count % 60 == 0:
                    # Parse first GGA for position
                    gga_line = next((l for l in lines if 'GGA' in l), None)
                    if gga_line:
                        fields = gga_line.split(',')
                        lat = fields[2] if len(fields) > 2 else "?"
                        lon = fields[4] if len(fields) > 4 else "?"
                        fix = fields[6] if len(fields) > 6 else "?"
                        print(f"Frame {frame_count:5d} | Seq {seq:6d} | Rate: {rate:.1f} Hz | Lat: {lat} Lon: {lon} Fix: {fix}")
            
            last_seq = seq
        
        time.sleep(0.001)  # 1ms sleep


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Read GPS NMEA from shared memory')
    parser.add_argument('robot_uuid', nargs='?', help='Robot UUID (optional)')
    parser.add_argument('--all', '-a', action='store_true', help='Show all NMEA sentences')
    args = parser.parse_args()
    
    # Find GPS shared memory
    if args.robot_uuid:
        shm_path = f"/dev/shm/flatsim_{args.robot_uuid}_GPS"
        if not os.path.exists(shm_path):
            print(f"Error: {shm_path} not found!")
            sys.exit(1)
    else:
        # Find most recent GPS shm
        shm_files = sorted(glob.glob("/dev/shm/flatsim_*_GPS"), 
                          key=lambda f: os.path.getmtime(f), reverse=True)
        if not shm_files:
            print("No GPS shared memory found!")
            print("Make sure test_gps_nmea is running")
            sys.exit(1)
        shm_path = shm_files[0]
    
    try:
        monitor_gps(shm_path, show_all=args.all)
    except KeyboardInterrupt:
        print("\nStopped")
