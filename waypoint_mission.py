#!/usr/bin/env python3
from pymavlink import mavutil
import time, csv, os
from datetime import datetime

LOG_DIR = "/home/pi/test_logs"


#if not os.path.exists(LOG_DIR):
#    raise FileNotFoundError(f"Log klasörü bulunamadı: {LOG_DIR}\n"
#                            f"Lütfen manuel oluştur: mkdir {LOG_DIR}")


while True:
    try:
        master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
        master.wait_heartbeat(timeout=10)
        break
    except Exception:
        time.sleep(2)


msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
start_lat = msg.lat / 1e7
start_lon = msg.lon / 1e7


log_file = os.path.join(LOG_DIR, f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
f = open(log_file, 'w', newline='')
writer = csv.writer(f)
writer.writerow(['time', 'lat', 'lon', 'mode', 'event'])

def log_event(event):
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
    else:
        lat, lon = 0, 0
    writer.writerow([datetime.now().strftime('%H:%M:%S'), lat, lon, master.flightmode, event])
    f.flush()


master.set_mode_apm('GUIDED')
time.sleep(1)
master.arducopter_arm()
master.motors_armed_wait()
log_event("ARMED")


waypoints = [
    (41.00234, 29.01245),
    (41.00290, 29.01320),
    (41.00200, 29.01350),
    (41.00160, 29.01290),
]


def goto(lat, lon, alt=0):
    """Belirtilen koordinata git komutu gönderir"""
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        int(0b110111111000),
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

def check_for_stop():
    """RC Channel 7 üzerinden acil durdurma kontrolü"""
    msg = master.recv_match(type='RC_CHANNELS', blocking=False)
    if msg and msg.chan7_raw < 1200:  # switch off
        master.set_mode_apm('HOLD')
        master.arducopter_disarm()
        log_event("EMERGENCY STOP")
        return True
    return False


try:
    for i, (lat, lon) in enumerate(waypoints):
        if check_for_stop():
            break
        log_event(f"GN{i+1} started")
        goto(lat, lon, 0)
        time.sleep(20)
        log_event(f"GN{i+1} reached")

    log_event("MISSION COMPLETE")
    master.set_mode_apm('RTL')
    log_event("RTL activated")
    time.sleep(10)

except KeyboardInterrupt:
    log_event("Keyboard Interrupt -> RTL")
    master.set_mode_apm('RTL')

finally:
    log_event("Mission Ended")
    f.close()
    master.close()
