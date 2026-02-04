#!/usr/bin/env python3
"""
PX4 Oracle for MTL Distance Calculation
Parses PX4 .ulg log files and calculates MTL distances for all policies
"""

import sys
import math
import pyulog

def get_mode_name(mode_num):
    """Convert PX4 mode number to mode name"""
    mode_map = {
        0: "MANUAL",
        1: "ALTCTL",
        2: "POSCTL",
        3: "AUTO_MISSION",
        4: "AUTO_LOITER",
        5: "AUTO_RTL",
        6: "ACRO",
        7: "OFFBOARD",
        8: "STABILIZED",
        9: "RATTITUDE",
        10: "AUTO_TAKEOFF",
        11: "AUTO_LAND",
        12: "AUTO_FOLLOW_TARGET",
        13: "AUTO_PRECLAND",
        14: "ORBIT",
        15: "AUTO_VTOL_TAKEOFF",
        16: "AUTO_RTL",
        17: "LOITER",
        18: "AUTO_LAND"
    }
    return mode_map.get(mode_num, "UNKNOWN")

def quaternion_to_euler(q_data, idx):
    """Convert quaternion to euler angles"""
    w = q_data['q[0]'][idx]
    x = q_data['q[1]'][idx]
    y = q_data['q[2]'][idx]
    z = q_data['q[3]'][idx]
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def calculate_distance(log_file_path):
    """
    Calculate MTL distance metrics for all PX4 policies from flight log
    Returns a dictionary with policy names as keys and distance values as values
    """
    ulog = pyulog.ULog(log_file_path)
    data_list = ulog.data_list
    
    # State variables
    state = {
        'current_flight_mode': None,
        'previous_flight_mode': None,
        'current_altitude': 0,
        'previous_altitude': 0,
        'home_altitude': 0,
        'current_alt': 0,
        'lat_current': 0,
        'lat_previous': 0,
        'lon_current': 0,
        'lon_previous': 0,
        'lat_avg': 0,
        'lon_avg': 0,
        'home_lat': 0,
        'home_lon': 0,
        'current_roll': 0,
        'current_pitch': 0,
        'current_heading': 0,
        'armed': 0,
        'parachute_on': 0,
        'goal_throttle': 1500,
        'current_rc_1': 1500,
        'current_rc_2': 1500,
        'current_rc_3': 1500,
        'current_rc_4': 1500,
        'vertical_speed': 0,
        'ground_speed': 0,
        'yawspeed_current': 0,
        'yawspeed_previous': 0,
        'rollspeed_current': 0,
        'rollspeed_previous': 0,
        'pitchspeed_current': 0,
        'pitchspeed_previous': 0,
        'circle_radius_current': 0,
        'circle_radius_previous': 0,
        'num_gps': 0,
        'gps_status': 1,
        'gps_failsafe_error': 0,
        'takeoff': 0,
        'alt_avg': 0,
        'alt_gps_avg': 0,
    }
    
    # Parameters
    params = {}
    
    # Counters
    gps_failsafe_count = 0
    
    # Parse log file and extract state
    altitude_samples = []
    roll_samples = []
    pitch_samples = []
    heading_samples = []
    gps_samples = []
    pos_samples = []
    
    for d in data_list:
        # Parse GPS position data
        if d.name == 'vehicle_gps_position':
            for i in range(len(d.data['timestamp'])):
                lat = d.data['latitude_deg'][i]
                lon = d.data['longitude_deg'][i]
                alt = d.data['altitude_msl_m'][i]
                state['lat_current'] = lat
                state['lon_current'] = lon
                state['current_altitude'] = alt
                pos_samples.append((lat, lon))
                gps_samples.append(alt)
        
        # Parse attitude data
        elif d.name == 'vehicle_attitude':
            for i in range(len(d.data['timestamp'])):
                roll, pitch, yaw = quaternion_to_euler(d.data, i)
                state['current_roll'] = roll
                state['current_pitch'] = pitch
                state['current_heading'] = yaw
                roll_samples.append(roll)
                pitch_samples.append(pitch)
                heading_samples.append(yaw)
        
        # Parse local position data
        elif d.name == 'vehicle_local_position':
            for i in range(len(d.data['timestamp'])):
                if 'z' in d.data:
                    altitude_samples.append(-d.data['z'][i])  # z is negative up
                if 'vz' in d.data:
                    state['vertical_speed'] = -d.data['vz'][i] * 100  # Convert to cm/s
                if 'vxy' in d.data:
                    state['ground_speed'] = math.sqrt(d.data['vx'][i]**2 + d.data['vy'][i]**2) if 'vx' in d.data and 'vy' in d.data else 0
        
        # Parse vehicle status
        elif d.name == 'vehicle_status':
            for i in range(len(d.data['timestamp'])):
                if 'nav_state' in d.data:
                    state['previous_flight_mode'] = state['current_flight_mode']
                    nav_state = d.data['nav_state'][i]
                    # Map nav_state to mode name
                    if nav_state == 0:
                        state['current_flight_mode'] = "MANUAL"
                    elif nav_state == 1:
                        state['current_flight_mode'] = "ALTCTL"
                    elif nav_state == 2:
                        state['current_flight_mode'] = "POSCTL"
                    elif nav_state == 3:
                        state['current_flight_mode'] = "AUTO_MISSION"
                    elif nav_state == 4:
                        state['current_flight_mode'] = "AUTO_LOITER"
                    elif nav_state == 5:
                        state['current_flight_mode'] = "AUTO_RTL"
                    elif nav_state == 6:
                        state['current_flight_mode'] = "ACRO"
                    elif nav_state == 14:
                        state['current_flight_mode'] = "ORBIT"
                    elif nav_state == 17:
                        state['current_flight_mode'] = "LOITER"
                    elif nav_state == 18:
                        state['current_flight_mode'] = "LAND"
                    else:
                        state['current_flight_mode'] = "UNKNOWN"
                
                if 'arming_state' in d.data:
                    state['armed'] = 1 if d.data['arming_state'][i] == 2 else 0
        
        # Parse manual control setpoint (RC inputs)
        elif d.name == 'manual_control_setpoint':
            for i in range(len(d.data['timestamp'])):
                if 'x' in d.data:
                    state['current_rc_2'] = int(d.data['x'][i] * 500 + 1500)  # Pitch
                if 'y' in d.data:
                    state['current_rc_1'] = int(d.data['y'][i] * 500 + 1500)  # Roll
                if 'z' in d.data:
                    state['current_rc_3'] = int(d.data['z'][i] * 500 + 1500)  # Throttle
                if 'r' in d.data:
                    state['current_rc_4'] = int(d.data['r'][i] * 500 + 1500)  # Yaw
        
        # Parse rate setpoint (angular rates)
        elif d.name == 'vehicle_rates_setpoint':
            for i in range(len(d.data['timestamp'])):
                state['rollspeed_previous'] = state['rollspeed_current']
                state['pitchspeed_previous'] = state['pitchspeed_current']
                state['yawspeed_previous'] = state['yawspeed_current']
                if 'roll' in d.data:
                    state['rollspeed_current'] = d.data['roll'][i]
                if 'pitch' in d.data:
                    state['pitchspeed_current'] = d.data['pitch'][i]
                if 'yaw' in d.data:
                    state['yawspeed_current'] = d.data['yaw'][i]
        
        # Parse parameters
        elif d.name == 'parameter':
            for key, value in d.data.items():
                if key != 'timestamp':
                    params[key] = value[0] if len(value) > 0 else 0
    
    # Calculate averages
    if len(altitude_samples) > 0:
        state['alt_avg'] = sum(altitude_samples) / len(altitude_samples)
        state['home_altitude'] = altitude_samples[0]
        state['previous_altitude'] = altitude_samples[0] if len(altitude_samples) > 1 else 0
        state['current_altitude'] = altitude_samples[-1]
    
    if len(roll_samples) > 0:
        state['roll_avg'] = sum(roll_samples) / len(roll_samples)
    if len(pitch_samples) > 0:
        state['pitch_avg'] = sum(pitch_samples) / len(pitch_samples)
    if len(gps_samples) > 0:
        state['alt_gps_avg'] = sum(gps_samples) / len(gps_samples)
    if len(pos_samples) > 0:
        lats = [p[0] for p in pos_samples]
        lons = [p[1] for p in pos_samples]
        state['lat_avg'] = sum(lats) / len(lats)
        state['lon_avg'] = sum(lons) / len(lons)
        state['home_lat'] = pos_samples[0][0]
        state['home_lon'] = pos_samples[0][1]
    
    state['current_alt'] = state['current_altitude'] - state['home_altitude']
    
    # Round lat/lon for comparison (like PGFuzz)
    state['lat_avg'] = round(state['lat_avg'] * 1000) / 1000
    state['lon_avg'] = round(state['lon_avg'] * 1000) / 1000
    state['home_lat'] = round(state['home_lat'] * 1000) / 1000
    state['home_lon'] = round(state['home_lon'] * 1000) / 1000
    
    # Calculate distances for all policies
    distances = {}
    
    # PX.CHUTE Policy
    P = [0] * 5
    if state['parachute_on'] == 1:
        P[0] = 1
    else:
        P[0] = -1
    if state['armed'] == 0:
        P[1] = 1
    else:
        P[1] = -1
    if state['current_flight_mode'] == "FLIP" or state['current_flight_mode'] == "ACRO":
        P[2] = 1
    else:
        P[2] = -1
    if state['current_alt'] > 0:
        P[3] = (state['current_alt'] - state['previous_altitude']) / state['current_alt']
    else:
        P[3] = 0
    chute_alt_min = params.get('CHUTE_ALT_MIN', 0)
    if chute_alt_min > 0:
        P[4] = (chute_alt_min - state['current_alt']) / chute_alt_min
    else:
        P[4] = 0
    distances['PX.CHUTE'] = -1 * (min(P[0], max(P[1], P[2], P[3], P[4])))
    
    # PX.RTL1 Policy
    P = [0] * 4
    tolerance = 0.001
    if abs(state['lat_avg'] - state['home_lat']) <= tolerance and abs(state['lon_avg'] - state['home_lon']) <= tolerance:
        P[0] = -1
    else:
        P[0] = 1
    rtl_alt = params.get('RTL_RETURN_ALT', 0) / 100  # Convert cm to m
    if rtl_alt > 0:
        P[1] = (rtl_alt - state['current_alt']) / rtl_alt
    else:
        P[1] = 0
    if state['current_flight_mode'] == "RTL" or state['current_flight_mode'] == "AUTO_RTL":
        P[2] = 1
    else:
        P[2] = -1
    if state['previous_altitude'] != 0:
        P[3] = (state['previous_altitude'] - state['current_alt']) / state['previous_altitude']
    else:
        P[3] = 0
    distances['PX.RTL1'] = -1 * (min(P[0], P[1], P[2], P[3]))
    
    # PX.ALTITUDE Policy
    P = [0] * 3
    if state['current_flight_mode'] == "ALTCTL":
        P[0] = 1
    else:
        P[0] = -1
    if abs(state['goal_throttle'] - 1500) <= 10:
        P[1] = 1
    else:
        P[1] = -1
    altitude_tolerance = 0.5
    if abs(state['current_alt'] - state['previous_altitude']) <= altitude_tolerance:
        P[2] = -1
    else:
        P[2] = 1
    distances['PX.ALTITUDE'] = -1 * (min(P[0], P[1], P[2]))
    
    # PX.HOLD1 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "LOITER":
        P[0] = 1
    else:
        P[0] = -1
    yaw_tolerance = 0.5
    if abs(state['yawspeed_current'] - state['yawspeed_previous']) <= yaw_tolerance:
        P[1] = -1
    else:
        P[1] = 1
    if abs(state['ground_speed']) <= 0.5:
        P[2] = -1
    else:
        P[2] = 1
    if abs(state['current_altitude'] - state['previous_altitude']) <= altitude_tolerance:
        P[3] = -1
    else:
        P[3] = 1
    distances['PX.HOLD1'] = -1 * (min(P[0], max(P[1], P[2], P[3])))
    
    # PX.HOLD2 Policy
    P = [0] * 3
    if state['current_flight_mode'] == "LOITER":
        P[0] = 1
    else:
        P[0] = -1
    mis_ltrmin_alt = params.get('MIS_LTRMIN_ALT', -1)
    if mis_ltrmin_alt != -1:
        P[1] = 1
    else:
        P[1] = -1
    if state['previous_altitude'] == 0 or abs(state['previous_altitude'] - state['current_altitude']) <= altitude_tolerance:
        P[2] = 0
    else:
        P[2] = (state['previous_altitude'] - state['current_altitude']) / state['previous_altitude']
    distances['PX.HOLD2'] = -1 * (min(P[0], P[1], P[2]))
    
    # PX.POSITION1 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "POSCTL":
        P[0] = 1
    else:
        P[0] = -1
    if abs(state['ground_speed']) <= 0.5:
        P[1] = -1
    else:
        P[1] = 1
    distances['PX.POSITION1'] = -1 * (min(P[0], P[1]))
    
    # PX.TAKEOFF1 Policy
    P = [0] * 2
    if state['takeoff'] == 1:
        P[0] = 1
    else:
        P[0] = -1
    mis_takeoff_alt = params.get('MIS_TAKEOFF_ALT', 0)
    if state['current_alt'] == 0 or abs(state['current_alt'] - mis_takeoff_alt) <= altitude_tolerance:
        P[1] = 0
    else:
        P[1] = (state['current_alt'] - mis_takeoff_alt) / state['current_alt']
    distances['PX.TAKEOFF1'] = -1 * (min(P[0], P[1]))
    
    # PX.TAKEOFF2 Policy
    P = [0] * 2
    if state['takeoff'] == 1:
        P[0] = 1
    else:
        P[0] = -1
    mpc_tko_speed = params.get('MPC_TKO_SPEED', 0)
    if abs(state['vertical_speed'] / 100 - mpc_tko_speed) <= 0.5:
        P[1] = -1
    else:
        P[1] = 1
    distances['PX.TAKEOFF2'] = -1 * (min(P[0], P[1]))
    
    # PX.LAND1 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "LAND":
        P[0] = 1
    else:
        P[0] = -1
    mpc_land_speed = params.get('MPC_LAND_SPEED', 0)
    if abs(abs(state['vertical_speed'] / 100) - abs(mpc_land_speed)) <= 0.5:
        P[1] = -1
    else:
        P[1] = 1
    distances['PX.LAND1'] = -1 * (min(P[0], P[1]))
    
    # PX.ORBIT6 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "ORBIT":
        P[0] = 1
    else:
        P[0] = -1
    if state['ground_speed'] == 0 or abs(state['ground_speed'] - 2) <= 0.5:
        P[1] = 0
    else:
        P[1] = (state['ground_speed'] - 2) / state['ground_speed']
    distances['PX.ORBIT6'] = -1 * (min(P[0], P[1]))
    
    # PX.ORBIT1 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "ORBIT":
        P[0] = 1
    else:
        P[0] = -1
    if abs(state['current_rc_2'] - 1500) <= 10:
        P[1] = 0
    else:
        P[1] = (1500 - state['current_rc_2']) / 1500
    if state['circle_radius_current'] > 0:
        P[2] = 1
    else:
        P[2] = -1
    if state['circle_radius_previous'] == 0 or abs(state['circle_radius_previous'] - state['circle_radius_current']) <= 0.1:
        P[3] = 0
    else:
        P[3] = (state['circle_radius_previous'] - state['circle_radius_current']) / state['circle_radius_previous']
    distances['PX.ORBIT1'] = -1 * (min(P[0], P[1], P[2], P[3]))
    
    # PX.ORBIT2 Policy
    P = [0] * 3
    if state['current_flight_mode'] == "ORBIT":
        P[0] = 1
    else:
        P[0] = -1
    if state['current_rc_2'] == 0 or abs(state['current_rc_2'] - 1500) <= 10:
        P[1] = 0
    else:
        P[1] = (state['current_rc_2'] - 1500) / state['current_rc_2']
    if state['circle_radius_previous'] == 0 or abs(state['circle_radius_previous'] - state['circle_radius_current']) <= 0.1:
        P[2] = 0
    else:
        P[2] = (state['circle_radius_previous'] - state['circle_radius_current']) / state['circle_radius_previous']
    distances['PX.ORBIT2'] = -1 * (min(P[0], P[1], P[2]))
    
    # PX.ORBIT3 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "ORBIT":
        P[0] = 1
    else:
        P[0] = -1
    if state['current_rc_1'] == 0 or abs(state['current_rc_1'] - 1500) <= 10:
        P[1] = 0
    else:
        P[1] = (state['current_rc_1'] - 1500) / state['current_rc_1']
    if state['circle_radius_current'] > 0:
        P[2] = 1
    else:
        P[2] = -1
    P[3] = -1  # Cannot measure circle speed directly
    distances['PX.ORBIT3'] = -1 * (min(P[0], P[1], P[2], P[3]))
    
    # PX.ORBIT4-5 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "ORBIT":
        P[0] = 1
    else:
        P[0] = -1
    if abs(1500 - state['current_rc_1']) <= 10:
        P[1] = 0
    else:
        P[1] = (1500 - state['current_rc_1']) / 1500
    if state['circle_radius_current'] < 0:
        P[2] = 1
    else:
        P[2] = -1
    P[3] = -1  # Cannot measure circle speed directly
    distances['PX.ORBIT4-5'] = -1 * (min(P[0], P[1], P[2], P[3]))
    
    # PX.RTL5 Policy
    P = [0] * 3
    if state['previous_flight_mode'] == "RTL" or state['previous_flight_mode'] == "AUTO_RTL" and state['current_flight_mode'] == "LAND":
        P[0] = 1
    else:
        P[0] = -1
    if abs(state['current_altitude'] - state['home_altitude']) <= altitude_tolerance:
        P[1] = 1
    else:
        P[1] = 0
    if state['armed'] == 0:
        P[2] = -1
    else:
        P[2] = 1
    distances['PX.RTL5'] = -1 * (min(P[0], P[1], P[2]))
    
    # PX.RTL4 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "RTL" or state['current_flight_mode'] == "AUTO_RTL":
        P[0] = 1
    else:
        P[0] = -1
    rtl_descend_alt = params.get('RTL_DESCEND_ALT', -1)
    if rtl_descend_alt == -1:
        P[1] = 1
    else:
        P[1] = -1
    if abs(state['ground_speed']) <= 0.5:
        P[2] = -1
    else:
        P[2] = 1
    if abs(state['current_altitude'] - state['previous_altitude']) <= altitude_tolerance:
        P[3] = -1
    else:
        P[3] = 1
    distances['PX.RTL4'] = -1 * (min(P[0], P[1], max(P[2], P[3])))
    
    # PX.RTL3 Policy
    P = [0] * 4
    if state['previous_flight_mode'] == "RTL" or state['previous_flight_mode'] == "AUTO_RTL":
        P[0] = 1
    else:
        P[0] = -1
    rtl_descend_alt = params.get('RTL_DESCEND_ALT', 0)
    if state['current_altitude'] == 0 or abs(state['current_altitude'] - rtl_descend_alt) <= altitude_tolerance:
        P[1] = 0
    else:
        P[1] = (state['current_altitude'] - rtl_descend_alt) / state['current_altitude']
    if abs(state['lat_avg'] - state['home_lat']) <= tolerance and abs(state['lon_avg'] - state['home_lon']) <= tolerance:
        P[2] = 1
    else:
        P[2] = -1
    if state['current_flight_mode'] == "LAND":
        P[3] = -1
    else:
        P[3] = 1
    distances['PX.RTL3'] = -1 * (min(P[0], P[1], P[2], P[3]))
    
    # PX.RTL2 Policy
    P = [0] * 5
    if state['current_flight_mode'] == "RTL" or state['current_flight_mode'] == "AUTO_RTL":
        P[0] = 1
    else:
        P[0] = -1
    if state['current_altitude'] == 0 or abs(state['current_altitude'] - rtl_descend_alt) <= altitude_tolerance:
        P[1] = 0
    else:
        P[1] = (state['current_altitude'] - rtl_descend_alt) / state['current_altitude']
    if abs(state['lat_avg'] - state['home_lat']) > tolerance or abs(state['lon_avg'] - state['home_lon']) > tolerance:
        P[2] = 1
    else:
        P[2] = -1
    if abs(state['ground_speed']) > 0.5:
        P[3] = -1
    else:
        P[3] = 1
    if abs(state['current_altitude'] - state['previous_altitude']) <= altitude_tolerance:
        P[4] = -1
    else:
        P[4] = 1
    distances['PX.RTL2'] = -1 * (min(P[0], P[1], P[2], max(P[3], P[4])))
    
    # PX.GPS.FS1 Policy
    P = [0] * 2
    if state['gps_status'] == 0:
        P[0] = 1
    else:
        P[0] = -1
    if state['gps_failsafe_error'] == 1:
        P[1] = -1
    else:
        P[1] = 1
    global_distance = -1 * (min(P[0], P[1]))
    com_pos_fs_delay = params.get('COM_POS_FS_DELAY', 1)
    if (global_distance < 0) and (gps_failsafe_count < com_pos_fs_delay):
        gps_failsafe_count += 1
        global_distance = -1 * global_distance
    distances['PX.GPS.FS1'] = global_distance
    
    # PX.GPS.FS2 Policy
    P = [0] * 3
    if state['gps_failsafe_error'] == 1:
        P[0] = 1
    else:
        P[0] = -1
    if (state['current_rc_1'] != 0) or (state['current_rc_2'] != 0) or (state['current_rc_3'] != 0) or (state['current_rc_4'] != 0):
        P[1] = 1
    else:
        P[1] = -1
    if state['current_flight_mode'] == "ALTCTL":
        P[2] = -1
    else:
        P[2] = 1
    distances['PX.GPS.FS2'] = -1 * (min(P[0], P[1], P[2]))
    
    # PX.GPS.FS3 Policy
    P = [0] * 3
    if state['gps_failsafe_error'] == 1:
        P[0] = 1
    else:
        P[0] = -1
    if (state['current_rc_1'] == 0) and (state['current_rc_2'] == 0) and (state['current_rc_3'] == 0) and (state['current_rc_4'] == 0):
        P[1] = 1
    else:
        P[1] = -1
    if state['current_flight_mode'] == "LAND":
        P[2] = -1
    else:
        P[2] = 1
    distances['PX.GPS.FS3'] = -1 * (min(P[0], P[1], P[2]))
    
    # PX_RTL_P4 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "RTL" or state['current_flight_mode'] == "AUTO_RTL":
        P[0] = 1
    else:
        P[0] = -1
    P[1] = -1  # Cannot directly measure target_alt
    distances['PX_RTL_P4'] = -1 * min(P[0], P[1])
    
    # PX_RTL_P5 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "RTL" or state['current_flight_mode'] == "AUTO_RTL":
        P[0] = 1
    else:
        P[0] = -1
    P[1] = -1  # Cannot directly measure target_alt
    distances['PX_RTL_P5'] = -1 * min(P[0], P[1])
    
    # PX_RTL_P6 Policy
    P = [0] * 3
    if state['current_flight_mode'] == "RTL" or state['current_flight_mode'] == "AUTO_RTL":
        P[0] = 1
    else:
        P[0] = -1
    P[1] = -1  # Cannot directly measure RTL_CONE_ANG
    P[2] = -1  # Cannot directly measure target_alt
    distances['PX_RTL_P6'] = -1 * min(P[0], P[1], P[2])
    
    # PX_ORBIT_P6 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "ORBIT":
        P[0] = 1
    else:
        P[0] = -1
    if abs(state['circle_radius_current']) < 1:
        P[1] = (1 - abs(state['circle_radius_current'])) / 1
    else:
        P[1] = -1
    distances['PX_ORBIT_P6'] = -1 * min(P[0], P[1])
    
    # PX_ORBIT_P7 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "ORBIT":
        P[0] = 1
    else:
        P[0] = -1
    mc_orbit_rad_max = params.get('MC_ORBIT_RAD_MAX', 100)
    if abs(state['circle_radius_current']) > mc_orbit_rad_max:
        P[1] = (abs(state['circle_radius_current']) - mc_orbit_rad_max) / abs(state['circle_radius_current'])
    else:
        P[1] = -1
    distances['PX_ORBIT_P7'] = -1 * min(P[0], P[1])
    
    # PX_ALTITUDE_P2 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "ALTCTL":
        P[0] = 1
    else:
        P[0] = -1
    if state['goal_throttle'] <= 1500:
        P[1] = 1
    else:
        P[1] = -1
    if state['vertical_speed'] > 0:
        P[2] = 1
    else:
        P[2] = -1
    P[3] = -1  # Cannot monitor MPC_Z_VEL_MAX_UP precisely
    distances['PX_ALTITUDE_P2'] = -1 * min(P[0], P[1], max(P[2], P[3]))
    
    # PX_ALTITUDE_P3 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "ALTCTL":
        P[0] = 1
    else:
        P[0] = -1
    if state['goal_throttle'] >= 1500:
        P[1] = 1
    else:
        P[1] = -1
    if state['vertical_speed'] < 0:
        P[2] = 1
    else:
        P[2] = -1
    P[3] = -1  # Cannot monitor MPC_Z_VEL_MAX_DN precisely
    distances['PX_ALTITUDE_P3'] = -1 * min(P[0], P[1], max(P[2], P[3]))
    
    # PX_POSITION_P2 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "POSCTL":
        P[0] = 1
    else:
        P[0] = -1
    if state['current_rc_2'] <= 2000:
        P[1] = 1
    else:
        P[1] = -1
    if state['current_rc_2'] >= 1000:
        P[2] = 1
    else:
        P[2] = -1
    P[3] = -1  # Cannot monitor x_velocity_setpoint
    distances['PX_POSITION_P2'] = -1 * min(P[0], P[1], P[2], P[3])
    
    # PX_POSITION_P3 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "POSCTL":
        P[0] = 1
    else:
        P[0] = -1
    if state['current_rc_1'] <= 2000:
        P[1] = 1
    else:
        P[1] = -1
    if state['current_rc_1'] >= 1000:
        P[2] = 1
    else:
        P[2] = -1
    P[3] = -1  # Cannot monitor y_velocity_setpoint
    distances['PX_POSITION_P3'] = -1 * min(P[0], P[1], P[2], P[3])
    
    # PX_RCFS_P1 Policy
    P = [0] * 3
    if (state['current_rc_1'] != 0) or (state['current_rc_2'] != 0) or (state['current_rc_3'] != 0) or (state['current_rc_4'] != 0):
        P[0] = 1
    else:
        P[0] = -1
    P[1] = -1  # Cannot monitor manual_control_update_time
    P[2] = -1  # Cannot monitor manual_control_loss_failsafe
    distances['PX_RCFS_P1'] = -1 * min(P[0], max(P[1], P[2]))
    
    # PX_BRAKE_P1 Policy
    P = [0] * 3
    P[0] = -1  # Cannot monitor Emergency_Braking_activated
    P[1] = -1  # Cannot monitor max_vertical_speed
    P[2] = -1  # Cannot monitor max_vertical_speed
    distances['PX_BRAKE_P1'] = -1 * min(P[0], max(P[1], P[2]))
    
    return distances

def get_mode_name_from_nav_state(nav_state):
    """Convert nav_state to flight mode name"""
    mode_map = {
        0: "MANUAL",
        1: "ALTCTL",
        2: "POSCTL",
        3: "AUTO_MISSION",
        4: "LOITER",
        5: "RTL",
        6: "ACRO",
        7: "OFFBOARD",
        8: "STABILIZED",
        9: "RATTITUDE",
        10: "AUTO_TAKEOFF",
        11: "AUTO_LAND",
        12: "AUTO_FOLLOW_TARGET",
        13: "AUTO_PRECLAND",
        14: "ORBIT",
        15: "AUTO_VTOL_TAKEOFF",
        16: "AUTO_RTL",
        17: "LOITER",
        18: "LAND"
    }
    return mode_map.get(nav_state, "UNKNOWN")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python px4_oracle.py <log_file.ulg>")
        sys.exit(1)
    
    log_file = sys.argv[1]
    
    try:
        distances = calculate_distance(log_file)
        
        print("\n" + "="*60)
        print("PX4 Policy Distance Calculation Results")
        print("="*60)
        print(f"{'Policy':<20s}  {'Distance':>10s}  {'Status'}")
        print("-"*60)
        
        for policy, distance in sorted(distances.items()):
            status = "Violated" if distance < 0 else "Satisfied"
            print(f"{policy:<20s}: {distance:8.4f}  [{status}]")
        
        print("="*60 + "\n")
        
    except Exception as e:
        print(f"Error processing log file: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
