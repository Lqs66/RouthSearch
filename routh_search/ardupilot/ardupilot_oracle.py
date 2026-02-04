import math
import sys
from pymavlink import mavutil
from math import radians, sin, cos, sqrt, atan2


def calculate_distance(log_file_path):
    """
    Calculate MTL distance metrics for all ArduPilot policies from flight log
    Returns a dictionary with policy names as keys and distance values as values
    """
    log_file = mavutil.mavlink_connection(log_file_path)
    
    # State variables
    state = {
        'current_flight_mode': None,
        'previous_flight_mode': None,
        'current_altitude': 0,
        'previous_altitude': 0,
        'home_altitude': 0,
        'current_alt': 0,
        'relative_alt': 0,
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
        'roll_avg': 0,
        'pitch_avg': 0,
        'roll_avg_flip': 0,
        'roll_initial': 0,
        'pitch_initial': 0,
        'yaw_initial': 0,
        'armed': 0,
        'parachute_on': 0,
        'actual_throttle': 0,
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
        'failsafe_error': 0,
        'rc_failsafe_error': 0,
        'takeoff': 0,
        'mission_cnt': 0,
        'alt_avg': 0,
        'alt_gps_avg': 0,
    }
    
    # Parameters
    params = {
        'chute_alt_min': 0,
        'rtl_alt': 0,
        'fs_thr_val': 0,
        'pilot_speed_up': 0,
        'expected_landing_speed': 0,
        'alt_source': 0,
        'sim_baro_disable': 0,
        'wpnav_rfnd_use': 0,
        'rangefinder_type': 0,
        'alt_source_type': 0,
        'angle_max': 0,
        'loit_speed': 0,
        'expected_flight_mode_from_fs': 0,
        'loit_acc_max': 0,
        'acro_trainer': 0,
        'land_speed': 0,
        'land_speed_high': 0,
        'wpnav_speed_dn': 0,
        'ek2_alt_source': 0,
        'rngfnd_type': 0,
    }
    
    # Counters
    gps_failsafe_cnt = 0
    brake_cnt = 0
    
    # Parse log file and extract state
    altitude_samples = []
    roll_samples = []
    pitch_samples = []
    heading_samples = []
    gps_samples = []
    pos_samples = []
    
    while True:
        msg = log_file.recv_msg()
        if msg is None:
            break
        
        msg_dict = msg.to_dict()
        msg_type = msg.get_type()
        
        # Parse attitude data
        if msg_type == 'ATT':
            state['current_roll'] = msg_dict.get('Roll', 0)
            state['current_pitch'] = msg_dict.get('Pitch', 0)
            state['current_heading'] = msg_dict.get('Yaw', 0)
            roll_samples.append(state['current_roll'])
            pitch_samples.append(state['current_pitch'])
            heading_samples.append(state['current_heading'])
        
        # Parse position data
        elif msg_type == 'POS':
            state['lat_current'] = msg_dict.get('Lat', 0)
            state['lon_current'] = msg_dict.get('Lng', 0)
            state['current_altitude'] = msg_dict.get('Alt', 0)
            state['relative_alt'] = msg_dict.get('RelHomeAlt', 0)
            altitude_samples.append(state['current_altitude'])
            pos_samples.append((state['lat_current'], state['lon_current']))
        
        # Parse mode changes
        elif msg_type == 'MODE':
            state['previous_flight_mode'] = state['current_flight_mode']
            mode_num = msg_dict.get('Mode', 0)
            state['current_flight_mode'] = get_mode_name(mode_num)
        
        # Parse GPS data
        elif msg_type == 'GPS':
            state['num_gps'] = msg_dict.get('NSats', 0)
            gps_alt = msg_dict.get('Alt', 0)
            gps_samples.append(gps_alt)
        
        # Parse RC channels
        elif msg_type == 'RCIN':
            state['current_rc_1'] = msg_dict.get('C1', 1500)
            state['current_rc_2'] = msg_dict.get('C2', 1500)
            state['current_rc_3'] = msg_dict.get('C3', 1500)
            state['current_rc_4'] = msg_dict.get('C4', 1500)
        
        # Parse armed status
        elif msg_type == 'STAT':
            state['armed'] = msg_dict.get('Armed', 0)
        
        # Parse velocity
        elif msg_type == 'VFR_HUD':
            state['ground_speed'] = msg_dict.get('Spd', 0)
            state['vertical_speed'] = msg_dict.get('ClimbRate', 0)
            state['actual_throttle'] = msg_dict.get('Thr', 0)
        
        # Parse rate data
        elif msg_type == 'RATE':
            state['rollspeed_previous'] = state['rollspeed_current']
            state['pitchspeed_previous'] = state['pitchspeed_current']
            state['yawspeed_previous'] = state['yawspeed_current']
            state['rollspeed_current'] = msg_dict.get('RDes', 0)
            state['pitchspeed_current'] = msg_dict.get('PDes', 0)
            state['yawspeed_current'] = msg_dict.get('YDes', 0)
        
        # Parse parameters
        elif msg_type == 'PARM':
            parm_name = msg_dict.get('Name', '')
            parm_value = msg_dict.get('Value', 0)
            if parm_name == 'CHUTE_ALT_MIN':
                params['chute_alt_min'] = parm_value
            elif parm_name == 'RTL_ALT':
                params['rtl_alt'] = parm_value
            elif parm_name == 'FS_THR_VALUE':
                params['fs_thr_val'] = parm_value
            elif parm_name == 'PILOT_SPEED_UP':
                params['pilot_speed_up'] = parm_value
            elif parm_name == 'EK2_ALT_SOURCE':
                params['alt_source'] = int(parm_value)
                params['ek2_alt_source'] = int(parm_value)
                params['alt_source_type'] = int(parm_value)
            elif parm_name == 'SIM_BARO_DISABLE':
                params['sim_baro_disable'] = int(parm_value)
            elif parm_name == 'WPNAV_RFND_USE':
                params['wpnav_rfnd_use'] = int(parm_value)
            elif parm_name == 'RNGFND_TYPE':
                params['rangefinder_type'] = int(parm_value)
                params['rngfnd_type'] = int(parm_value)
            elif parm_name == 'ANGLE_MAX':
                params['angle_max'] = parm_value
            elif parm_name == 'LOIT_SPEED':
                params['loit_speed'] = parm_value
            elif parm_name == 'FS_EKF_ACTION':
                params['expected_flight_mode_from_fs'] = int(parm_value)
            elif parm_name == 'LOIT_ACC_MAX':
                params['loit_acc_max'] = parm_value
            elif parm_name == 'ACRO_TRAINER':
                params['acro_trainer'] = int(parm_value)
            elif parm_name == 'LAND_SPEED':
                params['land_speed'] = parm_value
            elif parm_name == 'LAND_SPEED_HIGH':
                params['land_speed_high'] = parm_value
            elif parm_name == 'WPNAV_SPEED_DN':
                params['wpnav_speed_dn'] = parm_value
    
    # Calculate averages
    if len(altitude_samples) > 0:
        state['alt_avg'] = sum(altitude_samples) / len(altitude_samples)
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
    
    if len(altitude_samples) > 0:
        state['home_altitude'] = altitude_samples[0]
        state['previous_altitude'] = altitude_samples[0] if len(altitude_samples) > 1 else 0
        state['current_altitude'] = altitude_samples[-1]
    
    state['current_alt'] = state['current_altitude'] - state['home_altitude']
    
    # Calculate distances for all policies
    distances = {}
    
    # A.CHUTE Policy
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
    if params['chute_alt_min'] > 0:
        P[4] = (params['chute_alt_min'] - state['current_alt']) / params['chute_alt_min']
    else:
        P[4] = 0
    distances['A.CHUTE'] = -1 * (min(P[0], max(P[1], P[2], P[3], P[4])))
    
    # A.RTL1 Policy
    P = [0] * 4
    if state['lat_avg'] == state['home_lat'] and state['lon_avg'] == state['home_lon']:
        P[0] = -1
    else:
        P[0] = 1
    rtl_alt_meters = params['rtl_alt'] / 100
    if rtl_alt_meters > 0:
        P[1] = (rtl_alt_meters - state['current_alt']) / rtl_alt_meters
    else:
        P[1] = 0
    if state['current_flight_mode'] == "RTL":
        P[2] = 1
    else:
        P[2] = -1
    if state['previous_altitude'] != 0:
        P[3] = (state['previous_altitude'] - state['current_alt']) / state['previous_altitude']
    else:
        P[3] = 0
    distances['A.RTL1'] = -1 * (min(P[0], P[1], P[2], P[3]))
    
    # A.RTL2 Policy
    P = [0] * 5
    if state['current_flight_mode'] == "RTL":
        P[0] = 1
    else:
        P[0] = -1
    if rtl_alt_meters > 0:
        P[1] = (state['current_alt'] - rtl_alt_meters) / state['current_alt']
    else:
        P[1] = 0
    if state['lat_avg'] != state['home_lat'] and state['lon_avg'] != state['home_lon']:
        P[2] = 1
    else:
        P[2] = -1
    if state['lat_current'] != state['lat_previous'] or state['lon_current'] != state['lon_previous']:
        P[3] = -1
    else:
        P[3] = 1
    altitude_tolerance = 0.5
    if state['previous_altitude'] != 0 and abs(state['previous_altitude'] - state['current_altitude']) <= altitude_tolerance:
        P[4] = -1
    else:
        P[4] = 1
    distances['A.RTL2'] = -1 * (min(P[0], P[1], P[2], max(P[3], P[4])))
    
    # A.RTL3 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "RTL":
        P[0] = 1
    else:
        P[0] = -1
    if rtl_alt_meters > 0:
        P[1] = (state['current_alt'] - rtl_alt_meters) / state['current_alt']
    else:
        P[1] = 0
    if state['lat_avg'] == state['home_lat'] and state['lon_avg'] == state['home_lon']:
        P[2] = 1
    else:
        P[2] = -1
    previous_alt_round = round(state['previous_altitude'], 1)
    current_alt_round = round(state['current_altitude'], 1)
    if previous_alt_round != 0 and current_alt_round != 0:
        P[3] = (current_alt_round - previous_alt_round) / current_alt_round
    else:
        P[3] = 0
    distances['A.RTL3'] = -1 * (min(P[0], P[1], P[2], P[3]))
    
    # A.RTL4 Policy
    P = [0] * 3
    if state['previous_flight_mode'] == "RTL" and state['current_flight_mode'] == "LAND":
        P[0] = 1
    else:
        P[0] = -1
    if round(state['current_altitude'], 1) == round(state['home_altitude'], 1):
        P[1] = 1
    else:
        P[1] = 0
    if state['armed'] == 0:
        P[2] = -1
    else:
        P[2] = 1
    distances['A.RTL4'] = -1 * (min(P[0], P[1], P[2]))
    
    # A.FLIP1 Policy
    P = [0] * 5
    if state['current_flight_mode'] == "FLIP":
        P[0] = 1
    else:
        P[0] = -1
    if state['previous_flight_mode'] == "ACRO" or state['previous_flight_mode'] == "ALT_HOLD":
        P[1] = -1
    else:
        P[1] = 1
    if abs(state['roll_avg']) <= 45:
        P[2] = -1
    else:
        P[2] = 1
    if round(state['current_altitude'], 0) >= round(state['previous_altitude'], 0):
        P[3] = -1
    else:
        P[3] = 1
    if state['current_alt'] >= 10:
        P[4] = -1
    else:
        P[4] = 1
    distances['A.FLIP1'] = -1 * (min(P[0], max(P[1], P[2], P[3], P[4])))
    
    # A.ALTHOLD1 Policy
    P = [0] * 3
    if params['alt_source'] == 0:
        P[0] = 1
    else:
        P[0] = -1
    if round(state['alt_avg'], 1) != round(state['alt_gps_avg'], 1):
        P[1] = -1
        P[2] = -1
    else:
        P[1] = 1
        P[2] = 1
    distances['A.ALT_HOLD1'] = -1 * (min(P[0], max(P[1], P[2])))
    
    # A.ALTHOLD2 Policy
    P = [0] * 3
    if state['current_flight_mode'] == "ALT_HOLD":
        P[0] = 1
    else:
        P[0] = -1
    if abs(state['actual_throttle'] - 34) <= 2:
        P[1] = 1
    else:
        P[1] = -1
    altitude_tolerance = 0.5
    if abs(state['current_altitude'] - state['previous_altitude']) <= altitude_tolerance:
        P[2] = -1
    else:
        P[2] = 1
    distances['A.ALT_HOLD2'] = -1 * (min(P[0], P[1], P[2]))
    
    # A.CIRCLE1 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "CIRCLE":
        P[0] = 1
    else:
        P[0] = -1
    if state['current_rc_2'] == 1500:
        P[1] = 0
    else:
        P[1] = (1500 - state['current_rc_2']) / 1500
    if state['circle_radius_current'] > 0:
        P[2] = 1
    else:
        P[2] = -1
    if state['circle_radius_previous'] == 0 or (state['circle_radius_previous'] - state['circle_radius_current']) == 0:
        P[3] = 0
    else:
        P[3] = (state['circle_radius_previous'] - state['circle_radius_current']) / state['circle_radius_previous']
    distances['A.CIRCLE1'] = -1 * (min(P[0], P[1], P[2], P[3]))
    
    # A.LAND1 Policy
    P = [0] * 3
    if state['current_flight_mode'] == "LAND":
        P[0] = 1
    else:
        P[0] = -1
    if state['relative_alt'] == 0:
        P[1] = -1
    else:
        P[1] = (state['relative_alt'] - 10) / state['relative_alt']
    if abs(state['vertical_speed'] - params['expected_landing_speed']) <= 10:
        P[2] = -1
    else:
        P[2] = 1
    distances['A.LAND1'] = -1 * (min(P[0], P[1], P[2]))
    
    # A.GPS.FS1 Policy
    P = [0] * 3
    gps_modes = ["AUTO", "AUTOTUNE", "BRAKE", "CIRCLE", "DRIFT", "FOLLOW",
                 "GUIDED", "LOITER", "POSHOLD", "RTL", "SIMPLE", "SMART_RTL",
                 "THROW", "ZIGZAG"]
    if state['current_flight_mode'] in gps_modes:
        P[0] = 1
    else:
        P[0] = -1
    if (state['num_gps'] - 4) == 0:
        P[1] = 0
    else:
        P[1] = float((4 - state['num_gps']) / 4.0)
    if state['failsafe_error'] == 1:
        P[2] = -1
    else:
        P[2] = 1
    global_distance = -1 * (min(P[0], P[1], P[2]))
    if (global_distance < 0) and (gps_failsafe_cnt < 2):
        global_distance = -1 * global_distance
    distances['A.GPS.FS1'] = global_distance
    
    # A.BRAKE1 Policy
    P = [0] * 2
    if state['previous_flight_mode'] == "BRAKE":
        P[0] = 1
    else:
        P[0] = -1
    if abs(state['ground_speed']) <= 0.5:
        P[1] = -1
    else:
        P[1] = 1
    global_distance = -1 * (min(P[0], P[1]))
    if (state['previous_flight_mode'] == "BRAKE") and (abs(state['ground_speed']) > 0.5) and (brake_cnt < 2):
        global_distance = -1 * global_distance
    distances['A.BRAKE1'] = global_distance
    
    # A.LOITER1 Policy
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
    altitude_tolerance = 0.5
    if abs(state['current_altitude'] - state['previous_altitude']) <= altitude_tolerance:
        P[3] = -1
    else:
        P[3] = 1
    distances['A.LOITER1'] = -1 * (min(P[0], max(P[1], P[2], P[3])))
    
    # A.FLIP2 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "FLIP":
        P[0] = 1
    else:
        P[0] = -1
    if -90 <= abs(state['roll_avg_flip']) <= 45:
        P[1] = 1
    else:
        P[1] = -1
    P[2] = -1  # Cannot monitor roll rate
    if abs(state['roll_avg_flip']) > 0:
        P[3] = -1
    else:
        P[3] = 1
    distances['A.FLIP2'] = -1 * (min(P[0], P[1], max(P[2], P[3])))
    
    # A.FLIP3 Policy
    P = [0] * 4
    if state['previous_flight_mode'] == "FLIP":
        P[0] = 1
        if round(state['roll_initial'], 0) == round(state['current_roll'], 0):
            P[1] = -1
        else:
            P[1] = 1
        if round(state['pitch_initial'], 0) == round(state['current_pitch'], 0):
            P[2] = -1
        else:
            P[2] = 1
        if (state['yaw_initial'] - state['current_heading']) < 20:
            P[3] = -1
        else:
            P[3] = 1
    else:
        P[0] = -1
        P[1] = 1
        P[2] = 1
        P[3] = 1
    distances['A.FLIP3'] = -1 * (min(P[0], max(P[1], P[2], P[3])))
    
    # A.FLIP4 Policy
    P = [0] * 2
    if state['previous_flight_mode'] == "FLIP":
        P[0] = 1
        # Cannot measure elapsed time precisely in offline log
        P[1] = -1  # Assume it always completes within 2.5 seconds
    else:
        P[0] = -1
        P[1] = 1
    distances['A.FLIP4'] = -1 * (min(P[0], P[1]))
    
    # A.CIRCLE2 Policy
    P = [0] * 3
    if state['current_flight_mode'] == "CIRCLE":
        P[0] = 1
    else:
        P[0] = -1
    if state['current_rc_2'] == 0 or state['current_rc_2'] == 1500:
        P[1] = 0
    else:
        P[1] = (state['current_rc_2'] - 1500) / state['current_rc_2']
    if state['circle_radius_previous'] == 0 or (state['circle_radius_previous'] - state['circle_radius_current']) == 0:
        P[2] = 0
    else:
        P[2] = (state['circle_radius_previous'] - state['circle_radius_current']) / state['circle_radius_previous']
    distances['A.CIRCLE2'] = -1 * (min(P[0], P[1], P[2]))
    
    # A.CIRCLE3 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "CIRCLE":
        P[0] = 1
    else:
        P[0] = -1
    if state['current_rc_1'] == 0 or state['current_rc_1'] == 1500:
        P[1] = 0
    else:
        P[1] = (state['current_rc_1'] - 1500) / state['current_rc_1']
    if state['circle_radius_current'] > 0:
        P[2] = 1
    else:
        P[2] = -1
    P[3] = -1  # Cannot measure circle speed directly
    distances['A.CIRCLE3'] = -1 * (min(P[0], P[1], P[2], P[3]))
    
    # A.CIRCLE4-6 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "CIRCLE":
        P[0] = 1
    else:
        P[0] = -1
    if (1500 - state['current_rc_1']) == 0:
        P[1] = 0
    else:
        P[1] = (1500 - state['current_rc_1']) / 1500
    if state['circle_radius_current'] < 0:
        P[2] = 1
    else:
        P[2] = -1
    P[3] = -1  # Cannot measure circle speed directly
    distances['A.CIRCLE4-6'] = -1 * (min(P[0], P[1], P[2], P[3]))
    
    # A.CIRCLE7 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "CIRCLE":
        P[0] = 1
    else:
        P[0] = -1
    rollspeed_current_round = round(state['rollspeed_current'], 0)
    rollspeed_previous_round = round(state['rollspeed_previous'], 0)
    pitchspeed_current_round = round(state['pitchspeed_current'], 0)
    pitchspeed_previous_round = round(state['pitchspeed_previous'], 0)
    yawspeed_current_round = round(state['yawspeed_current'], 0)
    yawspeed_previous_round = round(state['yawspeed_previous'], 0)
    speed_tolerance = 100.0
    if abs(rollspeed_current_round - rollspeed_previous_round) <= speed_tolerance:
        P[1] = -1
    else:
        P[1] = 1
    if abs(pitchspeed_current_round - pitchspeed_previous_round) <= speed_tolerance:
        P[2] = -1
    else:
        P[2] = 1
    if abs(yawspeed_current_round - yawspeed_previous_round) <= speed_tolerance:
        P[3] = -1
    else:
        P[3] = 1
    distances['A.CIRCLE7'] = -1 * (min(P[0], max(P[1], P[2], P[3])))
    
    # A.LAND2 Policy
    P = [0] * 3
    if state['current_flight_mode'] == "LAND":
        P[0] = 1
    else:
        P[0] = -1
    if state['relative_alt'] == 10:
        P[1] = 0
    elif state['relative_alt'] == 0:
        P[1] = -1
    else:
        P[1] = (10 - state['relative_alt']) / 10
    if abs(state['vertical_speed'] - params['expected_landing_speed']) <= 10:
        P[2] = -1
    else:
        P[2] = 1
    distances['A.LAND2'] = -1 * (min(P[0], P[1], P[2]))
    
    # A.AUTO1 Policy
    P = [0] * 3
    if state['current_flight_mode'] == "AUTO":
        P[0] = 1
    else:
        P[0] = -1
    if abs(state['current_rc_4'] - 1500) > 10:
        P[1] = 1
    else:
        P[1] = -1
    yaw_tolerance = 0.1
    if abs(state['yawspeed_current'] - state['yawspeed_previous']) <= yaw_tolerance:
        P[2] = 1
    else:
        P[2] = -1
    distances['A.AUTO1'] = -1 * (min(P[0], P[1], P[2]))
    
    # A.GPS.FS2 Policy
    P = [0] * 3
    if state['failsafe_error'] == 1:
        P[0] = 1
    else:
        P[0] = -1
    if params['sim_baro_disable'] == 0:
        P[1] = 1
    else:
        P[1] = -1
    if round(state['alt_avg'], 1) != round(state['alt_gps_avg'], 1):
        P[2] = -1
    else:
        P[2] = 1
    distances['A.GPS.FS2'] = -1 * (min(P[0], P[1], P[2]))
    
    # A.RC.FS1 Policy
    P = [0] * 3
    if state['takeoff'] != 1:
        P[0] = 1
    else:
        P[0] = -1
    if (params['fs_thr_val'] == 0) or (params['fs_thr_val'] - state['current_rc_3']) == 0:
        P[1] = 0
    else:
        P[1] = float((params['fs_thr_val'] - state['current_rc_3']) / params['fs_thr_val'])
    if state['armed'] == 0:
        P[2] = -1
    else:
        P[2] = 1
    distances['A.RC.FS1'] = -1 * (min(P[0], P[1], P[2]))
    
    # A.RC.FS2 Policy
    P = [0] * 3
    if (params['fs_thr_val'] == 0) or (params['fs_thr_val'] - state['current_rc_3']) == 0:
        P[0] = 0
    else:
        P[0] = float((params['fs_thr_val'] - state['current_rc_3']) / params['fs_thr_val'])
    if state['rc_failsafe_error'] == 1:
        P[1] = -1
    else:
        P[1] = 1
    P[2] = 1  # Placeholder
    distances['A.RC.FS2'] = -1 * (min(P[0], P[1], P[2]))
    
    # A.SPORT1 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "SPORT":
        P[0] = 1
    else:
        P[0] = -1
    if abs(state['vertical_speed'] - params['pilot_speed_up']) <= 10:
        P[1] = -1
    else:
        P[1] = 1
    distances['A.SPORT1'] = -1 * (min(P[0], P[1]))
    
    # A.GUIDED1 Policy
    P = [0] * 5
    if state['current_flight_mode'] == "GUIDED":
        P[0] = 1
    else:
        P[0] = -1
    if state['mission_cnt'] == 0:
        P[1] = 1
    else:
        P[1] = -1
    yaw_tolerance = 0.5
    if abs(state['yawspeed_current'] - state['yawspeed_previous']) <= yaw_tolerance:
        P[2] = -1
    else:
        P[2] = 1
    if abs(state['ground_speed']) <= 0.5:
        P[3] = -1
    else:
        P[3] = 1
    altitude_tolerance = 0.5
    if abs(state['current_altitude'] - state['previous_altitude']) <= altitude_tolerance:
        P[4] = -1
    else:
        P[4] = 1
    distances['A.GUIDED1'] = -1 * (min(P[0], P[1], max(P[2], P[3], P[4])))
    
    # A.DRIFT1 Policy
    P = [0] * 3
    if state['failsafe_error'] == 1:
        P[0] = 1
    else:
        P[0] = -1
    if state['previous_flight_mode'] == "DRIFT":
        P[1] = 1
    else:
        P[1] = -1
    expected_flight_mode_from_fs = params['expected_flight_mode_from_fs']
    if (expected_flight_mode_from_fs == 1) or (expected_flight_mode_from_fs == 3):
        if state['current_flight_mode'] == "LAND":
            P[2] = -1
        else:
            P[2] = 1
    elif expected_flight_mode_from_fs == 2:
        if state['current_flight_mode'] == "ALT_HOLD":
            P[2] = -1
        else:
            P[2] = 1
    else:
        P[2] = -1
    distances['A.DRIFT1'] = -1 * (min(P[0], P[1], P[2]))
    
    # AP_RTL_P1 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "RTL":
        P[0] = 1
    else:
        P[0] = -1
    P[1] = -1  # Cannot directly observe target_alt
    distances['AP_RTL_P1'] = -1 * min(P[0], P[1])
    
    # AP_RTL_P2 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "RTL":
        P[0] = 1
    else:
        P[0] = -1
    P[1] = -1  # Cannot directly observe target_alt
    distances['AP_RTL_P2'] = -1 * min(P[0], P[1])
    
    # AP_RTL_P4 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "RTL":
        P[0] = 1
    else:
        P[0] = -1
    if params['wpnav_rfnd_use'] == 1:
        P[1] = 1
    else:
        P[1] = -1
    if params['rangefinder_type'] > 0:
        P[2] = 1
    else:
        P[2] = -1
    if params['alt_source_type'] != 1:
        P[3] = 1
    else:
        P[3] = -1
    distances['AP_RTL_P4'] = -1 * min(P[0], P[1], P[2], P[3])
    
    # AP_LAND_P4 Policy
    P = [0] * 3
    if state['current_flight_mode'] == "LAND":
        P[0] = 1
    else:
        P[0] = -1
    if state['actual_throttle'] == 0:
        P[1] = 0
    else:
        P[1] = -1
    if state['armed'] == 1:
        P[2] = 1
    else:
        P[2] = -1
    distances['AP_LAND_P4'] = -1 * min(P[0], P[1], P[2])
    
    # AP_ALTHOLD_P2 Policy
    P = [0] * 3
    if state['current_flight_mode'] == "ALT_HOLD":
        P[0] = 1
    else:
        P[0] = -1
    if state['current_rc_3'] >= 1000:
        P[1] = 1
    else:
        P[1] = -1
    if state['vertical_speed'] <= 0:
        P[2] = 1
    elif state['vertical_speed'] > params['pilot_speed_up']:
        P[2] = (state['vertical_speed'] - params['pilot_speed_up']) / state['vertical_speed']
    else:
        P[2] = -1
    distances['AP_ALTHOLD_P2'] = -1 * min(P[0], P[1], P[2])
    
    # AP_POSHOLD_P1 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "POSHOLD":
        P[0] = 1
    else:
        P[0] = -1
    angle_max = params['angle_max'] / 100
    if abs(state['current_roll']) > angle_max:
        P[1] = (abs(state['current_roll']) - angle_max) / abs(state['current_roll'])
    else:
        P[1] = -1
    distances['AP_POSHOLD_P1'] = -1 * min(P[0], P[1])
    
    # AP_POSHOLD_P2 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "POSHOLD":
        P[0] = 1
    else:
        P[0] = -1
    if abs(state['current_pitch']) > angle_max:
        P[1] = (abs(state['current_pitch']) - angle_max) / abs(state['current_pitch'])
    else:
        P[1] = -1
    distances['AP_POSHOLD_P2'] = -1 * min(P[0], P[1])
    
    # AP_POSHOLD_P3 Policy
    P = [0] * 3
    if state['current_flight_mode'] == "POSHOLD":
        P[0] = 1
    else:
        P[0] = -1
    if state['current_rc_3'] >= 1000:
        P[1] = 1
    else:
        P[1] = -1
    if state['vertical_speed'] <= 0:
        P[2] = 1
    elif state['vertical_speed'] > params['pilot_speed_up']:
        P[2] = (state['vertical_speed'] - params['pilot_speed_up']) / state['vertical_speed']
    else:
        P[2] = -1
    distances['AP_POSHOLD_P3'] = -1 * min(P[0], P[1], P[2])
    
    # AP_DRIFT_P1 Policy
    P = [0] * 3
    if state['current_flight_mode'] == "DRIFT":
        P[0] = 1
    else:
        P[0] = -1
    if state['current_rc_3'] == 1500:
        P[1] = 1
    else:
        P[1] = -1
    P[2] = -1  # Cannot monitor motor spin rate
    distances['AP_DRIFT_P1'] = -1 * min(P[0], P[1], P[2])
    
    # AP_DRIFT_P3 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "DRIFT":
        P[0] = 1
    else:
        P[0] = -1
    if abs(state['current_roll']) > 45:
        P[1] = (abs(state['current_roll']) - 45) / abs(state['current_roll'])
    else:
        P[1] = -1
    distances['AP_DRIFT_P3'] = -1 * min(P[0], P[1])
    
    # AP_LOITER_P2 Policy
    P = [0] * 3
    if state['current_flight_mode'] == "LOITER":
        P[0] = 1
    else:
        P[0] = -1
    if state['current_rc_3'] >= 1000:
        P[1] = 1
    else:
        P[1] = -1
    if state['vertical_speed'] <= 0:
        P[2] = 1
    elif state['vertical_speed'] > params['pilot_speed_up']:
        P[2] = (state['vertical_speed'] - params['pilot_speed_up']) / state['vertical_speed']
    else:
        P[2] = -1
    distances['AP_LOITER_P2'] = -1 * min(P[0], P[1], P[2])
    
    # AP_LOITER_P3 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "LOITER":
        P[0] = 1
    else:
        P[0] = -1
    loit_speed = params['loit_speed'] / 100
    if state['ground_speed'] > loit_speed:
        P[1] = (state['ground_speed'] - loit_speed) / state['ground_speed']
    else:
        P[1] = -1
    distances['AP_LOITER_P3'] = -1 * min(P[0], P[1])
    
    # AP_LOITER_P4 Policy
    P = [0] * 3
    if state['current_flight_mode'] == "LOITER":
        P[0] = 1
    else:
        P[0] = -1
    # Read loit_acc_max parameter from params dict if available
    loit_acc_max = params.get('loit_acc_max', 0)
    if loit_acc_max != 0:
        P[1] = 1
    else:
        P[1] = -1
    P[2] = -1  # Cannot monitor acceleration
    distances['AP_LOITER_P4'] = -1 * min(P[0], P[1], P[2])
    
    # AP_ACRO_P1 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "ACRO":
        P[0] = 1
    else:
        P[0] = -1
    acro_trainer = params.get('acro_trainer', 0)
    if acro_trainer >= 1:
        P[1] = 1
    else:
        P[1] = -1
    if abs(state['current_roll']) != 0:
        P[2] = 1
    else:
        P[2] = -1
    P[3] = -1  # Cannot monitor roll angle change precisely
    distances['AP_ACRO_P1'] = -1 * min(P[0], P[1], P[2], P[3])
    
    # AP_ACRO_P2 Policy
    P = [0] * 4
    if state['current_flight_mode'] == "ACRO":
        P[0] = 1
    else:
        P[0] = -1
    if acro_trainer >= 1:
        P[1] = 1
    else:
        P[1] = -1
    if abs(state['current_pitch']) != 0:
        P[2] = 1
    else:
        P[2] = -1
    P[3] = -1  # Cannot monitor pitch angle change precisely
    distances['AP_ACRO_P2'] = -1 * min(P[0], P[1], P[2], P[3])
    
    # AP_STABILIZE_P1 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "STABILIZE":
        P[0] = 1
    else:
        P[0] = -1
    if abs(state['current_roll']) > angle_max:
        P[1] = (abs(state['current_roll']) - angle_max) / abs(state['current_roll'])
    else:
        P[1] = -1
    distances['AP_STABILIZE_P1'] = -1 * min(P[0], P[1])
    
    # AP_STABILIZE_P2 Policy
    P = [0] * 2
    if state['current_flight_mode'] == "STABILIZE":
        P[0] = 1
    else:
        P[0] = -1
    if abs(state['current_pitch']) > angle_max:
        P[1] = (abs(state['current_pitch']) - angle_max) / abs(state['current_pitch'])
    else:
        P[1] = -1
    distances['AP_STABILIZE_P2'] = -1 * min(P[0], P[1])
    
    # AP_ZIGZAG_P1 Policy
    P = [0] * 3
    if state['current_flight_mode'] == "ZIGZAG":
        P[0] = 1
    else:
        P[0] = -1
    if state['current_rc_3'] >= 1000:
        P[1] = 1
    else:
        P[1] = -1
    if state['vertical_speed'] <= 0:
        P[2] = 1
    elif state['vertical_speed'] > params['pilot_speed_up']:
        P[2] = (state['vertical_speed'] - params['pilot_speed_up']) / state['vertical_speed']
    else:
        P[2] = -1
    distances['AP_ZIGZAG_P1'] = -1 * min(P[0], P[1], P[2])
    
    # AP_RCFS_P3 Policy
    P = [0] * 4
    if state['rc_failsafe_error'] == 1:
        P[0] = 1
    else:
        P[0] = -1
    if state['relative_alt'] <= 0.5:
        P[1] = 1
    else:
        P[1] = -1
    P[2] = -1  # Cannot track previous armed state
    if state['armed'] == 1:
        P[3] = 1
    else:
        P[3] = -1
    distances['AP_RCFS_P3'] = -1 * min(P[0], P[1], P[2], P[3])
    
    # AP_GCSFS_P1 Policy
    P = [0] * 5
    if state['current_flight_mode'] == "ACRO":
        P[0] = 1
    else:
        P[0] = -1
    if state['rc_failsafe_error'] == 1:
        P[1] = 1
    else:
        P[1] = -1
    P[2] = -1  # Cannot track previous disarmed state
    if state['relative_alt'] <= 0.5:
        P[3] = 1
    else:
        P[3] = -1
    if state['current_rc_3'] != 1000:
        P[4] = 1
    else:
        P[4] = -1
    distances['AP_GCSFS_P1'] = -1 * min(P[0], P[1], P[2], max(P[3], P[4]))
    
    # AP_GCSFS_P2 Policy
    P = [0] * 4
    P[0] = -1  # Cannot monitor GCS failsafe status directly
    if state['relative_alt'] <= 0.5:
        P[1] = 1
    else:
        P[1] = -1
    P[2] = -1  # Cannot track previous armed state
    if state['armed'] == 1:
        P[3] = 1
    else:
        P[3] = -1
    distances['AP_GCSFS_P2'] = -1 * min(P[0], P[1], P[2], P[3])
    
    # AP_BATTFS_P1 Policy
    P = [0] * 5
    P[0] = -1  # Cannot monitor battery failsafe status directly
    if state['current_flight_mode'] == "STABILIZE" or state['current_flight_mode'] == "ACRO":
        P[1] = 1
    else:
        P[1] = -1
    P[2] = -1  # Cannot track previous armed state
    landed_check = (state['relative_alt'] <= 0.5)
    if state['current_rc_3'] == 1000 or landed_check:
        P[3] = 1
    else:
        P[3] = -1
    if state['armed'] == 1:
        P[4] = 1
    else:
        P[4] = -1
    distances['AP_BATTFS_P1'] = -1 * min(P[0], P[1], P[2], P[3], P[4])
    
    return distances


def get_mode_name(mode_num):
    """Convert mode number to mode name for ArduPilot"""
    mode_map = {
        0: "STABILIZE",
        1: "ACRO",
        2: "ALT_HOLD",
        3: "AUTO",
        4: "GUIDED",
        5: "LOITER",
        6: "RTL",
        7: "CIRCLE",
        9: "LAND",
        11: "DRIFT",
        13: "SPORT",
        14: "FLIP",
        15: "AUTOTUNE",
        16: "POSHOLD",
        17: "BRAKE",
        18: "THROW",
        19: "AVOID_ADSB",
        20: "GUIDED_NOGPS",
        21: "SMART_RTL",
        22: "FLOWHOLD",
        23: "FOLLOW",
        24: "ZIGZAG",
    }
    return mode_map.get(mode_num, "UNKNOWN")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python ardupilot_oracle.py <log_file_path>")
        sys.exit(1)
    
    log_file_path = sys.argv[1]
    distances = calculate_distance(log_file_path)
    
    print("ArduPilot Policy Distances:")
    print("=" * 80)
    for policy, distance in distances.items():
        status = "Violated" if distance < 0 else "Satisfied"
        print(f"{policy:20s}: {distance:8.4f}  [{status}]")
