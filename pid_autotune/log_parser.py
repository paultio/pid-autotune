"""
ArduPilot dataflash log parser
Extracts relevant data for PID tuning and filtering analysis
"""

import numpy as np
import pandas as pd
from pymavlink import mavutil
from typing import Dict, List, Optional, Tuple

class LogParser:
    def __init__(self):
        self.required_messages = [
            'ATT',    # Attitude
            'RATE',   # Rate controller
            'RCOU',   # RC output
            'RCIN',   # RC input
            'IMU',    # IMU data
            'GYRO',   # Gyroscope data
            'ACC',    # Accelerometer data
            'PARM',   # Parameters
            'MODE',   # Flight mode
            'BARO',   # Barometer
            'GPS',    # GPS data
        ]
    
    def parse_log(self, log_file_path: str) -> Dict:
        """
        Parse ArduPilot dataflash log file
        
        Args:
            log_file_path (str): Path to the log file
            
        Returns:
            Dict: Parsed flight data organized by message type
        """
        try:
            mavlog = mavutil.mavlink_connection(log_file_path)
        except Exception as e:
            raise ValueError(f"Failed to open log file: {e}")
        
        flight_data = {
            'parameters': {},
            'attitude': [],
            'rates': [],
            'rc_output': [],
            'rc_input': [],
            'imu': [],
            'gyro': [],
            'accelerometer': [],
            'barometer': [],
            'gps': [],
            'mode_changes': [],
            'timestamp_range': None
        }
        
        first_timestamp = None
        last_timestamp = None
        
        print("Parsing log messages...")
        
        while True:
            msg = mavlog.recv_match(type=self.required_messages)
            if msg is None:
                break
            
            timestamp = getattr(msg, 'TimeUS', None) or getattr(msg, 'TimeMS', None)
            if timestamp:
                if first_timestamp is None:
                    first_timestamp = timestamp
                last_timestamp = timestamp
            
            msg_type = msg.get_type()
            
            if msg_type == 'PARM':
                flight_data['parameters'][msg.Name] = msg.Value
            
            elif msg_type == 'ATT':
                flight_data['attitude'].append({
                    'timestamp': timestamp,
                    'roll': msg.Roll,
                    'pitch': msg.Pitch,
                    'yaw': msg.Yaw,
                    'roll_rate': getattr(msg, 'RollRate', 0),
                    'pitch_rate': getattr(msg, 'PitchRate', 0),
                    'yaw_rate': getattr(msg, 'YawRate', 0)
                })
            
            elif msg_type == 'RATE':
                flight_data['rates'].append({
                    'timestamp': timestamp,
                    'roll_target': msg.RDes,
                    'pitch_target': msg.PDes,
                    'yaw_target': msg.YDes,
                    'roll_actual': msg.R,
                    'pitch_actual': msg.P,
                    'yaw_actual': msg.Y,
                    'roll_output': getattr(msg, 'ROut', 0),
                    'pitch_output': getattr(msg, 'POut', 0),
                    'yaw_output': getattr(msg, 'YOut', 0)
                })
            
            elif msg_type == 'RCOU':
                flight_data['rc_output'].append({
                    'timestamp': timestamp,
                    'ch1': msg.C1,
                    'ch2': msg.C2,
                    'ch3': msg.C3,
                    'ch4': msg.C4,
                    'ch5': getattr(msg, 'C5', 0),
                    'ch6': getattr(msg, 'C6', 0),
                    'ch7': getattr(msg, 'C7', 0),
                    'ch8': getattr(msg, 'C8', 0)
                })
            
            elif msg_type == 'RCIN':
                flight_data['rc_input'].append({
                    'timestamp': timestamp,
                    'ch1': msg.C1,
                    'ch2': msg.C2,
                    'ch3': msg.C3,
                    'ch4': msg.C4,
                    'ch5': getattr(msg, 'C5', 0),
                    'ch6': getattr(msg, 'C6', 0),
                    'ch7': getattr(msg, 'C7', 0),
                    'ch8': getattr(msg, 'C8', 0)
                })
            
            elif msg_type == 'IMU':
                flight_data['imu'].append({
                    'timestamp': timestamp,
                    'gyro_x': msg.GyrX,
                    'gyro_y': msg.GyrY,
                    'gyro_z': msg.GyrZ,
                    'accel_x': msg.AccX,
                    'accel_y': msg.AccY,
                    'accel_z': msg.AccZ
                })
            
            elif msg_type == 'GYRO':
                flight_data['gyro'].append({
                    'timestamp': timestamp,
                    'x': msg.GyrX,
                    'y': msg.GyrY,
                    'z': msg.GyrZ
                })
            
            elif msg_type == 'ACC':
                flight_data['accelerometer'].append({
                    'timestamp': timestamp,
                    'x': msg.AccX,
                    'y': msg.AccY,
                    'z': msg.AccZ
                })
            
            elif msg_type == 'MODE':
                flight_data['mode_changes'].append({
                    'timestamp': timestamp,
                    'mode': msg.Mode,
                    'mode_name': getattr(msg, 'ModeNum', 'Unknown')
                })
        
        flight_data['timestamp_range'] = (first_timestamp, last_timestamp)
        
        # Convert lists to pandas DataFrames for easier analysis
        for key in ['attitude', 'rates', 'rc_output', 'rc_input', 'imu', 'gyro', 'accelerometer']:
            if flight_data[key]:
                flight_data[key] = pd.DataFrame(flight_data[key])
                if 'timestamp' in flight_data[key].columns:
                    flight_data[key] = flight_data[key].sort_values('timestamp')
            else:
                # Create empty DataFrame with expected structure
                if key == 'rates':
                    flight_data[key] = pd.DataFrame(columns=['timestamp', 'roll_target', 'pitch_target', 'yaw_target', 
                                                           'roll_actual', 'pitch_actual', 'yaw_actual', 
                                                           'roll_output', 'pitch_output', 'yaw_output'])
                elif key == 'attitude':
                    flight_data[key] = pd.DataFrame(columns=['timestamp', 'roll', 'pitch', 'yaw', 
                                                           'roll_rate', 'pitch_rate', 'yaw_rate'])
                else:
                    flight_data[key] = pd.DataFrame()
        
        print(f"Parsed {len(flight_data['attitude'])} attitude messages")
        print(f"Parsed {len(flight_data['rates'])} rate messages")
        print(f"Found {len(flight_data['parameters'])} parameters")
        
        # Detect flight phases and filter data
        flight_data = self._detect_and_filter_flight_phases(flight_data)
        
        return flight_data
    
    def extract_pid_parameters(self, parameters: Dict) -> Dict:
        """Extract PID parameters from parameter dictionary"""
        pid_params = {}
        
        # Rate controller PIDs
        axes = ['ROLL', 'PITCH', 'YAW']
        for axis in axes:
            pid_params[f'{axis}_RATE'] = {
                'P': parameters.get(f'ATC_RAT_{axis[0]}_P', 0),
                'I': parameters.get(f'ATC_RAT_{axis[0]}_I', 0),
                'D': parameters.get(f'ATC_RAT_{axis[0]}_D', 0),
                'IMAX': parameters.get(f'ATC_RAT_{axis[0]}_IMAX', 0),
                'FILT': parameters.get(f'ATC_RAT_{axis[0]}_FILT', 0)
            }
        
        # Attitude controller PIDs
        for axis in axes:
            pid_params[f'{axis}_ANGLE'] = {
                'P': parameters.get(f'ATC_ANG_{axis[0]}_P', 0)
            }
        
        return pid_params
    
    def extract_filter_parameters(self, parameters: Dict) -> Dict:
        """Extract filter parameters from parameter dictionary"""
        filter_params = {
            'gyro_filters': {
                'GYRO_FILTER': parameters.get('INS_GYRO_FILTER', 0),
                'GYRO_NOTCH_ENABLE': parameters.get('INS_NOTCH_ENABLE', 0),
                'GYRO_NOTCH_FREQ': parameters.get('INS_NOTCH_FREQ', 0),
                'GYRO_NOTCH_BW': parameters.get('INS_NOTCH_BW', 0)
            },
            'accel_filters': {
                'ACCEL_FILTER': parameters.get('INS_ACCEL_FILTER', 0)
            },
            'rate_filters': {
                'ROLL_FILT': parameters.get('ATC_RAT_RLL_FILT', 0),
                'PITCH_FILT': parameters.get('ATC_RAT_PIT_FILT', 0),
                'YAW_FILT': parameters.get('ATC_RAT_YAW_FILT', 0)
            }
        }
        
        return filter_params
    
    def _detect_and_filter_flight_phases(self, flight_data: Dict) -> Dict:
        """
        Detect flight phases and filter data to only include actual flight periods
        Excludes ground operations, pre-flight, and post-flight data
        """
        if flight_data['rates'].empty or flight_data['attitude'].empty:
            print("⚠️  No sufficient data for flight phase detection")
            return flight_data
        
        print("🔍 Detecting flight phases...")
        
        # Get key data for flight detection
        rates_df = flight_data['rates']
        attitude_df = flight_data['attitude']
        
        # Detect flight phases using multiple indicators
        flight_periods = self._identify_flight_periods(rates_df, attitude_df, flight_data)
        
        if not flight_periods:
            print("⚠️  No clear flight periods detected - using all data")
            return flight_data
        
        # Filter all data to only include flight periods
        filtered_data = self._filter_data_to_flight_periods(flight_data, flight_periods)
        
        # Calculate statistics
        total_duration = (flight_data['timestamp_range'][1] - flight_data['timestamp_range'][0]) / 1e6
        flight_duration = sum([(end - start) / 1e6 for start, end in flight_periods])
        
        print(f"✈️  Detected {len(flight_periods)} flight period(s)")
        print(f"📊 Flight time: {flight_duration:.1f}s / {total_duration:.1f}s total ({flight_duration/total_duration*100:.1f}%)")
        
        return filtered_data
    
    def _identify_flight_periods(self, rates_df: pd.DataFrame, attitude_df: pd.DataFrame, 
                               flight_data: Dict) -> List[Tuple[int, int]]:
        """Identify periods when the drone is actually flying"""
        
        flight_indicators = []
        
        # 1. Rate activity indicator - significant rate commands suggest flight
        if not rates_df.empty:
            rate_activity = self._detect_rate_activity(rates_df)
            flight_indicators.append(rate_activity)
        
        # 2. Attitude movement indicator - significant attitude changes suggest flight
        if not attitude_df.empty:
            attitude_movement = self._detect_attitude_movement(attitude_df)
            flight_indicators.append(attitude_movement)
        
        # 3. Flight mode indicator - certain modes indicate flight
        if flight_data['mode_changes']:
            mode_activity = self._detect_flight_modes(flight_data['mode_changes'])
            flight_indicators.append(mode_activity)
        
        # 4. Altitude/barometer activity (if available)
        if flight_data['barometer']:
            altitude_activity = self._detect_altitude_changes(flight_data['barometer'])
            flight_indicators.append(altitude_activity)
        
        # Combine all indicators to find flight periods
        flight_periods = self._combine_flight_indicators(flight_indicators, rates_df)
        
        return flight_periods
    
    def _detect_rate_activity(self, rates_df: pd.DataFrame) -> List[bool]:
        """Detect periods of significant rate controller activity"""
        activity = []
        
        # Calculate rate command magnitudes
        if all(col in rates_df.columns for col in ['roll_target', 'pitch_target', 'yaw_target']):
            roll_cmd = rates_df['roll_target'].abs()
            pitch_cmd = rates_df['pitch_target'].abs()
            yaw_cmd = rates_df['yaw_target'].abs()
            
            # Define activity thresholds (rad/s)
            roll_threshold = 0.1
            pitch_threshold = 0.1
            yaw_threshold = 0.1
            
            # A sample is "active" if any axis has significant rate commands
            for i in range(len(rates_df)):
                is_active = (roll_cmd.iloc[i] > roll_threshold or 
                           pitch_cmd.iloc[i] > pitch_threshold or 
                           yaw_cmd.iloc[i] > yaw_threshold)
                activity.append(is_active)
        
        return activity
    
    def _detect_attitude_movement(self, attitude_df: pd.DataFrame) -> List[bool]:
        """Detect periods of significant attitude changes"""
        activity = []
        
        if all(col in attitude_df.columns for col in ['roll', 'pitch', 'yaw']):
            # Calculate attitude rate of change
            window_size = 10  # Look at changes over 10 samples
            
            for i in range(len(attitude_df)):
                start_idx = max(0, i - window_size)
                end_idx = min(len(attitude_df), i + window_size)
                
                if end_idx - start_idx > 5:  # Ensure minimum window
                    window_data = attitude_df.iloc[start_idx:end_idx]
                    
                    roll_range = window_data['roll'].max() - window_data['roll'].min()
                    pitch_range = window_data['pitch'].max() - window_data['pitch'].min()
                    yaw_range = window_data['yaw'].max() - window_data['yaw'].min()
                    
                    # Thresholds for significant attitude changes (radians)
                    roll_threshold = 0.05  # ~3 degrees
                    pitch_threshold = 0.05  # ~3 degrees
                    yaw_threshold = 0.1    # ~6 degrees
                    
                    is_active = (roll_range > roll_threshold or 
                               pitch_range > pitch_threshold or 
                               yaw_range > yaw_threshold)
                    activity.append(is_active)
                else:
                    activity.append(False)
        
        return activity
    
    def _detect_flight_modes(self, mode_changes: List[Dict]) -> List[bool]:
        """Detect flight periods based on flight modes"""
        # Flight modes that indicate the drone is flying or ready to fly
        flight_modes = {
            'STABILIZE': True,
            'ALT_HOLD': True,
            'LOITER': True,
            'AUTO': True,
            'RTL': True,
            'CIRCLE': True,
            'POSITION': True,
            'GUIDED': True,
            'SPORT': True,
            'FLIP': True,
            'AUTOTUNE': True,
            'POSHOLD': True,
            'LAND': True,
            'BRAKE': True,
            'THROW': True,
            'AVOID_ADSB': True,
            'GUIDED_NOGPS': True,
            'SMART_RTL': True,
            'FLOWHOLD': True,
            'FOLLOW': True,
            'ZIGZAG': True,
            'DISARM': False,
            'MANUAL': False,
            'ACRO': True,
        }
        
        # This is a simplified approach - in practice, you'd need to map this
        # to the actual data timestamps
        return []  # Skip mode-based detection for now
    
    def _detect_altitude_changes(self, barometer_data: List[Dict]) -> List[bool]:
        """Detect periods of altitude changes indicating flight"""
        if not barometer_data:
            return []
        
        activity = []
        altitude_threshold = 0.5  # meters
        
        for i, baro_sample in enumerate(barometer_data):
            if i < 10 or i >= len(barometer_data) - 10:
                activity.append(False)
                continue
            
            # Look at altitude changes over a window
            window_start = max(0, i - 10)
            window_end = min(len(barometer_data), i + 10)
            
            altitudes = [barometer_data[j].get('Alt', 0) for j in range(window_start, window_end)]
            altitude_range = max(altitudes) - min(altitudes)
            
            activity.append(altitude_range > altitude_threshold)
        
        return activity
    
    def _combine_flight_indicators(self, indicators: List[List[bool]], rates_df: pd.DataFrame) -> List[Tuple[int, int]]:
        """Combine multiple flight indicators to identify flight periods"""
        if not indicators:
            return []
        
        # Ensure all indicators have the same length as rates data
        target_length = len(rates_df)
        normalized_indicators = []
        
        for indicator in indicators:
            if len(indicator) == target_length:
                normalized_indicators.append(indicator)
            elif len(indicator) > 0:
                # Resample to match target length
                resampled = self._resample_boolean_array(indicator, target_length)
                normalized_indicators.append(resampled)
        
        if not normalized_indicators:
            return []
        
        # Combine indicators using OR logic (any indicator suggests flight)
        combined_activity = [False] * target_length
        for i in range(target_length):
            combined_activity[i] = any(ind[i] for ind in normalized_indicators if i < len(ind))
        
        # Apply smoothing to reduce noise
        smoothed_activity = self._smooth_flight_activity(combined_activity)
        
        # Find continuous flight periods
        flight_periods = self._find_continuous_periods(smoothed_activity, rates_df)
        
        return flight_periods
    
    def _resample_boolean_array(self, array: List[bool], target_length: int) -> List[bool]:
        """Resample boolean array to target length"""
        if len(array) == target_length:
            return array
        
        resampled = []
        ratio = len(array) / target_length
        
        for i in range(target_length):
            source_idx = int(i * ratio)
            source_idx = min(source_idx, len(array) - 1)
            resampled.append(array[source_idx])
        
        return resampled
    
    def _smooth_flight_activity(self, activity: List[bool]) -> List[bool]:
        """Apply smoothing to reduce noise in flight activity detection"""
        if len(activity) < 20:
            return activity
        
        smoothed = activity.copy()
        window_size = 20  # Smooth over ~5 seconds at 4Hz
        
        for i in range(window_size, len(activity) - window_size):
            window = activity[i-window_size:i+window_size]
            # If more than 30% of window is active, consider this point active
            if sum(window) > len(window) * 0.3:
                smoothed[i] = True
        
        return smoothed
    
    def _find_continuous_periods(self, activity: List[bool], rates_df: pd.DataFrame) -> List[Tuple[int, int]]:
        """Find continuous periods of flight activity"""
        if not activity or rates_df.empty:
            return []
        
        periods = []
        in_flight = False
        flight_start = None
        min_flight_duration = 30  # Minimum 30 seconds to be considered a flight
        
        for i, is_active in enumerate(activity):
            if is_active and not in_flight:
                # Start of flight period
                in_flight = True
                flight_start = i
            elif not is_active and in_flight:
                # End of flight period
                in_flight = False
                if flight_start is not None:
                    duration_samples = i - flight_start
                    # Check if duration is long enough
                    if duration_samples > min_flight_duration:
                        start_timestamp = rates_df.iloc[flight_start]['timestamp']
                        end_timestamp = rates_df.iloc[i-1]['timestamp']
                        periods.append((start_timestamp, end_timestamp))
                flight_start = None
        
        # Handle case where flight continues to end of log
        if in_flight and flight_start is not None:
            duration_samples = len(activity) - flight_start
            if duration_samples > min_flight_duration:
                start_timestamp = rates_df.iloc[flight_start]['timestamp']
                end_timestamp = rates_df.iloc[-1]['timestamp']
                periods.append((start_timestamp, end_timestamp))
        
        return periods
    
    def _filter_data_to_flight_periods(self, flight_data: Dict, flight_periods: List[Tuple[int, int]]) -> Dict:
        """Filter all flight data to only include flight periods"""
        filtered_data = flight_data.copy()
        
        # Filter DataFrame-based data
        dataframe_keys = ['attitude', 'rates', 'rc_output', 'rc_input', 'imu', 'gyro', 'accelerometer']
        
        for key in dataframe_keys:
            if key in filtered_data and not filtered_data[key].empty:
                df = filtered_data[key]
                if 'timestamp' in df.columns:
                    # Filter to flight periods
                    flight_mask = pd.Series(False, index=df.index)
                    
                    for start_time, end_time in flight_periods:
                        period_mask = (df['timestamp'] >= start_time) & (df['timestamp'] <= end_time)
                        flight_mask = flight_mask | period_mask
                    
                    filtered_data[key] = df[flight_mask].reset_index(drop=True)
        
        # Filter list-based data
        list_keys = ['barometer', 'gps', 'mode_changes']
        for key in list_keys:
            if key in filtered_data and filtered_data[key]:
                filtered_list = []
                for item in filtered_data[key]:
                    item_timestamp = item.get('timestamp')
                    if item_timestamp:
                        for start_time, end_time in flight_periods:
                            if start_time <= item_timestamp <= end_time:
                                filtered_list.append(item)
                                break
                filtered_data[key] = filtered_list
        
        return filtered_data