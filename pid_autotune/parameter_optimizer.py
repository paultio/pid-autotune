"""
Parameter optimization engine for ArduPilot
Uses control theory and optimization algorithms to suggest optimal PID and filter parameters
"""

import numpy as np
import pandas as pd
from scipy.optimize import minimize, differential_evolution
from typing import Dict, List, Tuple, Optional

class ParameterOptimizer:
    def __init__(self):
        self.pid_bounds = {
            'rate_p': (0.01, 0.5),
            'rate_i': (0.001, 0.3),
            'rate_d': (0.0001, 0.01),
            'rate_imax': (0.1, 0.5),
            'rate_filt': (5.0, 100.0),
            'angle_p': (1.0, 15.0)
        }
        
        self.filter_bounds = {
            'gyro_filter': (10.0, 200.0),
            'accel_filter': (10.0, 100.0),
            'notch_freq': (20.0, 400.0),
            'notch_bandwidth': (5.0, 50.0)
        }
    
    def optimize_pid(self, flight_data: Dict, pid_analysis: Dict) -> Dict:
        """
        Optimize PID parameters based on flight data analysis
        
        Args:
            flight_data (Dict): Parsed flight data
            pid_analysis (Dict): PID analysis results
            
        Returns:
            Dict: Optimized PID parameters
        """
        optimized_params = {}
        
        # Extract current parameters
        current_params = flight_data.get('parameters', {})
        
        # Optimize each axis
        for axis in ['roll', 'pitch', 'yaw']:
            if axis in pid_analysis and pid_analysis[axis]['sample_count'] > 0:
                axis_optimized = self._optimize_axis_pid(
                    flight_data, pid_analysis[axis], axis, current_params
                )
                optimized_params.update(axis_optimized)
        
        return optimized_params
    
    def _optimize_axis_pid(self, flight_data: Dict, axis_analysis: Dict, 
                          axis: str, current_params: Dict) -> Dict:
        """Optimize PID parameters for a single axis"""
        
        # Get current parameter values
        axis_prefix = {'roll': 'RLL', 'pitch': 'PIT', 'yaw': 'YAW'}[axis]
        
        current_rate_p = current_params.get(f'ATC_RAT_{axis_prefix}_P', 0.1)
        current_rate_i = current_params.get(f'ATC_RAT_{axis_prefix}_I', 0.05)
        current_rate_d = current_params.get(f'ATC_RAT_{axis_prefix}_D', 0.005)
        current_rate_imax = current_params.get(f'ATC_RAT_{axis_prefix}_IMAX', 0.3)
        current_rate_filt = current_params.get(f'ATC_RAT_{axis_prefix}_FILT', 20.0)
        current_angle_p = current_params.get(f'ATC_ANG_{axis_prefix}_P', 6.0)
        
        # Analyze current performance
        performance_score = axis_analysis.get('performance_score', 0)
        tracking_error = axis_analysis.get('tracking_error_rms', 0)
        oscillation_metrics = axis_analysis.get('oscillation_metrics', {})
        stability_metrics = axis_analysis.get('stability_metrics', {})
        
        # Determine optimization strategy based on current performance
        optimized_params = {}
        
        if performance_score < 50:  # Poor performance, need significant changes
            optimized_params.update(self._aggressive_pid_optimization(
                axis_prefix, current_rate_p, current_rate_i, current_rate_d,
                current_rate_imax, current_rate_filt, current_angle_p,
                tracking_error, oscillation_metrics, stability_metrics
            ))
        elif performance_score < 75:  # Moderate performance, fine-tuning needed
            optimized_params.update(self._moderate_pid_optimization(
                axis_prefix, current_rate_p, current_rate_i, current_rate_d,
                current_rate_imax, current_rate_filt, current_angle_p,
                tracking_error, oscillation_metrics, stability_metrics
            ))
        else:  # Good performance, minor adjustments
            optimized_params.update(self._fine_tune_pid(
                axis_prefix, current_rate_p, current_rate_i, current_rate_d,
                current_rate_imax, current_rate_filt, current_angle_p,
                tracking_error, oscillation_metrics, stability_metrics
            ))
        
        return optimized_params
    
    def _aggressive_pid_optimization(self, axis_prefix: str, p: float, i: float, d: float,
                                   imax: float, filt: float, angle_p: float,
                                   tracking_error: float, oscillation_metrics: Dict,
                                   stability_metrics: Dict) -> Dict:
        """Aggressive PID optimization for poor performance"""
        
        # Check for specific problems and apply targeted fixes
        optimized = {}
        
        # High tracking error - increase proportional gain
        if tracking_error > 0.1:
            new_p = min(p * 1.5, self.pid_bounds['rate_p'][1])
            optimized[f'ATC_RAT_{axis_prefix}_P'] = new_p
        
        # Oscillations - reduce gains and increase filtering
        if oscillation_metrics.get('amplitude', 0) > 0.05:
            damping_ratio = oscillation_metrics.get('damping_ratio', 1.0)
            
            if damping_ratio < 0.3:  # Underdamped
                new_p = max(p * 0.7, self.pid_bounds['rate_p'][0])
                new_d = max(d * 0.5, self.pid_bounds['rate_d'][0])
                new_filt = max(filt * 0.8, self.pid_bounds['rate_filt'][0])
                
                optimized[f'ATC_RAT_{axis_prefix}_P'] = new_p
                optimized[f'ATC_RAT_{axis_prefix}_D'] = new_d
                optimized[f'ATC_RAT_{axis_prefix}_FILT'] = new_filt
        
        # Poor stability - reduce integral gain
        if stability_metrics.get('stability_score', 0) < 0.3:
            new_i = max(i * 0.5, self.pid_bounds['rate_i'][0])
            optimized[f'ATC_RAT_{axis_prefix}_I'] = new_i
        
        # Slow response - increase angle P
        step_responses = []  # Would need to extract from analysis
        avg_rise_time = 0.2  # Default assumption
        
        if avg_rise_time > 0.3:
            new_angle_p = min(angle_p * 1.2, self.pid_bounds['angle_p'][1])
            optimized[f'ATC_ANG_{axis_prefix}_P'] = new_angle_p
        
        return optimized
    
    def _moderate_pid_optimization(self, axis_prefix: str, p: float, i: float, d: float,
                                 imax: float, filt: float, angle_p: float,
                                 tracking_error: float, oscillation_metrics: Dict,
                                 stability_metrics: Dict) -> Dict:
        """Moderate PID optimization for average performance"""
        
        optimized = {}
        
        # Fine-tune based on specific metrics
        if tracking_error > 0.05:
            # Slightly increase P gain
            new_p = min(p * 1.1, self.pid_bounds['rate_p'][1])
            optimized[f'ATC_RAT_{axis_prefix}_P'] = new_p
        
        # Adjust D gain based on oscillation frequency
        osc_freq = oscillation_metrics.get('frequency', 0)
        if osc_freq > 0:
            # Optimize D gain for the oscillation frequency
            optimal_d = self._calculate_optimal_d_gain(p, osc_freq)
            optimized[f'ATC_RAT_{axis_prefix}_D'] = optimal_d
        
        # Adjust I gain based on steady-state error
        steady_state_error = tracking_error * 0.1  # Estimate
        if steady_state_error > 0.01:
            new_i = min(i * 1.2, self.pid_bounds['rate_i'][1])
            optimized[f'ATC_RAT_{axis_prefix}_I'] = new_i
        
        return optimized
    
    def _fine_tune_pid(self, axis_prefix: str, p: float, i: float, d: float,
                      imax: float, filt: float, angle_p: float,
                      tracking_error: float, oscillation_metrics: Dict,
                      stability_metrics: Dict) -> Dict:
        """Fine-tune PID for already good performance"""
        
        optimized = {}
        
        # Very small adjustments
        if tracking_error > 0.02:
            new_p = min(p * 1.05, self.pid_bounds['rate_p'][1])
            optimized[f'ATC_RAT_{axis_prefix}_P'] = new_p
        
        # Optimize filter frequency
        if oscillation_metrics.get('frequency', 0) > 0:
            osc_freq = oscillation_metrics['frequency']
            # Set filter slightly below oscillation frequency
            new_filt = max(osc_freq * 0.8, self.pid_bounds['rate_filt'][0])
            optimized[f'ATC_RAT_{axis_prefix}_FILT'] = new_filt
        
        return optimized
    
    def _calculate_optimal_d_gain(self, p_gain: float, oscillation_freq: float) -> float:
        """Calculate optimal D gain based on P gain and oscillation frequency"""
        # Rule of thumb: D = P / (2 * pi * f)
        optimal_d = p_gain / (2 * np.pi * oscillation_freq)
        return np.clip(optimal_d, self.pid_bounds['rate_d'][0], self.pid_bounds['rate_d'][1])
    
    def optimize_filters(self, flight_data: Dict, filter_analysis: Dict) -> Dict:
        """
        Optimize filter parameters based on filter analysis
        
        Args:
            flight_data (Dict): Parsed flight data
            filter_analysis (Dict): Filter analysis results
            
        Returns:
            Dict: Optimized filter parameters
        """
        optimized_params = {}
        
        # Extract current parameters
        current_params = flight_data.get('parameters', {})
        
        # Optimize gyro filters
        gyro_filters = self._optimize_gyro_filters(filter_analysis, current_params)
        optimized_params.update(gyro_filters)
        
        # Optimize accelerometer filters
        accel_filters = self._optimize_accel_filters(filter_analysis, current_params)
        optimized_params.update(accel_filters)
        
        # Optimize rate controller filters
        rate_filters = self._optimize_rate_controller_filters(filter_analysis, current_params)
        optimized_params.update(rate_filters)
        
        return optimized_params
    
    def _optimize_gyro_filters(self, filter_analysis: Dict, current_params: Dict) -> Dict:
        """Optimize gyroscope filters"""
        optimized = {}
        
        current_gyro_filter = current_params.get('INS_GYRO_FILTER', 20.0)
        current_notch_freq = current_params.get('INS_NOTCH_FREQ', 0.0)
        current_notch_enable = current_params.get('INS_NOTCH_ENABLE', 0)
        
        # Analyze vibration data
        vibration_analysis = filter_analysis.get('vibration_analysis', {})
        overall_vibration = vibration_analysis.get('overall_assessment', {})
        
        if overall_vibration.get('vibration_detected', False):
            # Enable notch filter
            primary_freq = overall_vibration.get('primary_frequency', 0)
            if primary_freq > 0:
                optimized['INS_NOTCH_ENABLE'] = 1
                optimized['INS_NOTCH_FREQ'] = primary_freq
                optimized['INS_NOTCH_BW'] = max(primary_freq * 0.2, 5.0)
        
        # Optimize low-pass filter based on noise
        noise_analysis = filter_analysis.get('noise_analysis', {})
        noise_level = noise_analysis.get('overall_noise_level', 'moderate')
        
        if noise_level == 'high':
            # More aggressive low-pass filtering
            new_gyro_filter = max(current_gyro_filter * 0.7, 10.0)
        elif noise_level == 'low':
            # Less aggressive filtering for better response
            new_gyro_filter = min(current_gyro_filter * 1.3, 100.0)
        else:
            # Moderate adjustment
            new_gyro_filter = current_gyro_filter
        
        optimized['INS_GYRO_FILTER'] = new_gyro_filter
        
        return optimized
    
    def _optimize_accel_filters(self, filter_analysis: Dict, current_params: Dict) -> Dict:
        """Optimize accelerometer filters"""
        optimized = {}
        
        current_accel_filter = current_params.get('INS_ACCEL_FILTER', 10.0)
        
        # Analyze accelerometer data
        accel_analysis = filter_analysis.get('accel_analysis', {})
        
        # Check for clipping
        clipping_detected = False
        for axis_data in accel_analysis.values():
            if isinstance(axis_data, dict):
                clipping_analysis = axis_data.get('clipping_analysis', {})
                if clipping_analysis.get('clipping_detected', False):
                    clipping_detected = True
                    break
        
        if clipping_detected:
            # Increase filtering to reduce clipping
            new_accel_filter = max(current_accel_filter * 0.8, 5.0)
        else:
            # Check vibration levels
            vibration_detected = False
            for axis_data in accel_analysis.values():
                if isinstance(axis_data, dict):
                    vibration_analysis = axis_data.get('vibration_analysis', {})
                    if vibration_analysis.get('vibration_detected', False):
                        vibration_detected = True
                        break
            
            if vibration_detected:
                new_accel_filter = max(current_accel_filter * 0.9, 8.0)
            else:
                new_accel_filter = min(current_accel_filter * 1.1, 50.0)
        
        optimized['INS_ACCEL_FILTER'] = new_accel_filter
        
        return optimized
    
    def _optimize_rate_controller_filters(self, filter_analysis: Dict, current_params: Dict) -> Dict:
        """Optimize rate controller filters"""
        optimized = {}
        
        rate_analysis = filter_analysis.get('rate_controller_analysis', {})
        
        axes = ['roll', 'pitch', 'yaw']
        axis_mapping = {'roll': 'RLL', 'pitch': 'PIT', 'yaw': 'YAW'}
        
        for axis in axes:
            if axis in rate_analysis:
                axis_data = rate_analysis[axis]
                axis_prefix = axis_mapping[axis]
                
                current_filter = current_params.get(f'ATC_RAT_{axis_prefix}_FILT', 20.0)
                
                # Analyze control quality
                control_quality = axis_data.get('control_quality', {})
                tracking_quality = control_quality.get('tracking_quality', 0.5)
                control_smoothness = control_quality.get('control_smoothness', 0.5)
                
                # Analyze frequency content
                freq_analysis = axis_data.get('frequency_analysis', {})
                dominant_freq = freq_analysis.get('dominant_frequency', 0)
                
                if tracking_quality < 0.7 and dominant_freq > 0:
                    # Poor tracking, might need less filtering
                    new_filter = min(current_filter * 1.2, 100.0)
                elif control_smoothness < 0.7:
                    # Rough control, need more filtering
                    new_filter = max(current_filter * 0.8, 5.0)
                else:
                    # Good performance, minor adjustment
                    new_filter = current_filter
                
                optimized[f'ATC_RAT_{axis_prefix}_FILT'] = new_filter
        
        return optimized
    
    def _objective_function(self, params: np.ndarray, flight_data: Dict, 
                           analysis_data: Dict, param_names: List[str]) -> float:
        """
        Objective function for optimization
        
        Args:
            params: Parameter values to evaluate
            flight_data: Flight data
            analysis_data: Analysis results
            param_names: Parameter names
            
        Returns:
            float: Cost value (lower is better)
        """
        # Create parameter dictionary
        param_dict = dict(zip(param_names, params))
        
        # Calculate cost based on multiple criteria
        cost = 0.0
        
        # Tracking error cost
        tracking_error = analysis_data.get('tracking_error_rms', 0)
        cost += tracking_error * 100  # Weight tracking error heavily
        
        # Oscillation cost
        oscillation_metrics = analysis_data.get('oscillation_metrics', {})
        oscillation_amplitude = oscillation_metrics.get('amplitude', 0)
        cost += oscillation_amplitude * 50
        
        # Stability cost
        stability_metrics = analysis_data.get('stability_metrics', {})
        stability_score = stability_metrics.get('stability_score', 1.0)
        cost += (1.0 - stability_score) * 30
        
        # Penalize extreme parameter values
        for param_name, value in param_dict.items():
            if param_name in self.pid_bounds:
                bounds = self.pid_bounds[param_name]
                if value < bounds[0] or value > bounds[1]:
                    cost += 1000  # Large penalty for out-of-bounds
        
        return cost
    
    def _advanced_pid_optimization(self, flight_data: Dict, axis_analysis: Dict,
                                 axis: str, current_params: Dict) -> Dict:
        """Advanced PID optimization using numerical methods"""
        
        # Extract rate data for this axis
        rates_df = flight_data['rates']
        axis_prefix = {'roll': 'RLL', 'pitch': 'PIT', 'yaw': 'YAW'}[axis]
        
        target_col = f'{axis}_target'
        actual_col = f'{axis}_actual'
        
        if target_col not in rates_df.columns or actual_col not in rates_df.columns:
            return {}
        
        # Current parameter values
        current_p = current_params.get(f'ATC_RAT_{axis_prefix}_P', 0.1)
        current_i = current_params.get(f'ATC_RAT_{axis_prefix}_I', 0.05)
        current_d = current_params.get(f'ATC_RAT_{axis_prefix}_D', 0.005)
        
        # Parameter bounds for optimization
        bounds = [
            self.pid_bounds['rate_p'],
            self.pid_bounds['rate_i'],
            self.pid_bounds['rate_d']
        ]
        
        # Initial guess
        x0 = [current_p, current_i, current_d]
        
        # Parameter names
        param_names = [f'ATC_RAT_{axis_prefix}_P', f'ATC_RAT_{axis_prefix}_I', f'ATC_RAT_{axis_prefix}_D']
        
        try:
            # Use differential evolution for global optimization
            result = differential_evolution(
                self._objective_function,
                bounds,
                args=(flight_data, axis_analysis, param_names),
                seed=42,
                maxiter=100,
                popsize=15
            )
            
            if result.success:
                optimized_params = dict(zip(param_names, result.x))
                return optimized_params
            else:
                # Fall back to current parameters with small adjustments
                return self._fallback_optimization(axis_prefix, current_p, current_i, current_d, axis_analysis)
        
        except Exception as e:
            print(f"Optimization failed for {axis}: {e}")
            return self._fallback_optimization(axis_prefix, current_p, current_i, current_d, axis_analysis)
    
    def _fallback_optimization(self, axis_prefix: str, p: float, i: float, d: float,
                              axis_analysis: Dict) -> Dict:
        """Fallback optimization using heuristics"""
        optimized = {}
        
        performance_score = axis_analysis.get('performance_score', 50)
        
        if performance_score < 30:
            # Significant changes needed
            optimized[f'ATC_RAT_{axis_prefix}_P'] = min(p * 1.3, self.pid_bounds['rate_p'][1])
            optimized[f'ATC_RAT_{axis_prefix}_I'] = max(i * 0.7, self.pid_bounds['rate_i'][0])
            optimized[f'ATC_RAT_{axis_prefix}_D'] = max(d * 0.8, self.pid_bounds['rate_d'][0])
        elif performance_score < 70:
            # Moderate changes
            optimized[f'ATC_RAT_{axis_prefix}_P'] = min(p * 1.1, self.pid_bounds['rate_p'][1])
            optimized[f'ATC_RAT_{axis_prefix}_I'] = min(i * 1.1, self.pid_bounds['rate_i'][1])
            optimized[f'ATC_RAT_{axis_prefix}_D'] = min(d * 1.1, self.pid_bounds['rate_d'][1])
        else:
            # Small adjustments
            optimized[f'ATC_RAT_{axis_prefix}_P'] = min(p * 1.05, self.pid_bounds['rate_p'][1])
            optimized[f'ATC_RAT_{axis_prefix}_I'] = min(i * 1.05, self.pid_bounds['rate_i'][1])
            optimized[f'ATC_RAT_{axis_prefix}_D'] = min(d * 1.05, self.pid_bounds['rate_d'][1])
        
        return optimized