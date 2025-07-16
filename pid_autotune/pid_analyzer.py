"""
PID performance analysis for ArduPilot
Analyzes step responses, stability, and control loop performance
"""

import numpy as np
import pandas as pd
from scipy import signal
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from typing import Dict, List, Tuple, Optional
import os

class PIDAnalyzer:
    def __init__(self):
        self.sample_rate = 400  # Hz, typical ArduPilot loop rate
        
    def analyze(self, flight_data: Dict) -> Dict:
        """
        Analyze PID performance from flight data
        
        Args:
            flight_data (Dict): Parsed flight data
            
        Returns:
            Dict: Analysis results for each axis
        """
        if flight_data['rates'].empty:
            raise ValueError("No rate controller data found in log")
        
        results = {
            'roll': self._analyze_axis(flight_data, 'roll'),
            'pitch': self._analyze_axis(flight_data, 'pitch'),
            'yaw': self._analyze_axis(flight_data, 'yaw'),
            'overall_score': 0,
            'stability_analysis': {},
            'frequency_analysis': {},
            'performance_trends': {},
            'detailed_metrics': {},
            'visualization_data': {}
        }
        
        # Calculate overall performance score
        axis_scores = []
        for axis in ['roll', 'pitch', 'yaw']:
            if results[axis]['tracking_error_rms'] > 0:
                axis_scores.append(results[axis]['performance_score'])
        
        results['overall_score'] = np.mean(axis_scores) if axis_scores else 0
        
        # Stability analysis
        results['stability_analysis'] = self._analyze_stability(flight_data)
        
        # Frequency domain analysis
        results['frequency_analysis'] = self._analyze_frequency_response(flight_data)
        
        # Performance trends analysis
        results['performance_trends'] = self._analyze_performance_trends(flight_data)
        
        # Detailed metrics compilation
        results['detailed_metrics'] = self._compile_detailed_metrics(results, flight_data)
        
        # Prepare visualization data
        results['visualization_data'] = self._prepare_visualization_data(flight_data, results)
        
        return results
    
    def _analyze_axis(self, flight_data: Dict, axis: str) -> Dict:
        """Analyze PID performance for a single axis"""
        rates_df = flight_data['rates']
        
        target_col = f'{axis}_target'
        actual_col = f'{axis}_actual'
        
        if target_col not in rates_df.columns or actual_col not in rates_df.columns:
            return self._empty_axis_result()
        
        target = rates_df[target_col].values
        actual = rates_df[actual_col].values
        timestamps = rates_df['timestamp'].values
        
        # Calculate time vector in seconds
        time_vec = (timestamps - timestamps[0]) / 1e6  # Convert microseconds to seconds
        
        # Basic tracking metrics
        error = target - actual
        tracking_error_rms = np.sqrt(np.mean(error**2))
        tracking_error_max = np.max(np.abs(error))
        
        # Step response analysis
        step_responses = self._find_step_responses(time_vec, target, actual)
        
        # Oscillation analysis
        oscillation_metrics = self._analyze_oscillations(time_vec, error)
        
        # Stability metrics
        stability_metrics = self._calculate_stability_metrics(time_vec, target, actual)
        
        # Performance score (0-100)
        performance_score = self._calculate_performance_score(
            tracking_error_rms, oscillation_metrics, stability_metrics
        )
        
        return {
            'tracking_error_rms': tracking_error_rms,
            'tracking_error_max': tracking_error_max,
            'step_responses': step_responses,
            'oscillation_metrics': oscillation_metrics,
            'stability_metrics': stability_metrics,
            'performance_score': performance_score,
            'sample_count': len(target)
        }
    
    def _find_step_responses(self, time_vec: np.ndarray, target: np.ndarray, 
                           actual: np.ndarray) -> List[Dict]:
        """Find and analyze step responses in the data"""
        step_responses = []
        
        # Find significant target changes (steps)
        target_diff = np.diff(target)
        step_threshold = np.std(target_diff) * 3  # 3-sigma threshold
        
        step_indices = np.where(np.abs(target_diff) > step_threshold)[0]
        
        for step_idx in step_indices:
            if step_idx < 10 or step_idx > len(target) - 50:
                continue  # Skip steps too close to boundaries
            
            # Define analysis window (before and after step)
            start_idx = max(0, step_idx - 10)
            end_idx = min(len(target), step_idx + 50)
            
            step_time = time_vec[start_idx:end_idx]
            step_target = target[start_idx:end_idx]
            step_actual = actual[start_idx:end_idx]
            
            # Analyze this step response
            step_analysis = self._analyze_single_step(step_time, step_target, step_actual)
            step_analysis['step_index'] = step_idx
            step_analysis['step_magnitude'] = target_diff[step_idx]
            
            step_responses.append(step_analysis)
        
        return step_responses
    
    def _analyze_single_step(self, time_vec: np.ndarray, target: np.ndarray, 
                           actual: np.ndarray) -> Dict:
        """Analyze a single step response"""
        # Find steady-state values
        initial_value = np.mean(actual[:5])
        final_target = target[-1]
        final_actual = np.mean(actual[-10:])
        
        step_size = final_target - target[0]
        
        if abs(step_size) < 0.01:  # Ignore very small steps
            return {
                'rise_time': 0,
                'settling_time': 0,
                'overshoot': 0,
                'steady_state_error': 0,
                'valid': False
            }
        
        # Calculate rise time (10% to 90% of final value)
        rise_time = self._calculate_rise_time(time_vec, actual, initial_value, final_target)
        
        # Calculate settling time (within 5% of final value)
        settling_time = self._calculate_settling_time(time_vec, actual, final_target)
        
        # Calculate overshoot
        overshoot = self._calculate_overshoot(actual, initial_value, final_target)
        
        # Steady-state error
        steady_state_error = abs(final_actual - final_target)
        
        return {
            'rise_time': rise_time,
            'settling_time': settling_time,
            'overshoot': overshoot,
            'steady_state_error': steady_state_error,
            'valid': True
        }
    
    def _calculate_rise_time(self, time_vec: np.ndarray, response: np.ndarray, 
                           initial: float, final: float) -> float:
        """Calculate 10%-90% rise time"""
        step_size = final - initial
        if abs(step_size) < 0.01:
            return 0
        
        threshold_10 = initial + 0.1 * step_size
        threshold_90 = initial + 0.9 * step_size
        
        # Find crossing points
        if step_size > 0:
            idx_10 = np.where(response >= threshold_10)[0]
            idx_90 = np.where(response >= threshold_90)[0]
        else:
            idx_10 = np.where(response <= threshold_10)[0]
            idx_90 = np.where(response <= threshold_90)[0]
        
        if len(idx_10) > 0 and len(idx_90) > 0:
            return time_vec[idx_90[0]] - time_vec[idx_10[0]]
        
        return 0
    
    def _calculate_settling_time(self, time_vec: np.ndarray, response: np.ndarray, 
                               final: float, tolerance: float = 0.05) -> float:
        """Calculate settling time within tolerance of final value"""
        error = np.abs(response - final)
        threshold = abs(final) * tolerance
        
        # Find last point outside tolerance
        outside_tolerance = np.where(error > threshold)[0]
        
        if len(outside_tolerance) > 0:
            settling_idx = outside_tolerance[-1] + 1
            if settling_idx < len(time_vec):
                return time_vec[settling_idx] - time_vec[0]
        
        return time_vec[-1] - time_vec[0]
    
    def _calculate_overshoot(self, response: np.ndarray, initial: float, 
                           final: float) -> float:
        """Calculate maximum overshoot percentage"""
        step_size = final - initial
        if abs(step_size) < 0.01:
            return 0
        
        if step_size > 0:
            max_value = np.max(response)
            overshoot = max_value - final
        else:
            min_value = np.min(response)
            overshoot = final - min_value
        
        return (overshoot / abs(step_size)) * 100
    
    def _analyze_oscillations(self, time_vec: np.ndarray, error: np.ndarray) -> Dict:
        """Analyze oscillations in tracking error"""
        if len(error) < 100:
            return {'frequency': 0, 'amplitude': 0, 'damping_ratio': 0}
        
        # Remove DC component
        error_ac = error - np.mean(error)
        
        # Calculate RMS of oscillations
        oscillation_rms = np.sqrt(np.mean(error_ac**2))
        
        # Find dominant frequency using FFT
        dt = np.mean(np.diff(time_vec))
        freqs = np.fft.fftfreq(len(error_ac), dt)
        fft = np.fft.fft(error_ac)
        
        # Find peak frequency (excluding DC)
        positive_freqs = freqs[1:len(freqs)//2]
        positive_fft = np.abs(fft[1:len(fft)//2])
        
        if len(positive_fft) > 0:
            dominant_freq_idx = np.argmax(positive_fft)
            dominant_frequency = positive_freqs[dominant_freq_idx]
        else:
            dominant_frequency = 0
        
        # Estimate damping ratio from envelope decay
        damping_ratio = self._estimate_damping_ratio(error_ac)
        
        return {
            'frequency': dominant_frequency,
            'amplitude': oscillation_rms,
            'damping_ratio': damping_ratio
        }
    
    def _estimate_damping_ratio(self, signal_data: np.ndarray) -> float:
        """Estimate damping ratio from signal envelope"""
        try:
            # Find envelope using Hilbert transform
            analytic_signal = signal.hilbert(signal_data)
            envelope = np.abs(analytic_signal)
            
            # Find peaks in envelope
            peaks, _ = signal.find_peaks(envelope, height=np.max(envelope) * 0.1)
            
            if len(peaks) < 2:
                return 1.0  # Overdamped
            
            # Calculate logarithmic decrement
            peak_values = envelope[peaks]
            if len(peak_values) < 2:
                return 1.0
            
            # Find exponential decay rate
            peak_ratios = peak_values[:-1] / peak_values[1:]
            log_decrement = np.mean(np.log(peak_ratios))
            
            # Convert to damping ratio
            damping_ratio = log_decrement / np.sqrt(4 * np.pi**2 + log_decrement**2)
            
            return np.clip(damping_ratio, 0, 1)
        
        except:
            return 1.0
    
    def _calculate_stability_metrics(self, time_vec: np.ndarray, target: np.ndarray, 
                                   actual: np.ndarray) -> Dict:
        """Calculate stability metrics"""
        error = target - actual
        
        # Stability margin (how close to instability)
        error_variance = np.var(error)
        error_trend = np.polyfit(time_vec, error, 1)[0]  # Linear trend
        
        # Check for growing oscillations
        window_size = min(100, len(error) // 4)
        if window_size > 10:
            early_var = np.var(error[:window_size])
            late_var = np.var(error[-window_size:])
            variance_ratio = late_var / (early_var + 1e-10)
        else:
            variance_ratio = 1.0
        
        # Stability score (0-1, higher is more stable)
        stability_score = 1.0 / (1.0 + error_variance + abs(error_trend) + 
                                max(0, variance_ratio - 1))
        
        return {
            'error_variance': error_variance,
            'error_trend': error_trend,
            'variance_ratio': variance_ratio,
            'stability_score': stability_score
        }
    
    def _calculate_performance_score(self, tracking_error_rms: float, 
                                   oscillation_metrics: Dict, 
                                   stability_metrics: Dict) -> float:
        """Calculate overall performance score (0-100)"""
        # Penalize high tracking error
        tracking_score = max(0, 100 - tracking_error_rms * 1000)
        
        # Penalize oscillations
        oscillation_penalty = oscillation_metrics['amplitude'] * 500
        oscillation_score = max(0, 100 - oscillation_penalty)
        
        # Reward stability
        stability_score = stability_metrics['stability_score'] * 100
        
        # Weighted average
        overall_score = (tracking_score * 0.4 + oscillation_score * 0.3 + 
                        stability_score * 0.3)
        
        return np.clip(overall_score, 0, 100)
    
    def _analyze_stability(self, flight_data: Dict) -> Dict:
        """Analyze overall system stability"""
        if flight_data['rates'].empty:
            return {}
        
        rates_df = flight_data['rates']
        
        # Check for sustained oscillations
        roll_osc = self._detect_sustained_oscillation(rates_df['roll_actual'].values)
        pitch_osc = self._detect_sustained_oscillation(rates_df['pitch_actual'].values)
        yaw_osc = self._detect_sustained_oscillation(rates_df['yaw_actual'].values)
        
        return {
            'roll_oscillation': roll_osc,
            'pitch_oscillation': pitch_osc,
            'yaw_oscillation': yaw_osc,
            'overall_stable': not (roll_osc or pitch_osc or yaw_osc)
        }
    
    def _detect_sustained_oscillation(self, signal_data: np.ndarray) -> bool:
        """Detect if signal has sustained oscillations"""
        if len(signal_data) < 100:
            return False
        
        # Calculate autocorrelation
        autocorr = np.correlate(signal_data, signal_data, mode='full')
        autocorr = autocorr[len(autocorr)//2:]
        autocorr = autocorr / autocorr[0]  # Normalize
        
        # Look for periodic structure
        peaks, _ = signal.find_peaks(autocorr[10:], height=0.3)
        
        return len(peaks) > 3  # Multiple peaks indicate oscillation
    
    def _analyze_frequency_response(self, flight_data: Dict) -> Dict:
        """Analyze frequency response characteristics"""
        if flight_data['rates'].empty:
            return {}
        
        rates_df = flight_data['rates']
        
        # Calculate bandwidth and phase margin for each axis
        results = {}
        
        for axis in ['roll', 'pitch', 'yaw']:
            target_col = f'{axis}_target'
            actual_col = f'{axis}_actual'
            
            if target_col in rates_df.columns and actual_col in rates_df.columns:
                target = rates_df[target_col].values
                actual = rates_df[actual_col].values
                
                # Estimate bandwidth
                bandwidth = self._estimate_bandwidth(target, actual)
                
                # Estimate phase margin
                phase_margin = self._estimate_phase_margin(target, actual)
                
                results[axis] = {
                    'bandwidth': bandwidth,
                    'phase_margin': phase_margin
                }
        
        return results
    
    def _estimate_bandwidth(self, target: np.ndarray, actual: np.ndarray) -> float:
        """Estimate system bandwidth"""
        # Simple bandwidth estimation using response time
        if len(target) < 50:
            return 0
        
        # Find response to input changes
        target_changes = np.diff(target)
        significant_changes = np.where(np.abs(target_changes) > np.std(target_changes) * 2)[0]
        
        if len(significant_changes) == 0:
            return 0
        
        # Calculate average response time
        response_times = []
        for change_idx in significant_changes[:10]:  # Analyze first 10 changes
            if change_idx < 10 or change_idx > len(target) - 20:
                continue
            
            response_window = actual[change_idx:change_idx+20]
            # Simple rise time estimation
            response_time = self._calculate_rise_time(
                np.arange(len(response_window)) / self.sample_rate,
                response_window,
                response_window[0],
                response_window[-1]
            )
            
            if response_time > 0:
                response_times.append(response_time)
        
        if response_times:
            avg_response_time = np.mean(response_times)
            # Bandwidth ≈ 1 / (2π * rise_time)
            return 1 / (2 * np.pi * avg_response_time)
        
        return 0
    
    def _estimate_phase_margin(self, target: np.ndarray, actual: np.ndarray) -> float:
        """Estimate phase margin"""
        # Simplified phase margin estimation
        # In practice, this would require more sophisticated analysis
        
        # Calculate cross-correlation to find phase lag
        if len(target) < 100:
            return 0
        
        correlation = np.correlate(target, actual, mode='full')
        max_corr_idx = np.argmax(correlation)
        center_idx = len(correlation) // 2
        
        # Phase lag in samples
        phase_lag_samples = max_corr_idx - center_idx
        
        # Convert to degrees (rough approximation)
        phase_lag_degrees = (phase_lag_samples / len(target)) * 360
        
        # Phase margin is 180° - phase_lag
        phase_margin = 180 - abs(phase_lag_degrees)
        
        return np.clip(phase_margin, 0, 180)
    
    def _analyze_performance_trends(self, flight_data: Dict) -> Dict:
        """Analyze performance trends over time"""
        if flight_data['rates'].empty:
            return {}
        
        rates_df = flight_data['rates']
        timestamps = rates_df['timestamp'].values
        time_vec = (timestamps - timestamps[0]) / 1e6  # Convert to seconds
        
        trends = {}
        
        for axis in ['roll', 'pitch', 'yaw']:
            target_col = f'{axis}_target'
            actual_col = f'{axis}_actual'
            
            if target_col in rates_df.columns and actual_col in rates_df.columns:
                target = rates_df[target_col].values
                actual = rates_df[actual_col].values
                error = target - actual
                
                # Divide flight into segments for trend analysis
                num_segments = 10
                segment_size = len(error) // num_segments
                
                segment_metrics = []
                for i in range(num_segments):
                    start_idx = i * segment_size
                    end_idx = min((i + 1) * segment_size, len(error))
                    
                    if end_idx - start_idx > 10:  # Ensure minimum segment size
                        segment_error = error[start_idx:end_idx]
                        segment_rms = np.sqrt(np.mean(segment_error**2))
                        segment_max = np.max(np.abs(segment_error))
                        segment_var = np.var(segment_error)
                        
                        segment_metrics.append({
                            'time_start': time_vec[start_idx],
                            'time_end': time_vec[end_idx-1],
                            'rms_error': segment_rms,
                            'max_error': segment_max,
                            'error_variance': segment_var
                        })
                
                # Calculate performance degradation/improvement
                if len(segment_metrics) >= 2:
                    first_half = segment_metrics[:len(segment_metrics)//2]
                    second_half = segment_metrics[len(segment_metrics)//2:]
                    
                    first_avg_rms = np.mean([seg['rms_error'] for seg in first_half])
                    second_avg_rms = np.mean([seg['rms_error'] for seg in second_half])
                    
                    performance_trend = (second_avg_rms - first_avg_rms) / (first_avg_rms + 1e-10)
                else:
                    performance_trend = 0
                
                trends[axis] = {
                    'segment_metrics': segment_metrics,
                    'performance_trend': performance_trend,
                    'trend_direction': 'improving' if performance_trend < -0.1 else 'degrading' if performance_trend > 0.1 else 'stable'
                }
        
        return trends
    
    def _compile_detailed_metrics(self, results: Dict, flight_data: Dict) -> Dict:
        """Compile detailed performance metrics"""
        detailed = {}
        
        # Flight duration and data quality
        if not flight_data['rates'].empty:
            timestamps = flight_data['rates']['timestamp'].values
            flight_duration = (timestamps[-1] - timestamps[0]) / 1e6  # seconds
            data_rate = len(timestamps) / flight_duration
            
            detailed['flight_info'] = {
                'duration_seconds': flight_duration,
                'data_rate_hz': data_rate,
                'total_samples': len(timestamps),
                'data_quality': 'good' if data_rate > 200 else 'poor' if data_rate < 50 else 'fair'
            }
        
        # Control authority analysis
        detailed['control_authority'] = {}
        for axis in ['roll', 'pitch', 'yaw']:
            if axis in results and results[axis]['sample_count'] > 0:
                # Calculate control authority utilization
                rates_df = flight_data['rates']
                target_col = f'{axis}_target'
                actual_col = f'{axis}_actual'
                
                if target_col in rates_df.columns and actual_col in rates_df.columns:
                    target = rates_df[target_col].values
                    actual = rates_df[actual_col].values
                    
                    target_range = np.max(target) - np.min(target)
                    actual_range = np.max(actual) - np.min(actual)
                    
                    # Response linearity
                    if target_range > 0.1:  # Only if there's significant input
                        correlation = np.corrcoef(target, actual)[0, 1]
                        linearity_score = max(0, correlation) * 100
                    else:
                        linearity_score = 0
                    
                    detailed['control_authority'][axis] = {
                        'target_range': target_range,
                        'actual_range': actual_range,
                        'authority_utilization': min(100, (actual_range / max(target_range, 0.1)) * 100),
                        'linearity_score': linearity_score,
                        'max_rate_achieved': np.max(np.abs(actual))
                    }
        
        # Saturation analysis
        detailed['saturation_analysis'] = self._analyze_saturation(flight_data)
        
        # Performance consistency
        detailed['consistency_metrics'] = self._analyze_consistency(flight_data)
        
        return detailed
    
    def _analyze_saturation(self, flight_data: Dict) -> Dict:
        """Analyze control saturation events"""
        saturation_analysis = {}
        
        for axis in ['roll', 'pitch', 'yaw']:
            rates_df = flight_data['rates']
            target_col = f'{axis}_target'
            actual_col = f'{axis}_actual'
            
            if target_col in rates_df.columns and actual_col in rates_df.columns:
                target = rates_df[target_col].values
                actual = rates_df[actual_col].values
                
                # Define saturation thresholds (typical ArduPilot limits)
                saturation_thresholds = {
                    'roll': 8.0,   # rad/s
                    'pitch': 8.0,  # rad/s
                    'yaw': 4.0     # rad/s
                }
                
                threshold = saturation_thresholds.get(axis, 8.0)
                
                # Find saturation events
                saturated_samples = np.abs(actual) > (threshold * 0.9)  # 90% of max
                saturation_percentage = np.sum(saturated_samples) / len(saturated_samples) * 100
                
                # Find consecutive saturation events
                saturation_events = []
                in_saturation = False
                event_start = 0
                
                for i, is_saturated in enumerate(saturated_samples):
                    if is_saturated and not in_saturation:
                        in_saturation = True
                        event_start = i
                    elif not is_saturated and in_saturation:
                        in_saturation = False
                        event_duration = i - event_start
                        if event_duration > 5:  # Only count significant events
                            saturation_events.append({
                                'start_sample': event_start,
                                'duration_samples': event_duration,
                                'max_rate': np.max(np.abs(actual[event_start:i]))
                            })
                
                saturation_analysis[axis] = {
                    'saturation_percentage': saturation_percentage,
                    'saturation_events': len(saturation_events),
                    'longest_event_samples': max([e['duration_samples'] for e in saturation_events], default=0),
                    'severity': 'high' if saturation_percentage > 10 else 'medium' if saturation_percentage > 2 else 'low'
                }
        
        return saturation_analysis
    
    def _analyze_consistency(self, flight_data: Dict) -> Dict:
        """Analyze performance consistency across flight"""
        consistency = {}
        
        for axis in ['roll', 'pitch', 'yaw']:
            rates_df = flight_data['rates']
            target_col = f'{axis}_target'
            actual_col = f'{axis}_actual'
            
            if target_col in rates_df.columns and actual_col in rates_df.columns:
                target = rates_df[target_col].values
                actual = rates_df[actual_col].values
                error = target - actual
                
                # Calculate rolling RMS error
                window_size = 200  # 0.5 seconds at 400Hz
                rolling_rms = []
                
                for i in range(window_size, len(error) - window_size):
                    window_error = error[i-window_size:i+window_size]
                    rolling_rms.append(np.sqrt(np.mean(window_error**2)))
                
                if rolling_rms:
                    rms_variance = np.var(rolling_rms)
                    rms_coefficient_of_variation = np.std(rolling_rms) / (np.mean(rolling_rms) + 1e-10)
                    
                    consistency[axis] = {
                        'rms_variance': rms_variance,
                        'coefficient_of_variation': rms_coefficient_of_variation,
                        'consistency_score': max(0, 100 - rms_coefficient_of_variation * 100),
                        'performance_stability': 'stable' if rms_coefficient_of_variation < 0.2 else 'unstable'
                    }
        
        return consistency
    
    def _prepare_visualization_data(self, flight_data: Dict, results: Dict) -> Dict:
        """Prepare data for visualization"""
        viz_data = {}
        
        if flight_data['rates'].empty:
            return viz_data
        
        rates_df = flight_data['rates']
        timestamps = rates_df['timestamp'].values
        time_vec = (timestamps - timestamps[0]) / 1e6  # Convert to seconds
        
        # Time series data for each axis
        for axis in ['roll', 'pitch', 'yaw']:
            target_col = f'{axis}_target'
            actual_col = f'{axis}_actual'
            
            if target_col in rates_df.columns and actual_col in rates_df.columns:
                target = rates_df[target_col].values
                actual = rates_df[actual_col].values
                error = target - actual
                
                viz_data[axis] = {
                    'time': time_vec,
                    'target': target,
                    'actual': actual,
                    'error': error,
                    'performance_score': results[axis]['performance_score']
                }
        
        # Frequency domain data
        viz_data['frequency_data'] = {}
        for axis in ['roll', 'pitch', 'yaw']:
            if axis in viz_data:
                error = viz_data[axis]['error']
                if len(error) > 100:
                    dt = np.mean(np.diff(time_vec))
                    freqs = np.fft.fftfreq(len(error), dt)
                    fft = np.abs(np.fft.fft(error))
                    
                    # Only positive frequencies
                    positive_freqs = freqs[1:len(freqs)//2]
                    positive_fft = fft[1:len(fft)//2]
                    
                    viz_data['frequency_data'][axis] = {
                        'frequencies': positive_freqs,
                        'magnitude': positive_fft
                    }
        
        return viz_data
    
    def generate_performance_graphs(self, results: Dict, output_dir: str = "output") -> List[str]:
        """Generate performance visualization graphs"""
        os.makedirs(output_dir, exist_ok=True)
        generated_files = []
        
        viz_data = results.get('visualization_data', {})
        if not viz_data:
            return generated_files
        
        # 1. Time series plot for all axes
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle('Flight Performance Analysis - Time Series', fontsize=16, fontweight='bold')
        
        for i, axis in enumerate(['roll', 'pitch', 'yaw']):
            if axis in viz_data:
                data = viz_data[axis]
                time = data['time']
                target = data['target']
                actual = data['actual']
                score = data['performance_score']
                
                axes[i].plot(time, target, 'b-', label='Target', linewidth=1.5, alpha=0.8)
                axes[i].plot(time, actual, 'r-', label='Actual', linewidth=1.5, alpha=0.8)
                axes[i].fill_between(time, target, actual, alpha=0.3, color='orange', label='Error')
                
                axes[i].set_ylabel(f'{axis.capitalize()} Rate\\n(rad/s)', fontsize=11)
                axes[i].set_title(f'{axis.capitalize()} Axis - Performance Score: {score:.1f}/100', fontsize=12)
                axes[i].legend(loc='upper right')
                axes[i].grid(True, alpha=0.3)
                
                # Add performance indicator
                if score >= 70:
                    color = 'green'
                elif score >= 50:
                    color = 'orange'
                else:
                    color = 'red'
                axes[i].axhspan(axes[i].get_ylim()[0], axes[i].get_ylim()[1], 
                               alpha=0.1, color=color, zorder=0)
        
        axes[-1].set_xlabel('Time (seconds)', fontsize=11)
        plt.tight_layout()
        
        time_series_file = os.path.join(output_dir, 'performance_time_series.png')
        plt.savefig(time_series_file, dpi=300, bbox_inches='tight')
        plt.close()
        generated_files.append(time_series_file)
        
        # 2. Frequency domain analysis
        if 'frequency_data' in viz_data:
            fig, axes = plt.subplots(3, 1, figsize=(12, 10))
            fig.suptitle('Frequency Domain Analysis', fontsize=16, fontweight='bold')
            
            for i, axis in enumerate(['roll', 'pitch', 'yaw']):
                if axis in viz_data['frequency_data']:
                    freq_data = viz_data['frequency_data'][axis]
                    freqs = freq_data['frequencies']
                    magnitude = freq_data['magnitude']
                    
                    axes[i].semilogx(freqs, 20 * np.log10(magnitude + 1e-10), 'b-', linewidth=1.5)
                    axes[i].set_ylabel(f'{axis.capitalize()}\\nMagnitude (dB)', fontsize=11)
                    axes[i].set_title(f'{axis.capitalize()} Axis - Error Spectrum', fontsize=12)
                    axes[i].grid(True, alpha=0.3)
                    axes[i].set_xlim(0.1, 100)
                    
                    # Highlight problematic frequencies
                    peak_idx = np.argmax(magnitude[freqs > 1])  # Ignore DC component
                    if peak_idx < len(freqs) - 1:
                        peak_freq = freqs[freqs > 1][peak_idx]
                        axes[i].axvline(peak_freq, color='red', linestyle='--', alpha=0.7, 
                                       label=f'Peak: {peak_freq:.1f} Hz')
                        axes[i].legend()
            
            axes[-1].set_xlabel('Frequency (Hz)', fontsize=11)
            plt.tight_layout()
            
            freq_file = os.path.join(output_dir, 'frequency_analysis.png')
            plt.savefig(freq_file, dpi=300, bbox_inches='tight')
            plt.close()
            generated_files.append(freq_file)
        
        # 3. Performance summary dashboard
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Flight Performance Dashboard', fontsize=16, fontweight='bold')
        
        # Performance scores radar chart
        axes_names = ['Roll', 'Pitch', 'Yaw']
        scores = [results[axis]['performance_score'] for axis in ['roll', 'pitch', 'yaw']]
        
        angles = np.linspace(0, 2*np.pi, len(axes_names), endpoint=False).tolist()
        scores_plot = scores + [scores[0]]  # Complete the circle
        angles += angles[:1]
        
        ax1.plot(angles, scores_plot, 'o-', linewidth=2, color='blue')
        ax1.fill(angles, scores_plot, alpha=0.25, color='blue')
        ax1.set_xticks(angles[:-1])
        ax1.set_xticklabels(axes_names)
        ax1.set_ylim(0, 100)
        ax1.set_title('Performance Scores by Axis', fontsize=12)
        ax1.grid(True)
        
        # Error statistics
        axes_labels = ['Roll', 'Pitch', 'Yaw']
        rms_errors = [results[axis]['tracking_error_rms'] for axis in ['roll', 'pitch', 'yaw']]
        max_errors = [results[axis]['tracking_error_max'] for axis in ['roll', 'pitch', 'yaw']]
        
        x = np.arange(len(axes_labels))
        width = 0.35
        
        ax2.bar(x - width/2, rms_errors, width, label='RMS Error', alpha=0.8)
        ax2.bar(x + width/2, max_errors, width, label='Max Error', alpha=0.8)
        ax2.set_xlabel('Axis')
        ax2.set_ylabel('Error (rad/s)')
        ax2.set_title('Tracking Error Statistics')
        ax2.set_xticks(x)
        ax2.set_xticklabels(axes_labels)
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Step response analysis
        if results['roll']['step_responses']:
            step_data = []
            for axis in ['roll', 'pitch', 'yaw']:
                valid_steps = [s for s in results[axis]['step_responses'] if s.get('valid', False)]
                if valid_steps:
                    avg_overshoot = np.mean([s['overshoot'] for s in valid_steps])
                    avg_settling = np.mean([s['settling_time'] for s in valid_steps])
                    step_data.append((axis, avg_overshoot, avg_settling))
            
            if step_data:
                axes_names = [d[0].capitalize() for d in step_data]
                overshoots = [d[1] for d in step_data]
                settling_times = [d[2] for d in step_data]
                
                ax3.bar(axes_names, overshoots, alpha=0.8, color='orange')
                ax3.set_ylabel('Overshoot (%)')
                ax3.set_title('Average Step Response Overshoot')
                ax3.grid(True, alpha=0.3)
                
                ax4.bar(axes_names, settling_times, alpha=0.8, color='green')
                ax4.set_ylabel('Settling Time (s)')
                ax4.set_title('Average Step Response Settling Time')
                ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        dashboard_file = os.path.join(output_dir, 'performance_dashboard.png')
        plt.savefig(dashboard_file, dpi=300, bbox_inches='tight')
        plt.close()
        generated_files.append(dashboard_file)
        
        return generated_files
    
    def _empty_axis_result(self) -> Dict:
        """Return empty result for axis with no data"""
        return {
            'tracking_error_rms': 0,
            'tracking_error_max': 0,
            'step_responses': [],
            'oscillation_metrics': {'frequency': 0, 'amplitude': 0, 'damping_ratio': 0},
            'stability_metrics': {'error_variance': 0, 'error_trend': 0, 
                                'variance_ratio': 1, 'stability_score': 0},
            'performance_score': 0,
            'sample_count': 0
        }