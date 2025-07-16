"""
Filter analysis for ArduPilot
Analyzes gyro, accelerometer, and rate controller filtering performance
"""

import numpy as np
import pandas as pd
from scipy import signal
from scipy.fft import fft, fftfreq
from typing import Dict, List, Tuple, Optional

class FilterAnalyzer:
    def __init__(self):
        self.sample_rate = 400  # Hz, typical ArduPilot loop rate
        self.nyquist_freq = self.sample_rate / 2
        
    def analyze(self, flight_data: Dict) -> Dict:
        """
        Analyze filter performance from flight data
        
        Args:
            flight_data (Dict): Parsed flight data
            
        Returns:
            Dict: Filter analysis results
        """
        results = {
            'gyro_analysis': self._analyze_gyro_filtering(flight_data),
            'accel_analysis': self._analyze_accel_filtering(flight_data),
            'rate_controller_analysis': self._analyze_rate_controller_filtering(flight_data),
            'noise_analysis': self._analyze_noise_characteristics(flight_data),
            'vibration_analysis': self._analyze_vibration_levels(flight_data),
            'filter_effectiveness': {},
            'recommended_frequencies': {}
        }
        
        # Calculate filter effectiveness
        results['filter_effectiveness'] = self._calculate_filter_effectiveness(results)
        
        # Recommend filter frequencies
        results['recommended_frequencies'] = self._recommend_filter_frequencies(results)
        
        return results
    
    def _analyze_gyro_filtering(self, flight_data: Dict) -> Dict:
        """Analyze gyroscope filtering performance"""
        if flight_data['gyro'].empty and flight_data['imu'].empty:
            return self._empty_gyro_result()
        
        # Use gyro data if available, otherwise use IMU gyro data
        if not flight_data['gyro'].empty:
            gyro_data = flight_data['gyro']
            gyro_x = gyro_data['x'].values
            gyro_y = gyro_data['y'].values
            gyro_z = gyro_data['z'].values
        else:
            imu_data = flight_data['imu']
            gyro_x = imu_data['gyro_x'].values
            gyro_y = imu_data['gyro_y'].values
            gyro_z = imu_data['gyro_z'].values
        
        # Analyze each axis
        results = {}
        axes = ['x', 'y', 'z']
        gyro_signals = [gyro_x, gyro_y, gyro_z]
        
        for axis, gyro_signal in zip(axes, gyro_signals):
            if len(gyro_signal) < 100:
                continue
                
            # Frequency domain analysis
            freq_analysis = self._frequency_domain_analysis(gyro_signal)
            
            # Noise analysis
            noise_analysis = self._noise_analysis(gyro_signal)
            
            # Vibration detection
            vibration_analysis = self._detect_vibrations(gyro_signal)
            
            results[axis] = {
                'frequency_analysis': freq_analysis,
                'noise_analysis': noise_analysis,
                'vibration_analysis': vibration_analysis,
                'rms_level': np.sqrt(np.mean(gyro_signal**2)),
                'peak_level': np.max(np.abs(gyro_signal))
            }
        
        return results
    
    def _analyze_accel_filtering(self, flight_data: Dict) -> Dict:
        """Analyze accelerometer filtering performance"""
        if flight_data['accelerometer'].empty and flight_data['imu'].empty:
            return self._empty_accel_result()
        
        # Use accelerometer data if available, otherwise use IMU accel data
        if not flight_data['accelerometer'].empty:
            accel_data = flight_data['accelerometer']
            accel_x = accel_data['x'].values
            accel_y = accel_data['y'].values
            accel_z = accel_data['z'].values
        else:
            imu_data = flight_data['imu']
            accel_x = imu_data['accel_x'].values
            accel_y = imu_data['accel_y'].values
            accel_z = imu_data['accel_z'].values
        
        # Remove gravity component from Z-axis (approximate)
        accel_z_no_gravity = accel_z - np.mean(accel_z)
        
        # Analyze each axis
        results = {}
        axes = ['x', 'y', 'z']
        accel_signals = [accel_x, accel_y, accel_z_no_gravity]
        
        for axis, accel_signal in zip(axes, accel_signals):
            if len(accel_signal) < 100:
                continue
                
            # Frequency domain analysis
            freq_analysis = self._frequency_domain_analysis(accel_signal)
            
            # Vibration analysis
            vibration_analysis = self._detect_vibrations(accel_signal)
            
            # Clipping detection
            clipping_analysis = self._detect_clipping(accel_signal)
            
            results[axis] = {
                'frequency_analysis': freq_analysis,
                'vibration_analysis': vibration_analysis,
                'clipping_analysis': clipping_analysis,
                'rms_level': np.sqrt(np.mean(accel_signal**2)),
                'peak_level': np.max(np.abs(accel_signal))
            }
        
        return results
    
    def _analyze_rate_controller_filtering(self, flight_data: Dict) -> Dict:
        """Analyze rate controller input filtering"""
        if flight_data['rates'].empty:
            return {}
        
        rates_df = flight_data['rates']
        
        results = {}
        axes = ['roll', 'pitch', 'yaw']
        
        for axis in axes:
            actual_col = f'{axis}_actual'
            target_col = f'{axis}_target'
            
            if actual_col not in rates_df.columns:
                continue
            
            actual_rates = rates_df[actual_col].values
            target_rates = rates_df[target_col].values if target_col in rates_df.columns else None
            
            if len(actual_rates) < 100:
                continue
            
            # Analyze rate signal filtering
            freq_analysis = self._frequency_domain_analysis(actual_rates)
            
            # Noise in rate controller
            noise_analysis = self._noise_analysis(actual_rates)
            
            # Control signal quality
            control_quality = self._analyze_control_signal_quality(actual_rates, target_rates)
            
            results[axis] = {
                'frequency_analysis': freq_analysis,
                'noise_analysis': noise_analysis,
                'control_quality': control_quality,
                'rms_level': np.sqrt(np.mean(actual_rates**2)),
                'peak_level': np.max(np.abs(actual_rates))
            }
        
        return results
    
    def _frequency_domain_analysis(self, signal_data: np.ndarray) -> Dict:
        """Perform frequency domain analysis of signal"""
        if len(signal_data) < 100:
            return {'dominant_frequency': 0, 'frequency_spread': 0, 'spectral_peaks': []}
        
        # Remove DC component
        signal_ac = signal_data - np.mean(signal_data)
        
        # Calculate FFT
        n_samples = len(signal_ac)
        fft_result = fft(signal_ac)
        frequencies = fftfreq(n_samples, 1/self.sample_rate)
        
        # Only consider positive frequencies
        positive_freqs = frequencies[:n_samples//2]
        positive_fft = np.abs(fft_result[:n_samples//2])
        
        # Find dominant frequency
        if len(positive_fft) > 1:
            dominant_freq_idx = np.argmax(positive_fft[1:]) + 1  # Skip DC
            dominant_frequency = positive_freqs[dominant_freq_idx]
        else:
            dominant_frequency = 0
        
        # Calculate frequency spread (spectral width)
        power_spectrum = positive_fft**2
        total_power = np.sum(power_spectrum)
        
        if total_power > 0:
            # Calculate weighted mean frequency
            mean_freq = np.sum(positive_freqs * power_spectrum) / total_power
            
            # Calculate frequency spread (standard deviation)
            freq_variance = np.sum(power_spectrum * (positive_freqs - mean_freq)**2) / total_power
            frequency_spread = np.sqrt(freq_variance)
        else:
            frequency_spread = 0
        
        # Find spectral peaks
        spectral_peaks = self._find_spectral_peaks(positive_freqs, positive_fft)
        
        return {
            'dominant_frequency': dominant_frequency,
            'frequency_spread': frequency_spread,
            'spectral_peaks': spectral_peaks,
            'power_spectrum': power_spectrum,
            'frequencies': positive_freqs
        }
    
    def _find_spectral_peaks(self, frequencies: np.ndarray, 
                           magnitude: np.ndarray) -> List[Dict]:
        """Find significant peaks in frequency spectrum"""
        if len(magnitude) < 10:
            return []
        
        # Find peaks with minimum height and prominence
        threshold = np.max(magnitude) * 0.1  # 10% of max
        peaks, properties = signal.find_peaks(
            magnitude, 
            height=threshold, 
            distance=5,  # Minimum distance between peaks
            prominence=threshold * 0.5
        )
        
        spectral_peaks = []
        for peak_idx in peaks:
            if peak_idx < len(frequencies):
                spectral_peaks.append({
                    'frequency': frequencies[peak_idx],
                    'magnitude': magnitude[peak_idx],
                    'prominence': properties['prominences'][list(peaks).index(peak_idx)]
                })
        
        # Sort by magnitude
        spectral_peaks.sort(key=lambda x: x['magnitude'], reverse=True)
        
        return spectral_peaks[:10]  # Return top 10 peaks
    
    def _noise_analysis(self, signal_data: np.ndarray) -> Dict:
        """Analyze noise characteristics in signal"""
        if len(signal_data) < 100:
            return {'noise_level': 0, 'snr': 0, 'noise_type': 'unknown'}
        
        # Remove slow variations (high-pass filter)
        sos = signal.butter(4, 1.0, btype='high', fs=self.sample_rate, output='sos')
        noise_signal = signal.sosfilt(sos, signal_data)
        
        # Calculate noise metrics
        noise_rms = np.sqrt(np.mean(noise_signal**2))
        signal_rms = np.sqrt(np.mean(signal_data**2))
        
        snr = 20 * np.log10(signal_rms / (noise_rms + 1e-10))
        
        # Analyze noise type
        noise_type = self._classify_noise_type(noise_signal)
        
        return {
            'noise_level': noise_rms,
            'snr': snr,
            'noise_type': noise_type
        }
    
    def _classify_noise_type(self, noise_signal: np.ndarray) -> str:
        """Classify type of noise in signal"""
        # Simple noise classification based on spectral characteristics
        freq_analysis = self._frequency_domain_analysis(noise_signal)
        
        # Check for white noise (flat spectrum)
        power_spectrum = freq_analysis['power_spectrum']
        if len(power_spectrum) > 10:
            # Calculate spectral flatness
            geometric_mean = np.exp(np.mean(np.log(power_spectrum + 1e-10)))
            arithmetic_mean = np.mean(power_spectrum)
            spectral_flatness = geometric_mean / arithmetic_mean
            
            if spectral_flatness > 0.5:
                return 'white'
            elif freq_analysis['dominant_frequency'] < 5:
                return 'low_frequency'
            else:
                return 'colored'
        
        return 'unknown'
    
    def _detect_vibrations(self, signal_data: np.ndarray) -> Dict:
        """Detect vibrations in signal"""
        if len(signal_data) < 100:
            return {'vibration_detected': False, 'vibration_frequency': 0, 'vibration_amplitude': 0}
        
        freq_analysis = self._frequency_domain_analysis(signal_data)
        
        # Look for strong peaks in typical vibration frequency ranges
        vibration_detected = False
        vibration_frequency = 0
        vibration_amplitude = 0
        
        for peak in freq_analysis['spectral_peaks']:
            # Check for common vibration frequencies
            if 10 <= peak['frequency'] <= 200:  # Typical range for drone vibrations
                if peak['magnitude'] > np.std(signal_data) * 3:  # Significant peak
                    vibration_detected = True
                    vibration_frequency = peak['frequency']
                    vibration_amplitude = peak['magnitude']
                    break
        
        return {
            'vibration_detected': vibration_detected,
            'vibration_frequency': vibration_frequency,
            'vibration_amplitude': vibration_amplitude
        }
    
    def _detect_clipping(self, signal_data: np.ndarray) -> Dict:
        """Detect signal clipping"""
        if len(signal_data) < 10:
            return {'clipping_detected': False, 'clipping_percentage': 0}
        
        # Simple clipping detection based on signal saturation
        signal_range = np.max(signal_data) - np.min(signal_data)
        if signal_range == 0:
            return {'clipping_detected': False, 'clipping_percentage': 0}
        
        # Check for values at extremes
        threshold = signal_range * 0.05  # 5% of range
        max_val = np.max(signal_data)
        min_val = np.min(signal_data)
        
        clipped_high = np.sum(signal_data > (max_val - threshold))
        clipped_low = np.sum(signal_data < (min_val + threshold))
        
        clipping_percentage = (clipped_high + clipped_low) / len(signal_data) * 100
        
        return {
            'clipping_detected': clipping_percentage > 1.0,  # 1% threshold
            'clipping_percentage': clipping_percentage
        }
    
    def _analyze_control_signal_quality(self, actual_rates: np.ndarray, 
                                      target_rates: Optional[np.ndarray]) -> Dict:
        """Analyze quality of control signals"""
        if target_rates is None:
            return {'tracking_quality': 0, 'control_smoothness': 0}
        
        if len(actual_rates) != len(target_rates) or len(actual_rates) < 10:
            return {'tracking_quality': 0, 'control_smoothness': 0}
        
        # Calculate tracking quality
        error = actual_rates - target_rates
        tracking_rms = np.sqrt(np.mean(error**2))
        target_rms = np.sqrt(np.mean(target_rates**2))
        
        tracking_quality = 1 / (1 + tracking_rms / (target_rms + 1e-10))
        
        # Calculate control smoothness (derivative analysis)
        actual_derivative = np.diff(actual_rates)
        control_smoothness = 1 / (1 + np.std(actual_derivative))
        
        return {
            'tracking_quality': tracking_quality,
            'control_smoothness': control_smoothness
        }
    
    def _analyze_noise_characteristics(self, flight_data: Dict) -> Dict:
        """Analyze overall noise characteristics"""
        noise_results = {}
        
        # Analyze IMU noise
        if not flight_data['imu'].empty:
            imu_data = flight_data['imu']
            
            # Gyro noise
            gyro_signals = [imu_data['gyro_x'].values, imu_data['gyro_y'].values, imu_data['gyro_z'].values]
            gyro_noise = [self._noise_analysis(signal) for signal in gyro_signals]
            
            # Accel noise
            accel_signals = [imu_data['accel_x'].values, imu_data['accel_y'].values, imu_data['accel_z'].values]
            accel_noise = [self._noise_analysis(signal) for signal in accel_signals]
            
            noise_results['gyro_noise'] = gyro_noise
            noise_results['accel_noise'] = accel_noise
        
        # Overall noise assessment
        if 'gyro_noise' in noise_results:
            avg_gyro_snr = np.mean([n['snr'] for n in noise_results['gyro_noise']])
            avg_accel_snr = np.mean([n['snr'] for n in noise_results['accel_noise']])
            
            noise_results['overall_noise_level'] = self._categorize_noise_level(avg_gyro_snr, avg_accel_snr)
        
        return noise_results
    
    def _categorize_noise_level(self, gyro_snr: float, accel_snr: float) -> str:
        """Categorize overall noise level"""
        avg_snr = (gyro_snr + accel_snr) / 2
        
        if avg_snr > 40:
            return 'low'
        elif avg_snr > 20:
            return 'moderate'
        else:
            return 'high'
    
    def _analyze_vibration_levels(self, flight_data: Dict) -> Dict:
        """Analyze vibration levels across all sensors"""
        vibration_results = {}
        
        # Analyze IMU vibrations
        if not flight_data['imu'].empty:
            imu_data = flight_data['imu']
            
            # Check each axis for vibrations
            for axis in ['x', 'y', 'z']:
                gyro_col = f'gyro_{axis}'
                accel_col = f'accel_{axis}'
                
                if gyro_col in imu_data.columns:
                    gyro_vib = self._detect_vibrations(imu_data[gyro_col].values)
                    vibration_results[f'gyro_{axis}'] = gyro_vib
                
                if accel_col in imu_data.columns:
                    accel_vib = self._detect_vibrations(imu_data[accel_col].values)
                    vibration_results[f'accel_{axis}'] = accel_vib
        
        # Overall vibration assessment
        vibration_detected = any(
            result.get('vibration_detected', False) 
            for result in vibration_results.values()
        )
        
        if vibration_detected:
            vibration_frequencies = [
                result['vibration_frequency'] 
                for result in vibration_results.values() 
                if result.get('vibration_detected', False)
            ]
            primary_vibration_freq = np.mean(vibration_frequencies) if vibration_frequencies else 0
        else:
            primary_vibration_freq = 0
        
        vibration_results['overall_assessment'] = {
            'vibration_detected': vibration_detected,
            'primary_frequency': primary_vibration_freq
        }
        
        return vibration_results
    
    def _calculate_filter_effectiveness(self, analysis_results: Dict) -> Dict:
        """Calculate effectiveness of current filtering"""
        effectiveness = {}
        
        # Gyro filter effectiveness
        if 'gyro_analysis' in analysis_results:
            gyro_noise_levels = []
            for axis_result in analysis_results['gyro_analysis'].values():
                if 'noise_analysis' in axis_result:
                    gyro_noise_levels.append(axis_result['noise_analysis']['snr'])
            
            if gyro_noise_levels:
                avg_gyro_snr = np.mean(gyro_noise_levels)
                effectiveness['gyro_filter'] = min(100, max(0, (avg_gyro_snr - 10) * 5))
        
        # Accelerometer filter effectiveness
        if 'accel_analysis' in analysis_results:
            accel_vibration_levels = []
            for axis_result in analysis_results['accel_analysis'].values():
                if 'vibration_analysis' in axis_result:
                    vib_detected = axis_result['vibration_analysis']['vibration_detected']
                    accel_vibration_levels.append(0 if vib_detected else 100)
            
            if accel_vibration_levels:
                effectiveness['accel_filter'] = np.mean(accel_vibration_levels)
        
        # Rate controller filter effectiveness
        if 'rate_controller_analysis' in analysis_results:
            rate_quality_scores = []
            for axis_result in analysis_results['rate_controller_analysis'].values():
                if 'control_quality' in axis_result:
                    quality = axis_result['control_quality']['tracking_quality']
                    rate_quality_scores.append(quality * 100)
            
            if rate_quality_scores:
                effectiveness['rate_controller_filter'] = np.mean(rate_quality_scores)
        
        return effectiveness
    
    def _recommend_filter_frequencies(self, analysis_results: Dict) -> Dict:
        """Recommend filter frequencies based on analysis"""
        recommendations = {}
        
        # Gyro filter recommendations
        if 'vibration_analysis' in analysis_results:
            vibration_freq = analysis_results['vibration_analysis']['overall_assessment']['primary_frequency']
            if vibration_freq > 0:
                # Recommend notch filter frequency
                recommendations['gyro_notch_freq'] = vibration_freq
                recommendations['gyro_notch_bandwidth'] = vibration_freq * 0.2  # 20% of center frequency
        
        # Low-pass filter recommendations
        if 'noise_analysis' in analysis_results:
            noise_level = analysis_results['noise_analysis'].get('overall_noise_level', 'moderate')
            
            if noise_level == 'high':
                recommendations['gyro_lpf'] = 30  # Aggressive filtering
                recommendations['rate_controller_lpf'] = 25
            elif noise_level == 'moderate':
                recommendations['gyro_lpf'] = 50  # Balanced filtering
                recommendations['rate_controller_lpf'] = 40
            else:  # low noise
                recommendations['gyro_lpf'] = 80  # Light filtering
                recommendations['rate_controller_lpf'] = 60
        
        return recommendations
    
    def _empty_gyro_result(self) -> Dict:
        """Return empty result for gyro analysis"""
        return {
            'x': {'frequency_analysis': {}, 'noise_analysis': {}, 'vibration_analysis': {}},
            'y': {'frequency_analysis': {}, 'noise_analysis': {}, 'vibration_analysis': {}},
            'z': {'frequency_analysis': {}, 'noise_analysis': {}, 'vibration_analysis': {}}
        }
    
    def _empty_accel_result(self) -> Dict:
        """Return empty result for accelerometer analysis"""
        return {
            'x': {'frequency_analysis': {}, 'vibration_analysis': {}, 'clipping_analysis': {}},
            'y': {'frequency_analysis': {}, 'vibration_analysis': {}, 'clipping_analysis': {}},
            'z': {'frequency_analysis': {}, 'vibration_analysis': {}, 'clipping_analysis': {}}
        }