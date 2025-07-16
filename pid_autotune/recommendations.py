"""
Human-readable recommendations generator
Translates technical analysis into actionable insights for users
"""

import numpy as np
from typing import Dict, List, Tuple, Optional

class RecommendationEngine:
    def __init__(self):
        self.priority_levels = {
            'critical': 'CRITICAL',
            'high': 'HIGH',
            'medium': 'MEDIUM',
            'low': 'LOW',
            'info': 'INFO'
        }
        
        self.improvement_categories = {
            'stability': 'Flight Stability',
            'tracking': 'Response Tracking',
            'filtering': 'Noise Filtering',
            'vibration': 'Vibration Reduction',
            'tuning': 'PID Tuning'
        }
    
    def generate_recommendations(self, analysis_data: Dict) -> List[Dict]:
        """
        Generate human-readable recommendations based on analysis
        
        Args:
            analysis_data (Dict): Complete analysis results
            
        Returns:
            List[Dict]: List of recommendations with priorities and actions
        """
        recommendations = []
        
        # PID-related recommendations
        if 'pid' in analysis_data:
            pid_recommendations = self._generate_pid_recommendations(analysis_data['pid'])
            recommendations.extend(pid_recommendations)
        
        # Filter-related recommendations
        if 'filter' in analysis_data:
            filter_recommendations = self._generate_filter_recommendations(analysis_data['filter'])
            recommendations.extend(filter_recommendations)
        
        # Sort recommendations by priority
        priority_order = ['critical', 'high', 'medium', 'low', 'info']
        recommendations.sort(key=lambda x: priority_order.index(x['priority']))
        
        return recommendations
    
    def _generate_pid_recommendations(self, pid_analysis: Dict) -> List[Dict]:
        """Generate PID-specific recommendations"""
        recommendations = []
        
        overall_score = pid_analysis.get('overall_score', 0)
        
        # Overall performance assessment
        if overall_score < 30:
            recommendations.append({
                'priority': 'critical',
                'category': 'tuning',
                'title': 'Poor PID Performance Detected',
                'description': f'Overall PID performance score is {overall_score:.1f}/100. Significant tuning improvements needed.',
                'action': 'Review and apply the optimized PID parameters. Consider starting with conservative values and gradually increasing.',
                'technical_details': 'Low performance score indicates poor tracking, oscillations, or instability.',
                'estimated_impact': 'High - Will significantly improve flight characteristics'
            })
        elif overall_score < 60:
            recommendations.append({
                'priority': 'high',
                'category': 'tuning',
                'title': 'PID Performance Needs Improvement',
                'description': f'Overall PID performance score is {overall_score:.1f}/100. Moderate tuning improvements recommended.',
                'action': 'Apply the suggested PID parameter adjustments and test carefully.',
                'technical_details': 'Performance score indicates room for improvement in tracking and stability.',
                'estimated_impact': 'Medium - Will improve responsiveness and stability'
            })
        
        # Analyze each axis
        for axis in ['roll', 'pitch', 'yaw']:
            if axis in pid_analysis:
                axis_recommendations = self._analyze_axis_performance(axis, pid_analysis[axis])
                recommendations.extend(axis_recommendations)
        
        # Stability analysis
        stability_analysis = pid_analysis.get('stability_analysis', {})
        if not stability_analysis.get('overall_stable', True):
            recommendations.append({
                'priority': 'critical',
                'category': 'stability',
                'title': 'Flight Instability Detected',
                'description': 'Sustained oscillations detected in one or more axes. Flight may be unstable.',
                'action': 'Immediately reduce PID gains, especially P and D gains. Check for mechanical issues.',
                'technical_details': 'Oscillations indicate the control system is approaching instability.',
                'estimated_impact': 'Critical - Essential for safe flight'
            })
        
        return recommendations
    
    def _analyze_axis_performance(self, axis: str, axis_data: Dict) -> List[Dict]:
        """Analyze performance of a single axis"""
        recommendations = []
        
        performance_score = axis_data.get('performance_score', 0)
        tracking_error = axis_data.get('tracking_error_rms', 0)
        oscillation_metrics = axis_data.get('oscillation_metrics', {})
        step_responses = axis_data.get('step_responses', [])
        
        axis_name = axis.capitalize()
        
        # Tracking performance
        if tracking_error > 0.1:
            recommendations.append({
                'priority': 'high',
                'category': 'tracking',
                'title': f'{axis_name} Axis Poor Tracking',
                'description': f'{axis_name} axis has high tracking error ({tracking_error:.3f} rad/s RMS).',
                'action': f'Increase {axis_name} rate P gain. Consider increasing angle P gain if response is slow.',
                'technical_details': f'High RMS tracking error indicates poor following of commanded rates.',
                'estimated_impact': 'Medium - Will improve responsiveness'
            })
        elif tracking_error > 0.05:
            recommendations.append({
                'priority': 'medium',
                'category': 'tracking',
                'title': f'{axis_name} Axis Moderate Tracking Error',
                'description': f'{axis_name} axis has moderate tracking error ({tracking_error:.3f} rad/s RMS).',
                'action': f'Small increase in {axis_name} rate P gain may help.',
                'technical_details': f'Moderate tracking error suggests room for improvement.',
                'estimated_impact': 'Low - Minor improvement in responsiveness'
            })
        
        # Oscillation analysis
        osc_amplitude = oscillation_metrics.get('amplitude', 0)
        osc_frequency = oscillation_metrics.get('frequency', 0)
        damping_ratio = oscillation_metrics.get('damping_ratio', 1.0)
        
        if osc_amplitude > 0.05:
            if damping_ratio < 0.3:
                recommendations.append({
                    'priority': 'high',
                    'category': 'stability',
                    'title': f'{axis_name} Axis Underdamped Oscillations',
                    'description': f'{axis_name} axis shows underdamped oscillations at {osc_frequency:.1f} Hz.',
                    'action': f'Reduce {axis_name} rate P and D gains. Increase rate filter frequency.',
                    'technical_details': f'Damping ratio of {damping_ratio:.2f} indicates underdamped response.',
                    'estimated_impact': 'High - Will reduce oscillations and improve stability'
                })
            elif damping_ratio > 0.9:
                recommendations.append({
                    'priority': 'medium',
                    'category': 'tuning',
                    'title': f'{axis_name} Axis Overdamped Response',
                    'description': f'{axis_name} axis shows slow, overdamped response.',
                    'action': f'Increase {axis_name} rate P gain for faster response.',
                    'technical_details': f'High damping ratio of {damping_ratio:.2f} indicates slow response.',
                    'estimated_impact': 'Medium - Will improve responsiveness'
                })
        
        # Step response analysis
        if step_responses:
            avg_overshoot = np.mean([sr.get('overshoot', 0) for sr in step_responses if sr.get('valid', False)])
            avg_settling_time = np.mean([sr.get('settling_time', 0) for sr in step_responses if sr.get('valid', False)])
            
            if avg_overshoot > 20:  # 20% overshoot
                recommendations.append({
                    'priority': 'medium',
                    'category': 'tuning',
                    'title': f'{axis_name} Axis Excessive Overshoot',
                    'description': f'{axis_name} axis shows {avg_overshoot:.1f}% average overshoot.',
                    'action': f'Reduce {axis_name} rate P gain or increase D gain.',
                    'technical_details': 'High overshoot indicates aggressive tuning.',
                    'estimated_impact': 'Medium - Will reduce overshoot and improve precision'
                })
            
            if avg_settling_time > 0.5:  # 500ms settling time
                recommendations.append({
                    'priority': 'medium',
                    'category': 'tuning',
                    'title': f'{axis_name} Axis Slow Settling',
                    'description': f'{axis_name} axis takes {avg_settling_time:.2f}s to settle.',
                    'action': f'Increase {axis_name} rate P gain and angle P gain.',
                    'technical_details': 'Slow settling indicates conservative tuning.',
                    'estimated_impact': 'Medium - Will improve response speed'
                })
        
        return recommendations
    
    def _generate_filter_recommendations(self, filter_analysis: Dict) -> List[Dict]:
        """Generate filter-specific recommendations"""
        recommendations = []
        
        # Vibration analysis
        vibration_analysis = filter_analysis.get('vibration_analysis', {})
        overall_vibration = vibration_analysis.get('overall_assessment', {})
        
        if overall_vibration.get('vibration_detected', False):
            primary_freq = overall_vibration.get('primary_frequency', 0)
            recommendations.append({
                'priority': 'high',
                'category': 'vibration',
                'title': 'Vibrations Detected',
                'description': f'Significant vibrations detected at {primary_freq:.1f} Hz.',
                'action': f'Enable notch filter at {primary_freq:.1f} Hz. Check motor balance and propeller condition.',
                'technical_details': 'Vibrations can affect flight performance and GPS accuracy.',
                'estimated_impact': 'High - Will improve flight smoothness and navigation accuracy'
            })
        
        # Noise analysis
        noise_analysis = filter_analysis.get('noise_analysis', {})
        noise_level = noise_analysis.get('overall_noise_level', 'moderate')
        
        if noise_level == 'high':
            recommendations.append({
                'priority': 'high',
                'category': 'filtering',
                'title': 'High Sensor Noise Detected',
                'description': 'Sensors show high noise levels affecting control performance.',
                'action': 'Reduce gyro and accelerometer filter frequencies. Check IMU mounting.',
                'technical_details': 'High noise can cause poor control performance and motor heating.',
                'estimated_impact': 'High - Will improve control smoothness and efficiency'
            })
        elif noise_level == 'low':
            recommendations.append({
                'priority': 'low',
                'category': 'filtering',
                'title': 'Low Sensor Noise - Consider Reducing Filtering',
                'description': 'Sensors show low noise levels. Less filtering may improve response.',
                'action': 'Consider increasing gyro and rate controller filter frequencies.',
                'technical_details': 'Low noise allows for less aggressive filtering.',
                'estimated_impact': 'Low - May slightly improve response time'
            })
        
        # Gyro filter analysis
        gyro_analysis = filter_analysis.get('gyro_analysis', {})
        self._analyze_gyro_performance(gyro_analysis, recommendations)
        
        # Accelerometer filter analysis
        accel_analysis = filter_analysis.get('accel_analysis', {})
        self._analyze_accel_performance(accel_analysis, recommendations)
        
        # Rate controller filter analysis
        rate_analysis = filter_analysis.get('rate_controller_analysis', {})
        self._analyze_rate_controller_performance(rate_analysis, recommendations)
        
        return recommendations
    
    def _analyze_gyro_performance(self, gyro_analysis: Dict, recommendations: List[Dict]):
        """Analyze gyroscope performance and add recommendations"""
        
        for axis, axis_data in gyro_analysis.items():
            if not isinstance(axis_data, dict):
                continue
                
            vibration_analysis = axis_data.get('vibration_analysis', {})
            if vibration_analysis.get('vibration_detected', False):
                vib_freq = vibration_analysis.get('vibration_frequency', 0)
                recommendations.append({
                    'priority': 'medium',
                    'category': 'vibration',
                    'title': f'Gyro {axis.upper()}-axis Vibration',
                    'description': f'Vibration detected in gyro {axis.upper()}-axis at {vib_freq:.1f} Hz.',
                    'action': f'Check mechanical setup. Consider notch filter at {vib_freq:.1f} Hz.',
                    'technical_details': 'Gyro vibrations affect rate control performance.',
                    'estimated_impact': 'Medium - Will improve control precision'
                })
            
            # Check for extremely high or low signal levels
            rms_level = axis_data.get('rms_level', 0)
            if rms_level > 10:  # Very high gyro rates
                recommendations.append({
                    'priority': 'medium',
                    'category': 'tuning',
                    'title': f'High Gyro Activity on {axis.upper()}-axis',
                    'description': f'Gyro {axis.upper()}-axis shows high activity ({rms_level:.1f} rad/s RMS).',
                    'action': 'Check for oscillations or aggressive tuning. Consider reducing PID gains.',
                    'technical_details': 'High gyro activity may indicate instability or aggressive tuning.',
                    'estimated_impact': 'Medium - May improve stability and reduce motor heating'
                })
    
    def _analyze_accel_performance(self, accel_analysis: Dict, recommendations: List[Dict]):
        """Analyze accelerometer performance and add recommendations"""
        
        for axis, axis_data in accel_analysis.items():
            if not isinstance(axis_data, dict):
                continue
                
            # Check for clipping
            clipping_analysis = axis_data.get('clipping_analysis', {})
            if clipping_analysis.get('clipping_detected', False):
                clipping_pct = clipping_analysis.get('clipping_percentage', 0)
                recommendations.append({
                    'priority': 'high',
                    'category': 'filtering',
                    'title': f'Accelerometer {axis.upper()}-axis Clipping',
                    'description': f'Accelerometer {axis.upper()}-axis shows {clipping_pct:.1f}% clipping.',
                    'action': 'Reduce accelerometer filter frequency. Check for excessive vibrations.',
                    'technical_details': 'Clipping indicates signal saturation affecting measurements.',
                    'estimated_impact': 'High - Will improve altitude and position control'
                })
            
            # Check for vibrations
            vibration_analysis = axis_data.get('vibration_analysis', {})
            if vibration_analysis.get('vibration_detected', False):
                vib_freq = vibration_analysis.get('vibration_frequency', 0)
                recommendations.append({
                    'priority': 'medium',
                    'category': 'vibration',
                    'title': f'Accelerometer {axis.upper()}-axis Vibration',
                    'description': f'Accelerometer {axis.upper()}-axis shows vibration at {vib_freq:.1f} Hz.',
                    'action': 'Check IMU mounting. Consider vibration dampening.',
                    'technical_details': 'Accelerometer vibrations affect position estimation.',
                    'estimated_impact': 'Medium - Will improve position hold and navigation'
                })
    
    def _analyze_rate_controller_performance(self, rate_analysis: Dict, recommendations: List[Dict]):
        """Analyze rate controller performance and add recommendations"""
        
        for axis, axis_data in rate_analysis.items():
            if not isinstance(axis_data, dict):
                continue
                
            control_quality = axis_data.get('control_quality', {})
            tracking_quality = control_quality.get('tracking_quality', 0.5)
            control_smoothness = control_quality.get('control_smoothness', 0.5)
            
            axis_name = axis.capitalize()
            
            if tracking_quality < 0.5:
                recommendations.append({
                    'priority': 'medium',
                    'category': 'tracking',
                    'title': f'{axis_name} Rate Controller Poor Tracking',
                    'description': f'{axis_name} rate controller shows poor tracking quality ({tracking_quality:.2f}).',
                    'action': f'Review {axis_name} PID tuning. Consider reducing filter frequency.',
                    'technical_details': 'Poor tracking quality indicates suboptimal control parameters.',
                    'estimated_impact': 'Medium - Will improve response accuracy'
                })
            
            if control_smoothness < 0.5:
                recommendations.append({
                    'priority': 'medium',
                    'category': 'filtering',
                    'title': f'{axis_name} Rate Controller Rough Output',
                    'description': f'{axis_name} rate controller output is rough ({control_smoothness:.2f}).',
                    'action': f'Increase {axis_name} rate controller filter frequency.',
                    'technical_details': 'Rough control output indicates insufficient filtering.',
                    'estimated_impact': 'Medium - Will improve motor efficiency and reduce noise'
                })
    
    def format_recommendations_for_display(self, recommendations: List[Dict]) -> str:
        """Format recommendations for human-readable display"""
        
        if not recommendations:
            return "✅ No specific recommendations - Your flight performance looks good!"
        
        output = []
        output.append("🔧 FLIGHT PERFORMANCE RECOMMENDATIONS")
        output.append("=" * 50)
        
        # Group by priority
        priority_groups = {}
        for rec in recommendations:
            priority = rec['priority']
            if priority not in priority_groups:
                priority_groups[priority] = []
            priority_groups[priority].append(rec)
        
        priority_order = ['critical', 'high', 'medium', 'low', 'info']
        priority_icons = {
            'critical': '🚨',
            'high': '⚠️',
            'medium': '🔶',
            'low': '💡',
            'info': 'ℹ️'
        }
        
        for priority in priority_order:
            if priority in priority_groups:
                output.append(f"\\n{priority_icons[priority]} {priority.upper()} PRIORITY")
                output.append("-" * 30)
                
                for i, rec in enumerate(priority_groups[priority], 1):
                    output.append(f"\\n{i}. {rec['title']}")
                    output.append(f"   📝 {rec['description']}")
                    output.append(f"   🔧 ACTION: {rec['action']}")
                    output.append(f"   📊 IMPACT: {rec['estimated_impact']}")
                    if rec.get('technical_details'):
                        output.append(f"   🔬 TECHNICAL: {rec['technical_details']}")
        
        # Add summary
        critical_count = len(priority_groups.get('critical', []))
        high_count = len(priority_groups.get('high', []))
        
        output.append(f"\\n📈 SUMMARY")
        output.append("-" * 20)
        output.append(f"Total recommendations: {len(recommendations)}")
        if critical_count > 0:
            output.append(f"🚨 Critical issues: {critical_count} - Address immediately!")
        if high_count > 0:
            output.append(f"⚠️ High priority: {high_count} - Address soon")
        
        return "\\n".join(output)
    
    def generate_parameter_change_summary(self, original_params: Dict, 
                                        optimized_params: Dict) -> str:
        """Generate a summary of parameter changes"""
        
        if not optimized_params:
            return "No parameter changes recommended."
        
        output = []
        output.append("📊 PARAMETER CHANGES SUMMARY")
        output.append("=" * 40)
        
        # Group parameters by type
        pid_params = {}
        filter_params = {}
        
        for param, value in optimized_params.items():
            if 'ATC_' in param:
                pid_params[param] = value
            elif 'INS_' in param:
                filter_params[param] = value
            else:
                filter_params[param] = value  # Default to filter category
        
        if pid_params:
            output.append("\\n🎯 PID PARAMETER CHANGES:")
            for param, new_value in pid_params.items():
                old_value = original_params.get(param, 'Not set')
                if isinstance(old_value, (int, float)) and isinstance(new_value, (int, float)):
                    change_pct = ((new_value - old_value) / old_value) * 100 if old_value != 0 else 0
                    direction = "📈" if change_pct > 0 else "📉"
                    output.append(f"  {param}: {old_value:.4f} → {new_value:.4f} {direction} ({change_pct:+.1f}%)")
                else:
                    output.append(f"  {param}: {old_value} → {new_value}")
        
        if filter_params:
            output.append("\\n🔍 FILTER PARAMETER CHANGES:")
            for param, new_value in filter_params.items():
                old_value = original_params.get(param, 'Not set')
                if isinstance(old_value, (int, float)) and isinstance(new_value, (int, float)):
                    change_pct = ((new_value - old_value) / old_value) * 100 if old_value != 0 else 0
                    direction = "📈" if change_pct > 0 else "📉"
                    output.append(f"  {param}: {old_value:.4f} → {new_value:.4f} {direction} ({change_pct:+.1f}%)")
                else:
                    output.append(f"  {param}: {old_value} → {new_value}")
        
        output.append("\\n⚠️ IMPORTANT NOTES:")
        output.append("- Test parameter changes gradually and carefully")
        output.append("- Start with small changes and increase slowly")
        output.append("- Always have a safety pilot ready")
        output.append("- Back up your original parameters before applying changes")
        
        return "\\n".join(output)