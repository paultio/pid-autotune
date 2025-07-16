#!/usr/bin/env python3
"""
Example usage of the PID Auto-Tuner
"""

from pid_autotune import PIDAutoTuner
import os

def main():
    print("🚁 ArduPilot PID Auto-Tuner - Example Usage")
    print("=" * 50)
    
    # Example log file path (replace with your actual log file)
    log_file = "25-05-07_15-05-34.bin"
    
    # Check if log file exists
    if not os.path.exists(log_file):
        print(f"❌ Log file '{log_file}' not found!")
        print("Please replace 'flight_log.bin' with the path to your actual log file.")
        return
    
    try:
        # Initialize the auto-tuner
        auto_tuner = PIDAutoTuner()
        
        # Analyze the log file
        print(f"📂 Analyzing log file: {log_file}")
        results = auto_tuner.analyze_log(log_file, analysis_type='pid')
        
        # Display results summary
        print("\\n📊 ANALYSIS RESULTS")
        print("-" * 30)
        
        if 'pid' in results.get('analysis_data', {}):
            pid_data = results['analysis_data']['pid']
            overall_score = pid_data.get('overall_score', 0)
            print(f"🎯 Overall PID Performance: {overall_score:.1f}/100")
            
            # Show individual axis scores
            for axis in ['roll', 'pitch', 'yaw']:
                if axis in pid_data:
                    axis_score = pid_data[axis].get('performance_score', 0)
                    rms_error = pid_data[axis].get('tracking_error_rms', 0)
                    max_error = pid_data[axis].get('tracking_error_max', 0)
                    print(f"   {axis.title()}: {axis_score:.1f}/100 (RMS: {rms_error:.3f}, Max: {max_error:.3f} rad/s)")
            
            # Show detailed flight performance metrics
            detailed_metrics = pid_data.get('detailed_metrics', {})
            if detailed_metrics:
                print("\\n🔍 DETAILED PERFORMANCE METRICS")
                print("-" * 30)
                
                # Flight information
                flight_info = detailed_metrics.get('flight_info', {})
                if flight_info:
                    duration = flight_info.get('duration_seconds', 0)
                    data_rate = flight_info.get('data_rate_hz', 0)
                    data_quality = flight_info.get('data_quality', 'unknown')
                    print(f"📏 Flight Duration: {duration:.1f} seconds")
                    print(f"📡 Data Rate: {data_rate:.1f} Hz ({data_quality.upper()} quality)")
                
                # Control authority analysis
                control_authority = detailed_metrics.get('control_authority', {})
                if control_authority:
                    print("\\n🎛️  CONTROL AUTHORITY:")
                    for axis in ['roll', 'pitch', 'yaw']:
                        if axis in control_authority:
                            auth_data = control_authority[axis]
                            utilization = auth_data.get('authority_utilization', 0)
                            linearity = auth_data.get('linearity_score', 0)
                            max_rate = auth_data.get('max_rate_achieved', 0)
                            print(f"   {axis.title()}: {utilization:.1f}% utilization, {linearity:.1f}% linearity, {max_rate:.2f} rad/s max")
                
                # Saturation analysis
                saturation_analysis = detailed_metrics.get('saturation_analysis', {})
                if saturation_analysis:
                    print("\\n⚡ SATURATION ANALYSIS:")
                    for axis in ['roll', 'pitch', 'yaw']:
                        if axis in saturation_analysis:
                            sat_data = saturation_analysis[axis]
                            percentage = sat_data.get('saturation_percentage', 0)
                            events = sat_data.get('saturation_events', 0)
                            severity = sat_data.get('severity', 'unknown')
                            
                            if percentage > 0:
                                print(f"   {axis.title()}: {percentage:.1f}% saturated, {events} events ({severity.upper()})")
                            else:
                                print(f"   {axis.title()}: No saturation detected")
                
                # Performance trends
                performance_trends = pid_data.get('performance_trends', {})
                if performance_trends:
                    print("\\n📈 PERFORMANCE TRENDS:")
                    for axis in ['roll', 'pitch', 'yaw']:
                        if axis in performance_trends:
                            trend_data = performance_trends[axis]
                            direction = trend_data.get('trend_direction', 'unknown')
                            trend_value = trend_data.get('performance_trend', 0)
                            
                            if direction == 'improving':
                                icon = '📈'
                            elif direction == 'degrading':
                                icon = '📉'
                            else:
                                icon = '➡️'
                            
                            print(f"   {icon} {axis.title()}: {direction.upper()} ({trend_value:+.1%})")
                
                # Consistency metrics
                consistency_metrics = detailed_metrics.get('consistency_metrics', {})
                if consistency_metrics:
                    print("\\n🎯 PERFORMANCE CONSISTENCY:")
                    for axis in ['roll', 'pitch', 'yaw']:
                        if axis in consistency_metrics:
                            cons_data = consistency_metrics[axis]
                            score = cons_data.get('consistency_score', 0)
                            stability = cons_data.get('performance_stability', 'unknown')
                            
                            if score >= 80:
                                icon = '✅'
                            elif score >= 60:
                                icon = '⚠️'
                            else:
                                icon = '❌'
                            
                            print(f"   {icon} {axis.title()}: {score:.1f}% consistent ({stability.upper()})")
            
            # Show generated graphs
            if 'generated_graphs' in results:
                print("\\n📈 GENERATED PERFORMANCE GRAPHS:")
                for graph_file in results['generated_graphs']:
                    print(f"   📊 {graph_file}")
                print("   Open these PNG files to view detailed performance visualizations")
        
        if 'filter' in results.get('analysis_data', {}):
            filter_data = results['analysis_data']['filter']
            
            # Check for vibrations
            vibration_analysis = filter_data.get('vibration_analysis', {})
            overall_vib = vibration_analysis.get('overall_assessment', {})
            
            if overall_vib.get('vibration_detected', False):
                primary_freq = overall_vib.get('primary_frequency', 0)
                print(f"⚠️  Vibrations detected at {primary_freq:.1f} Hz")
            else:
                print("✅ No significant vibrations detected")
        
        # Show recommendations
        recommendations = results.get('recommendations', [])
        if recommendations:
            print(f"\\n💡 RECOMMENDATIONS ({len(recommendations)} total)")
            print("-" * 30)
            
            # Show top 3 recommendations
            for i, rec in enumerate(recommendations[:3], 1):
                priority = rec['priority'].upper()
                print(f"{i}. [{priority}] {rec['title']}")
                print(f"   {rec['description']}")
                print(f"   Action: {rec['action']}")
                print()
        
        # Show parameter changes
        optimized_params = results.get('optimized_parameters', {})
        if optimized_params:
            print(f"🔧 PARAMETER CHANGES ({len(optimized_params)} parameters)")
            print("-" * 30)
            
            original_params = results.get('original_parameters', {})
            
            for param, new_value in list(optimized_params.items())[:5]:  # Show first 5
                old_value = original_params.get(param, 'Not set')
                print(f"{param}: {old_value} → {new_value}")
            
            if len(optimized_params) > 5:
                print(f"... and {len(optimized_params) - 5} more parameters")
        
        # Generate output files
        print("\\n📄 GENERATING OUTPUT FILES")
        print("-" * 30)
        
        output_dir = "output"
        generated_files = auto_tuner.generate_output_files(results, output_dir)
        
        for file_type, file_path in generated_files.items():
            if file_path and os.path.exists(file_path):
                print(f"✅ {file_type}: {file_path}")
        
        print("\\n🎉 Analysis complete!")
        print("\\n⚠️  IMPORTANT SAFETY REMINDERS:")
        print("• Always backup your original parameters before applying changes")
        print("• Test parameter changes gradually and carefully")
        print("• Always have a safety pilot ready during testing")
        print("• Monitor for oscillations or instability")
        
    except Exception as e:
        print(f"❌ Error during analysis: {str(e)}")
        print("\\nTroubleshooting tips:")
        print("• Ensure the log file is a valid ArduPilot .bin file")
        print("• Check that the log contains flight data (not just ground tests)")
        print("• Verify the log has rate controller and IMU data")

if __name__ == "__main__":
    main()