"""
Command-line interface for PID Auto-Tuner
"""

import click
import os
import sys
from .main import PIDAutoTuner
from .recommendations import RecommendationEngine

@click.command()
@click.argument('log_file', type=click.Path(exists=True))
@click.option('--analysis-type', '-t', 
              type=click.Choice(['pid', 'filter', 'both']), 
              default='both',
              help='Type of analysis to perform')
@click.option('--output-dir', '-o', 
              default='output',
              help='Output directory for results')
@click.option('--quiet', '-q', 
              is_flag=True,
              help='Suppress verbose output')
@click.option('--no-recommendations', 
              is_flag=True,
              help='Skip generating recommendations')
@click.option('--format', '-f',
              type=click.Choice(['param', 'json', 'csv', 'all']),
              default='all',
              help='Output format')
def main(log_file, analysis_type, output_dir, quiet, no_recommendations, format):
    """
    ArduPilot PID Auto-Tuner
    
    Analyze dataflash logs and generate optimized PID and filter parameters.
    
    LOG_FILE: Path to the ArduPilot dataflash log file (.bin)
    """
    
    if not quiet:
        click.echo("🚁 ArduPilot PID Auto-Tuner v1.0.0")
        click.echo("=" * 40)
    
    # Validate log file
    if not log_file.endswith('.bin'):
        click.echo("❌ Error: Log file must be a .bin file", err=True)
        sys.exit(1)
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    try:
        # Initialize auto-tuner
        auto_tuner = PIDAutoTuner()
        
        if not quiet:
            click.echo(f"📂 Analyzing log file: {log_file}")
            click.echo(f"🔍 Analysis type: {analysis_type}")
            click.echo(f"📁 Output directory: {output_dir}")
            click.echo()
        
        # Analyze the log
        with click.progressbar(length=100, label='Analyzing log') as bar:
            # Parse log
            bar.update(20)
            
            # Perform analysis
            results = auto_tuner.analyze_log(log_file, analysis_type)
            bar.update(60)
            
            # Generate outputs
            generated_files = auto_tuner.generate_output_files(results, output_dir)
            bar.update(20)
        
        if not quiet:
            click.echo("✅ Analysis complete!")
            click.echo()
        
        # Display results summary
        display_results_summary(results, quiet)
        
        # Display recommendations
        if not no_recommendations and results.get('recommendations'):
            display_recommendations(results['recommendations'], quiet)
        
        # Display generated files
        display_generated_files(generated_files, quiet)
        
    except Exception as e:
        click.echo(f"❌ Error during analysis: {str(e)}", err=True)
        if not quiet:
            click.echo("\\nTroubleshooting tips:")
            click.echo("• Ensure the log file is a valid ArduPilot .bin file")
            click.echo("• Check that the log contains flight data (not just ground tests)")
            click.echo("• Verify the log has rate controller and IMU data")
        sys.exit(1)

def display_results_summary(results: dict, quiet: bool):
    """Display analysis results summary"""
    if quiet:
        return
    
    click.echo("📊 ANALYSIS RESULTS")
    click.echo("-" * 20)
    
    # PID analysis summary
    if 'pid' in results.get('analysis_data', {}):
        pid_data = results['analysis_data']['pid']
        overall_score = pid_data.get('overall_score', 0)
        
        click.echo(f"🎯 PID Performance Score: {overall_score:.1f}/100")
        
        performance_rating = get_performance_rating(overall_score)
        rating_color = get_rating_color(performance_rating)
        click.echo(f"📈 Performance Rating: {click.style(performance_rating, fg=rating_color)}")
        
        # Individual axis scores
        for axis in ['roll', 'pitch', 'yaw']:
            if axis in pid_data:
                axis_score = pid_data[axis].get('performance_score', 0)
                click.echo(f"   {axis.title()}: {axis_score:.1f}/100")
    
    # Filter analysis summary
    if 'filter' in results.get('analysis_data', {}):
        filter_data = results['analysis_data']['filter']
        
        # Vibration detection
        vibration_analysis = filter_data.get('vibration_analysis', {})
        overall_vib = vibration_analysis.get('overall_assessment', {})
        
        if overall_vib.get('vibration_detected', False):
            primary_freq = overall_vib.get('primary_frequency', 0)
            click.echo(f"⚠️  Vibrations detected at {primary_freq:.1f} Hz")
        else:
            click.echo("✅ No significant vibrations detected")
        
        # Noise level
        noise_analysis = filter_data.get('noise_analysis', {})
        noise_level = noise_analysis.get('overall_noise_level', 'unknown')
        noise_color = {'low': 'green', 'moderate': 'yellow', 'high': 'red'}.get(noise_level, 'white')
        click.echo(f"🔊 Noise Level: {click.style(noise_level.upper(), fg=noise_color)}")
    
    # Parameter changes
    if results.get('optimized_parameters'):
        param_count = len(results['optimized_parameters'])
        click.echo(f"🔧 Parameters to optimize: {param_count}")
    
    click.echo()

def display_recommendations(recommendations: list, quiet: bool):
    """Display recommendations"""
    if quiet:
        return
    
    click.echo("💡 RECOMMENDATIONS")
    click.echo("-" * 20)
    
    if not recommendations:
        click.echo("✅ No specific recommendations - Your flight performance looks good!")
        return
    
    # Count by priority
    priority_counts = {}
    for rec in recommendations:
        priority = rec['priority']
        priority_counts[priority] = priority_counts.get(priority, 0) + 1
    
    # Display priority summary
    priority_colors = {
        'critical': 'red',
        'high': 'yellow',
        'medium': 'blue',
        'low': 'green',
        'info': 'white'
    }
    
    for priority in ['critical', 'high', 'medium', 'low', 'info']:
        if priority in priority_counts:
            count = priority_counts[priority]
            color = priority_colors[priority]
            click.echo(f"{click.style(priority.upper(), fg=color)}: {count} recommendation{'s' if count > 1 else ''}")
    
    click.echo()
    
    # Show top 3 recommendations
    click.echo("🔝 Top Recommendations:")
    for i, rec in enumerate(recommendations[:3], 1):
        priority_color = priority_colors.get(rec['priority'], 'white')
        click.echo(f"{i}. {click.style(rec['title'], fg=priority_color)}")
        click.echo(f"   {rec['description']}")
        click.echo()
    
    if len(recommendations) > 3:
        click.echo(f"... and {len(recommendations) - 3} more recommendations in the detailed report.")
    
    click.echo()

def display_generated_files(generated_files: dict, quiet: bool):
    """Display generated files"""
    if quiet:
        return
    
    click.echo("📄 GENERATED FILES")
    click.echo("-" * 20)
    
    file_descriptions = {
        'parameter_file': '🔧 ArduPilot Parameter File (.param)',
        'analysis_report': '📊 Detailed Analysis Report (.txt)',
        'recommendations_file': '💡 Recommendations Report (.txt)',
        'parameter_comparison': '📋 Parameter Comparison (.csv)',
        'json_export': '📦 JSON Export (.json)'
    }
    
    for file_type, file_path in generated_files.items():
        if file_path and os.path.exists(file_path):
            description = file_descriptions.get(file_type, f'📄 {file_type}')
            click.echo(f"{description}: {file_path}")
    
    click.echo()
    click.echo("⚠️  IMPORTANT: Always backup your original parameters before applying changes!")
    click.echo("🧪 Test new parameters carefully and gradually.")

def get_performance_rating(score: float) -> str:
    """Get performance rating from score"""
    if score >= 90:
        return "EXCELLENT"
    elif score >= 80:
        return "GOOD"
    elif score >= 70:
        return "FAIR"
    elif score >= 50:
        return "POOR"
    else:
        return "VERY POOR"

def get_rating_color(rating: str) -> str:
    """Get color for performance rating"""
    color_map = {
        'EXCELLENT': 'green',
        'GOOD': 'green',
        'FAIR': 'yellow',
        'POOR': 'red',
        'VERY POOR': 'red'
    }
    return color_map.get(rating, 'white')

@click.command()
@click.argument('log_file', type=click.Path(exists=True))
@click.option('--output-dir', '-o', default='output')
def quick_analysis(log_file, output_dir):
    """Quick analysis with minimal output"""
    try:
        auto_tuner = PIDAutoTuner()
        results = auto_tuner.analyze_log(log_file, 'both')
        
        # Generate only parameter file
        if results.get('optimized_parameters'):
            param_file = auto_tuner.output_generator.generate_parameter_file(
                results['optimized_parameters'], output_dir
            )
            click.echo(f"Parameter file: {param_file}")
        
        # Quick summary
        if 'pid' in results.get('analysis_data', {}):
            overall_score = results['analysis_data']['pid'].get('overall_score', 0)
            click.echo(f"Performance score: {overall_score:.1f}/100")
        
        # Critical recommendations only
        critical_recs = [r for r in results.get('recommendations', []) if r['priority'] == 'critical']
        if critical_recs:
            click.echo("⚠️  Critical issues found - check detailed report!")
        
    except Exception as e:
        click.echo(f"Error: {str(e)}", err=True)
        sys.exit(1)

if __name__ == '__main__':
    main()