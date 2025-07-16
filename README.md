# ArduPilot PID Auto-Tuner

Automatically analyze ArduPilot dataflash logs and generate optimized PID and filter parameters to improve flight performance.

## Features

- **Comprehensive Analysis**: Analyzes PID performance, filter effectiveness, and flight stability
- **Automatic Optimization**: Uses control theory to calculate optimal parameters
- **Human-Readable Recommendations**: Provides clear, actionable insights
- **ArduPilot Integration**: Generates parameter files ready for upload
- **Safety-First Approach**: Emphasizes gradual testing and safety procedures

## Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd pid-autotune
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Install the package:
```bash
pip install -e .
```

## Usage

### Command Line Interface

Basic usage:
```bash
pid-autotune your_flight_log.bin
```

Analysis options:
```bash
# PID analysis only
pid-autotune your_flight_log.bin --analysis-type pid

# Filter analysis only
pid-autotune your_flight_log.bin --analysis-type filter

# Both PID and filter analysis (default)
pid-autotune your_flight_log.bin --analysis-type both
```

Output options:
```bash
# Specify output directory
pid-autotune your_flight_log.bin --output-dir /path/to/output

# Quiet mode (minimal output)
pid-autotune your_flight_log.bin --quiet

# Skip recommendations
pid-autotune your_flight_log.bin --no-recommendations
```

### Python API

```python
from pid_autotune import PIDAutoTuner

# Initialize the auto-tuner
auto_tuner = PIDAutoTuner()

# Analyze a log file
results = auto_tuner.analyze_log('flight_log.bin', analysis_type='both')

# Generate output files
generated_files = auto_tuner.generate_output_files(results, 'output')

# Access analysis results
pid_analysis = results['analysis_data']['pid']
filter_analysis = results['analysis_data']['filter']
recommendations = results['recommendations']
optimized_parameters = results['optimized_parameters']
```

## Output Files

The tool generates several output files:

1. **Parameter File (.param)**: Ready-to-upload ArduPilot parameter file
2. **Analysis Report (.txt)**: Detailed technical analysis
3. **Recommendations (.txt)**: Human-readable improvement suggestions
4. **Parameter Comparison (.csv)**: Before/after parameter comparison
5. **JSON Export (.json)**: Complete results in JSON format

## Analysis Types

### PID Analysis
- **Performance Scoring**: Overall and per-axis performance evaluation
- **Step Response Analysis**: Rise time, overshoot, settling time
- **Stability Assessment**: Oscillation detection and damping analysis
- **Tracking Performance**: Error analysis and responsiveness measurement

### Filter Analysis
- **Noise Detection**: Sensor noise characterization
- **Vibration Analysis**: Frequency domain analysis of vibrations
- **Filter Effectiveness**: Current filter performance evaluation
- **Frequency Response**: System bandwidth and response characteristics

## Safety Guidelines

⚠️ **IMPORTANT SAFETY INFORMATION**

1. **Always backup your original parameters** before applying changes
2. **Test parameter changes gradually** and carefully
3. **Start with small changes** and increase slowly
4. **Always have a safety pilot ready** during testing
5. **Monitor motor temperatures** during tuning
6. **Be prepared to revert** to original parameters if needed

## Testing Procedure

1. Load parameters in Mission Planner or similar GCS
2. Start with altitude hold or loiter mode
3. Test small stick inputs first
4. Gradually increase input magnitude
5. Monitor for oscillations or instability
6. Record new logs for verification

## Requirements

- Python 3.7+
- ArduPilot dataflash logs (.bin format)
- Flight data containing IMU, attitude, and rate controller messages

## Dependencies

- numpy: Numerical computations
- pandas: Data manipulation
- scipy: Signal processing and optimization
- matplotlib: Plotting (optional)
- pymavlink: ArduPilot log parsing
- click: Command-line interface

## Log Requirements

For optimal analysis, your log should contain:
- Flight data (not just ground tests)
- Rate controller messages (RATE)
- Attitude messages (ATT)
- IMU data (IMU, GYRO, ACC)
- Parameter information (PARM)
- RC input/output data (RCIN, RCOU)

## Performance Scoring

The tool uses a 0-100 scoring system:
- **90-100**: Excellent performance
- **80-89**: Good performance
- **70-79**: Fair performance (minor tuning needed)
- **50-69**: Poor performance (significant tuning needed)
- **0-49**: Very poor performance (major issues)

## Troubleshooting

### Common Issues

1. **"No rate controller data found"**
   - Ensure your log contains actual flight data
   - Check that RATE messages are present in the log

2. **"Failed to open log file"**
   - Verify the file is a valid .bin file
   - Check file permissions and path

3. **Poor analysis results**
   - Ensure sufficient flight data (at least 1-2 minutes)
   - Include varied maneuvers for better analysis
   - Check for hardware issues (vibrations, loose connections)

### Getting Better Results

1. **Log Quality**:
   - Use fresh flight data with varied maneuvers
   - Include altitude holds, position changes, and attitude changes
   - Avoid logs with only ground tests or brief flights

2. **Flight Conditions**:
   - Fly in calm conditions for baseline analysis
   - Include both gentle and more aggressive maneuvers
   - Ensure the aircraft is properly balanced

3. **Hardware Check**:
   - Verify motor balance and propeller condition
   - Check for loose connections or mounting issues
   - Ensure IMU is properly isolated from vibrations

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Disclaimer

This tool is provided as-is for educational and research purposes. Users are responsible for:
- Validating all parameter changes
- Following safe testing procedures
- Understanding their aircraft's behavior
- Complying with local regulations

Always prioritize safety and test changes incrementally.