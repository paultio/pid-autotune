"""
Main entry point for PID auto-tuning tool
"""

from .log_parser import LogParser
from .pid_analyzer import PIDAnalyzer
from .filter_analyzer import FilterAnalyzer
from .parameter_optimizer import ParameterOptimizer
from .recommendations import RecommendationEngine
from .output_generator import OutputGenerator

class PIDAutoTuner:
    def __init__(self):
        self.log_parser = LogParser()
        self.pid_analyzer = PIDAnalyzer()
        self.filter_analyzer = FilterAnalyzer()
        self.optimizer = ParameterOptimizer()
        self.recommender = RecommendationEngine()
        self.output_generator = OutputGenerator()
    
    def analyze_log(self, log_file_path, analysis_type="both"):
        """
        Analyze dataflash log and generate optimized parameters
        
        Args:
            log_file_path (str): Path to the dataflash log file
            analysis_type (str): "pid", "filter", or "both"
        
        Returns:
            dict: Analysis results and recommendations
        """
        print(f"Parsing log file: {log_file_path}")
        flight_data = self.log_parser.parse_log(log_file_path)
        
        results = {
            "original_parameters": {},
            "optimized_parameters": {},
            "recommendations": [],
            "analysis_data": {}
        }
        
        if analysis_type in ["pid", "both"]:
            print("Analyzing PID performance...")
            pid_analysis = self.pid_analyzer.analyze(flight_data)
            results["analysis_data"]["pid"] = pid_analysis
            
            pid_optimized = self.optimizer.optimize_pid(flight_data, pid_analysis)
            results["optimized_parameters"].update(pid_optimized)
        
        if analysis_type in ["filter", "both"]:
            print("Analyzing filter performance...")
            filter_analysis = self.filter_analyzer.analyze(flight_data)
            results["analysis_data"]["filter"] = filter_analysis
            
            filter_optimized = self.optimizer.optimize_filters(flight_data, filter_analysis)
            results["optimized_parameters"].update(filter_optimized)
        
        print("Generating recommendations...")
        results["recommendations"] = self.recommender.generate_recommendations(
            results["analysis_data"]
        )
        
        # Generate performance graphs
        if analysis_type in ["pid", "both"]:
            print("Generating performance graphs...")
            graph_files = self.pid_analyzer.generate_performance_graphs(
                results["analysis_data"]["pid"], "output"
            )
            results["generated_graphs"] = graph_files
        
        return results
    
    def generate_output_files(self, results, output_dir="output"):
        """Generate parameter files and reports"""
        return self.output_generator.generate_all(results, output_dir)