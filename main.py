#!/usr/bin/env python3
"""
DroneBrain - Autonomous Drone Vision & Control System

Main entry point for the DroneBrain system.

Usage:
    python main.py [--config path/to/config.yaml] [--log-level DEBUG]

Author: DroneBrain Team
"""

import argparse
import logging
import sys
from pathlib import Path

from src.core.pipeline_manager import PipelineManager

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))


def setup_logging(log_level: str = "INFO"):
    """
    Configure logging system.

    Args:
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR)
    """
    # Create logs directory if it doesn't exist
    log_dir = Path("logs")
    log_dir.mkdir(exist_ok=True)

    # Configure logging format
    log_format = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"

    # Configure root logger
    logging.basicConfig(
        level=getattr(logging, log_level.upper()),
        format=log_format,
        handlers=[
            # Console handler
            logging.StreamHandler(sys.stdout),
            # File handler with rotation
            logging.FileHandler(
                log_dir / "dronebrain.log",
                mode="a",
            ),
        ],
    )

    # Reduce verbosity of some noisy libraries
    logging.getLogger("urllib3").setLevel(logging.WARNING)
    logging.getLogger("PIL").setLevel(logging.WARNING)


def print_banner():
    """Print DroneBrain banner."""
    banner = """
    ╔═══════════════════════════════════════════════════════════╗
    ║                                                           ║
    ║                      DRONEBRAIN                           ║
    ║          Autonomous Drone Vision & Control System         ║
    ║                                                           ║
    ║                    Version 0.1.0                          ║
    ║                                                           ║
    ╚═══════════════════════════════════════════════════════════╝
    """
    print(banner)


def main():
    """Main entry point."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="DroneBrain - Autonomous Drone Vision & Control System"
    )

    parser.add_argument(
        "--config",
        type=str,
        default="config/config.yaml",
        help="Path to configuration file (default: config/config.yaml)",
    )

    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level (default: INFO)",
    )

    parser.add_argument(
        "--dev",
        action="store_true",
        help="Development mode (enables DEBUG logging)",
    )

    args = parser.parse_args()

    # Set log level
    log_level = "DEBUG" if args.dev else args.log_level

    # Setup logging
    setup_logging(log_level)

    # Print banner
    print_banner()

    logger = logging.getLogger(__name__)
    logger.info("Starting DroneBrain system...")

    try:
        # Create pipeline manager
        manager = PipelineManager(config_path=args.config)

        # Initialize all components
        if not manager.initialize():
            logger.error("System initialization failed")
            sys.exit(1)

        # Start all components
        manager.start()

        # Run main loop
        manager.run()

    except Exception as e:
        logger.exception(f"Fatal error: {e}")
        sys.exit(1)

    logger.info("DroneBrain shutdown complete")
    sys.exit(0)


if __name__ == "__main__":
    main()
