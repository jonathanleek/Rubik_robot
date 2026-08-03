#!/usr/bin/env python3
"""
Entry point for the Rubik Robot API server.

Usage:
    python -m rubik_robot.run [--driver gpio|pca9685] [--host 0.0.0.0] [--port 5000]

Examples:
    # Start with PCA9685 driver (default), accessible on local network
    python -m rubik_robot.run

    # Start with GPIO driver on a specific port
    python -m rubik_robot.run --driver gpio --port 8080

    # Start accessible only from localhost
    python -m rubik_robot.run --host 127.0.0.1
"""

import argparse
from rubik_robot.app import create_app


def main():
    parser = argparse.ArgumentParser(
        description="Rubik Robot API Server",
        epilog="See README.md for full API documentation.",
    )
    parser.add_argument(
        "--driver",
        choices=["gpio", "pca9685"],
        default="pca9685",
        help="Hardware driver to use (default: pca9685)",
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Host to bind to (default: 0.0.0.0, all interfaces)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5000,
        help="Port to listen on (default: 5000)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable Flask debug mode (not for production use)",
    )

    args = parser.parse_args()

    print(f"Starting Rubik Robot API server...")
    print(f"  Driver: {args.driver}")
    print(f"  Host:   {args.host}")
    print(f"  Port:   {args.port}")
    print()

    app = create_app(driver_type=args.driver)
    app.run(host=args.host, port=args.port, debug=args.debug, threaded=True)


if __name__ == "__main__":
    main()
