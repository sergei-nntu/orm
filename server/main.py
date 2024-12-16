import os
import sys
import rclpy

from rclpy.logging import get_logger
from api import create_app


if __name__ == '__main__':
    rclpy.init()

    logger = get_logger(__name__)
    logger.info("Starting the application...")

    try:
        app = create_app()

        host = app.config.get('HOST', '0.0.0.0')
        port = app.config.get('PORT', 5000)

        logger.info(f"Running on {host}:{port}")

        app.run(host=host, port=port)
    except KeyboardInterrupt:
        logger.info("Application interrupted by user.")
    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
        sys.exit(1)
    finally:
        logger.info("Shutting down rclpy...")
        rclpy.shutdown()