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

        host = os.getenv("APP_HOST", "0.0.0.0")
        port = int(os.getenv("APP_PORT", "5001"))

        logger.info(f"Running on {host}:{port}")

        app.run(host="0.0.0.0", port=5001)
    except KeyboardInterrupt:
        logger.info("Application interrupted by user.")
    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
        sys.exit(1)
    finally:
        logger.info("Shutting down rclpy...")
        rclpy.shutdown()