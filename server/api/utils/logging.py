import logging


def configure_logging():
    logging.basicConfig(level=logging.INFO)  # Set the global logging level
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)  # Set the specific logger level

    # Optional: Add a console handler if needed
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(
        logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    )
    logger.addHandler(console_handler)
