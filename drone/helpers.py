import logging

try:
    from .log_handler import UDPLogHandler
except ImportError:
    from log_handler import UDPLogHandler


def setup_logging(drone_name, log_level=logging.INFO):
    logger = logging.getLogger(drone_name)
    logger.setLevel(log_level)
    
    # Console handler (idempotent)
    if not any(isinstance(h, logging.StreamHandler) for h in logger.handlers):
        console_handler = logging.StreamHandler()
        console_formatter = logging.Formatter(
            f'[{drone_name}] %(asctime)s - %(levelname)s - %(message)s', '%H:%M:%S'
        )
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)
    
    # UDP handler (idempotent)
    if not any(isinstance(h, UDPLogHandler) for h in logger.handlers):
        udp_handler = UDPLogHandler(drone_name=drone_name)
        logger.addHandler(udp_handler)
    
    return logger