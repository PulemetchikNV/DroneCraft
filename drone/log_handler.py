import logging
import socket

class UDPLogHandler(logging.Handler):
    """Custom logging handler that sends logs to UDP server"""
    def __init__(self, server_ip='192.168.2.124', server_port=9999, drone_name='unknown_drone'):
        super().__init__()
        self.server_ip = server_ip
        self.server_port = server_port
        self.drone_name = drone_name
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
    def emit(self, record):
        try:
            # Map Python logging levels to our custom log types
            level_mapping = {
                'DEBUG': 'info',
                'INFO': 'drone',
                'WARNING': 'warning',
                'ERROR': 'error',
                'CRITICAL': 'critical'
            }
            log_type = level_mapping.get(record.levelname, 'info')
            
            # Format message: drone_id|log_type|message
            message = f"{self.drone_name}|{log_type}|{record.getMessage()}"
            
            # Send to UDP server
            self.sock.sendto(message.encode('utf-8'), (self.server_ip, self.server_port))
        except Exception:
            # Don't raise exceptions in logging handler
            pass
    
    def close(self):
        if hasattr(self, 'sock'):
            self.sock.close()
        super().close()

