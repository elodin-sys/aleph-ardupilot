#!/usr/bin/env python3
"""
Hello Service - Example systemd service for Aleph

This is a minimal example of a Python daemon that runs as a systemd service.
It demonstrates:
  - Proper daemon structure for systemd
  - Logging to journald (via stdout/stderr)
  - Graceful shutdown handling
  - Configuration via environment variables

When deployed to Aleph, you can:
  - View logs: journalctl -u hello-service -f
  - Check status: systemctl status hello-service
  - Restart: systemctl restart hello-service
"""

import os
import signal
import sys
import time
from datetime import datetime

# Configuration via environment variables (set in systemd unit or module)
MESSAGE = os.environ.get("HELLO_MESSAGE", "Hello from Aleph!")
INTERVAL = int(os.environ.get("HELLO_INTERVAL", "10"))

# Global flag for graceful shutdown
running = True


def signal_handler(signum, frame):
    """Handle shutdown signals gracefully."""
    global running
    print(f"Received signal {signum}, shutting down...")
    running = False


def main():
    """Main service loop."""
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    print(f"Hello Service starting...")
    print(f"  Message: {MESSAGE}")
    print(f"  Interval: {INTERVAL} seconds")
    sys.stdout.flush()

    iteration = 0
    while running:
        iteration += 1
        timestamp = datetime.now().isoformat()
        print(f"[{timestamp}] #{iteration}: {MESSAGE}")
        sys.stdout.flush()

        # Sleep in small increments to respond to signals quickly
        for _ in range(INTERVAL):
            if not running:
                break
            time.sleep(1)

    print("Hello Service stopped.")
    sys.stdout.flush()


if __name__ == "__main__":
    main()

