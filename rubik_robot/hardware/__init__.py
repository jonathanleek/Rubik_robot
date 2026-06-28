"""Hardware drivers (PCA9685 servos, Pi camera) and their mock equivalents.

Importing this package does *not* import any Pi-only libraries; the real
drivers import ``board`` / ``picamera2`` lazily inside their constructors so the
rest of the application (and the mock drivers) work on any machine.
"""
