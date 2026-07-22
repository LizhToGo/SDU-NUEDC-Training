"""Thin production entry point for the NUEDC 2025 C application."""

# Importing the application starts its camera/UART event loop.  Keeping this
# file tiny avoids maintaining a second copy of the measurement program.
import single_shot_measurement  # noqa: F401

