"""Rubik's cube robot — Raspberry Pi hardware API server.

The Pi exposes its hardware (servos + camera) over an HTTP API. All solving and
orchestration lives in the client (e.g. an Apache Airflow DAG in a separate
project). Importing this package pulls in no hardware libraries.
"""

__version__ = "0.1.0"
