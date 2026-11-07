"""Shared resource and numeric transport limits for the reference pipeline.

The tested camera fixture is 640x400. These ceilings also allow 640x480; changing
them requires profiling the full chain and reviewing queue memory on the Jetson.
They describe supported software inputs, not measured hardware capacity.
"""

MAX_IMAGE_PIXELS = 640 * 480
# Per raw image, including row padding. This bounds supported callback work,
# not buffers already allocated by DDS, synchronization queues or native code.
MAX_IMAGE_PAYLOAD_BYTES = MAX_IMAGE_PIXELS * 8
MAX_OBSTACLE_POINTS = 20_000
# Packed XYZ float64 is the largest supported PointCloud2 representation.
# This bounds admitted payload/decoding work, not prior DDS allocations.
MAX_CLOUD_PAYLOAD_BYTES = MAX_OBSTACLE_POINTS * 3 * 8

# Numeric position representation only, not a physical localization bound.
# Ingress checks horizontal cell-radius and vertical half-width independently;
# external-vision egress uses this as its default position-rounding allowance.
POSITION_NUMERIC_ALLOWANCE_M = 1e-6

# Per independent source stream. Synchronized histories hold at most twice
# this many poses. Callers may configure a smaller cap, never a larger one.
MAX_POSE_HISTORY_SAMPLES = 2_048

# Published reference: 244x244 cells, 13 goals, 23 prepared waypoints and a
# 1.65 MB JSON file. These bound Python object counts/work on the nominal Orin
# Nano 8 GB target; they are not measured free-RAM or callback-time guarantees.
MAX_GRID_AXIS = 512
MAX_GRID_CELLS = 262_144
MAX_CAPSULE_CELLS = 16_384
MAX_SEARCH_STATES = 65_536
MAX_SEARCH_QUEUE = 65_536
MAX_PLANNING_WORK = 2_000_000
MAX_ROUTE_GOALS = 64
MAX_PREPARED_WAYPOINTS = 4_096
MAX_COURSE_BYTES = 8 * 1024 * 1024
