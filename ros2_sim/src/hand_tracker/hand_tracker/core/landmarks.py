"""MediaPipe Hands landmark indices and finger definitions."""

NUM_LANDMARKS = 21

# Output order == servo channels 0-4 of the original project.
FINGER_ORDER = ("little", "index", "middle", "ring", "thumb")

# Landmark chains whose *interior* points are the measured flexion joints:
#   wrist, MCP, PIP, DIP, TIP  -> flexion at MCP, PIP, DIP (3 angles)
FINGER_CHAINS = {
    "little": (0, 17, 18, 19, 20),
    "index": (0, 5, 6, 7, 8),
    "middle": (0, 9, 10, 11, 12),
    "ring": (0, 13, 14, 15, 16),
}

THUMB_TIP, INDEX_MCP, LITTLE_MCP = 4, 5, 17
