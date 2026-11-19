"""Single source of truth for the hand's dimensions (metres). Used by gen_meshes.py (visual meshes)
and the geometry test; the xacro repeats the joint positions (test_geometry.py checks they agree)."""

PALM_H = 0.085                                   # wrist (palm frame origin) -> knuckle line
PALM = dict(half=(0.045, 0.0155, 0.0425), center=(0.0, 0.0, 0.0425), round=0.012)
# thenar pad the thumb is mounted on (visual bulge on the palm's lower thumb side)
PAD = dict(center=(0.036, -0.020, 0.030), radii=(0.018, 0.012, 0.026))

GAP = 0.0012                                     # half the visible gap at every finger joint

# x of the MCP, (proximal, middle, distal) lengths, base radius. Index is next to the thumb (+X).
FINGERS = {
    "little": dict(x=-0.034, lengths=(0.034, 0.021, 0.018), r=0.0075),
    "ring":   dict(x=-0.0115, lengths=(0.042, 0.027, 0.020), r=0.0085),
    "middle": dict(x=0.0115, lengths=(0.046, 0.029, 0.022), r=0.0090),
    "index":  dict(x=0.034, lengths=(0.041, 0.026, 0.020), r=0.0085),
}
# per-segment (base radius, tip radius) as a fraction of the finger radius: slight taper
FINGER_TAPER = ((1.0, 0.88), (0.84, 0.74), (0.72, 0.60))

THUMB = dict(
    mount=(0.046, -0.037, 0.027),                # CMC: low on the palm's side, on the thenar pad
    tilt=1.2217,                                 # open pose: 70 deg from the index finger, pointing sideways
    lengths=(0.034, 0.030, 0.024),               # metacarpal, proximal, distal
    radii=((0.0105, 0.0098), (0.0095, 0.0088), (0.0085, 0.0068)),
)

FOREARM = dict(length=0.272, wrist_ab=(0.026, 0.022), bottom_ab=(0.043, 0.038),
               cuff=dict(z=(-0.032, 0.003), ab=(0.0295, 0.0255)))
BASE_Z = 0.28                                    # wrist height above the stand plate's underside
PLATE = dict(size=(0.20, 0.11, 0.008), center=(0.03, 0.0))   # black acrylic stand plate (x, y centre)
BOX = dict(size=(0.075, 0.05, 0.042), center=(0.085, 0.0))   # "electronics" block on the plate


def segment_radii(finger):
    r = FINGERS[finger]["r"]
    return [(r * a, r * b) for a, b in FINGER_TAPER]
