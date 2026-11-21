# Per-finger proportional curl + gesture. Finger order = servo channels 0-4.
# Thumb value = how far the thumb has swept sideways across the palm (0 = out to the side, 1 = across the palm).
std_msgs/Header header
bool valid                   # false when tracking was lost (values hold the last good state)
string handedness
string gesture               # Fist | Pointing | Peace | Open Hand | Call Me | Pinch | None
float32[5] curl              # little, index, middle, ring, thumb; 0 = straight, 1 = fully closed
float32[2] wrist           # wrist pronation (+) / supination (-) and flexion (+) / extension (-), radians; 0 0 = neutral or not measured
bool reach_valid           # reach control is on in the camera script (it sends a hand position)
float32[3] reach            # grasp world: [x right+, y up+, closeness+] in -1..1, relative to the re-centred pose; see hand_tracker/core/reach.py
