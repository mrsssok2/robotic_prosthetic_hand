# Per-finger proportional curl + gesture. Finger order = servo channels 0-4.
# Thumb value = how far the thumb has swept sideways across the palm (0 = out to the side, 1 = across the palm).
std_msgs/Header header
bool valid                   # false when tracking was lost (values hold the last good state)
string handedness
string gesture               # Fist | Pointing | Peace | Open Hand | Call Me | Pinch | None
float32[5] curl              # little, index, middle, ring, thumb; 0 = straight, 1 = fully closed
