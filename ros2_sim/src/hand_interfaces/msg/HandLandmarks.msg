# One tracked hand: the 21 MediaPipe landmarks (prefer world landmarks, metres).
std_msgs/Header header
string handedness            # "Left" / "Right" as reported by MediaPipe
float32 score                # handedness confidence 0..1
geometry_msgs/Point[21] landmarks
