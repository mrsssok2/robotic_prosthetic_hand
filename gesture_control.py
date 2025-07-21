import cv2
import numpy as np
import mediapipe as mp
import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Initialize MediaPipe
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)
mp_draw = mp.solutions.drawing_utils

# Finger mapping and landmark IDs
# UPDATED: Swapped Index (1) and Ring (3) as requested
finger_map = {
    0: "Little",
    1: "Index",  # Changed from "Ring" to "Index"
    2: "Middle",
    3: "Ring",   # Changed from "Index" to "Ring"
    4: "Thumb"
}

# Fingertip IDs and pip (proximal interphalangeal) joint IDs
fingertip_ids = [20, 16, 12, 8, 4]
pip_joint_ids = [18, 14, 10, 6, 2]

# Setup I2C bus for servo control
i2c = busio.I2C(board.SCL, board.SDA)
# Setup PCA9685
pca = PCA9685(i2c)
pca.frequency = 50  # 50Hz for servos

# Create Servo instances - mapping to fingers
# UPDATED: Servo channel to finger mapping according to the updated specifications
# Channel 0 = Little, Channel 1 = Index, Channel 2 = Middle, Channel 3 = Ring, Channel 4 = Thumb
servos = {
    "Little": servo.Servo(pca.channels[0]),
    "Index": servo.Servo(pca.channels[1]),
    "Middle": servo.Servo(pca.channels[2]),
    "Ring": servo.Servo(pca.channels[3]),
    "Thumb": servo.Servo(pca.channels[4]),
}

# Set initial positions - all fingers up (extended) except thumb
# For all fingers except thumb: 0 degrees = up, 180 degrees = down (folded)
# For thumb: 180 degrees = up, 0 degrees = down (folded)
for finger, servo_obj in servos.items():
    if finger == "Thumb":
        servo_obj.angle = 180  # Thumb up
    else:
        servo_obj.angle = 0    # Other fingers up

# Camera setup
width, height = 640, 480
cap = cv2.VideoCapture(0)
cap.set(3, width)
cap.set(4, height)

if not cap.isOpened():
    print("Error: Could not open camera")
    pca.deinit()
    exit()

# FPS calculation variables
prev_time = 0
current_time = 0

# Smoothing variables to prevent servo jitter
last_servo_positions = {finger: servos[finger].angle for finger in servos}
smoothing_factor = 0.3  # Lower = smoother but slower response

print("Hand gesture recognition with robotic control started. Press 'q' to quit.")
print("Servo mapping: Channel 0=Little, Channel 1=Index, Channel 2=Middle, Channel 3=Ring, Channel 4=Thumb")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
            
        # Flip the frame for a selfie-view
        frame = cv2.flip(frame, 1)
        
        # Convert BGR to RGB
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process the image
        result = hands.process(img_rgb)
        
        # Calculate FPS
        current_time = time.time()
        fps = 1 / (current_time - prev_time) if current_time != prev_time else 0
        prev_time = current_time
        
        # Display FPS
        cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Status text
        finger_status = {name: "Closed" for name in finger_map.values()}
        
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                # Draw landmarks
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                # Extract landmark positions
                lm_list = []
                for id, lm in enumerate(hand_landmarks.landmark):
                    h, w, _ = frame.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    lm_list.append((cx, cy))
                
                # Calculate angles for each finger
                angles = []
                for i in range(5):
                    if len(lm_list) > max(fingertip_ids[i], pip_joint_ids[i]):
                        tip_y = lm_list[fingertip_ids[i]][1]
                        pip_y = lm_list[pip_joint_ids[i]][1]
                        
                        # Special case for thumb which moves horizontally
                        if i == 4:  # Thumb
                            tip_x = lm_list[fingertip_ids[i]][0]
                            pip_x = lm_list[pip_joint_ids[i]][0]
                            angle = 0 if tip_x < pip_x else 180
                        else:  # Other fingers
                            angle = 0 if tip_y < pip_y else 180
                        
                        angles.append(angle)
                        
                        # Update finger status
                        finger_name = finger_map[i]
                        if angle > 90:  # If angle > 90, finger is closed
                            finger_status[finger_name] = "Closed"
                        else:  # If angle < 90, finger is open
                            finger_status[finger_name] = "Open"
                        
                        # Control corresponding servo with smoothing
                        target_angle = 0  # Default position
                        
                        if finger_name == "Thumb":
                            # For thumb: 180 = up, 0 = down
                            target_angle = 180 if finger_status[finger_name] == "Open" else 0
                        else:
                            # For other fingers: 0 = up, 180 = down
                            target_angle = 0 if finger_status[finger_name] == "Open" else 180
                        
                        # Apply smoothing to prevent jitter
                        current_angle = last_servo_positions[finger_name]
                        new_angle = current_angle + smoothing_factor * (target_angle - current_angle)
                        last_servo_positions[finger_name] = new_angle
                        
                        # Set servo position
                        try:
                            servos[finger_name].angle = new_angle
                        except Exception as e:
                            print(f"Error setting servo angle for {finger_name}: {e}")
                            
                # Display finger angles
                y_pos = 70
                for i, (finger, status) in enumerate(finger_status.items()):
                    if i < len(angles):
                        text = f"{finger}: {status} ({angles[i]}°)"
                        cv2.putText(frame, text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        y_pos += 30
                        
                # Add gesture recognition
                # Count number of open fingers
                open_fingers = sum(1 for status in finger_status.values() if status == "Open")
                
                # Basic gesture recognition
                gesture = "Unknown"
                if open_fingers == 0:
                    gesture = "Fist"
                elif open_fingers == 1 and finger_status["Index"] == "Open":
                    gesture = "Pointing"
                elif open_fingers == 2 and finger_status["Index"] == "Open" and finger_status["Middle"] == "Open":
                    gesture = "Peace"
                elif open_fingers == 5:
                    gesture = "Open Hand"
                elif finger_status["Thumb"] == "Open" and finger_status["Little"] == "Open" and finger_status["Index"] == "Closed" and finger_status["Middle"] == "Closed" and finger_status["Ring"] == "Closed":
                    gesture = "Call Me"
                
                # Display the recognized gesture
                cv2.putText(frame, f"Gesture: {gesture}", (10, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        else:
            # When no hand detected, display servo positions
            y_pos = 70
            for finger, servo_obj in servos.items():
                current_pos = last_servo_positions[finger]
                text = f"{finger} Servo: {int(current_pos)}°"
                cv2.putText(frame, text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                y_pos += 30
                
        # Display the image
        cv2.imshow("Hand Gesture Recognition with Robotic Control", frame)
        
        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Program interrupted")
except Exception as e:
    print(f"Error: {e}")
finally:
    # Clean up
    print("Cleaning up...")
    # Reset all servos to initial position before exit
    for finger, servo_obj in servos.items():
        if finger == "Thumb":
            servo_obj.angle = 180  # Thumb up
        else:
            servo_obj.angle = 0    # Other fingers up
    time.sleep(1)
    
    # Deinitialize
    cap.release()
    cv2.destroyAllWindows()
    pca.deinit()
    print("Program ended")