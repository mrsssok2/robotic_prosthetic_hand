activate the virtual environment 

```bash
cd ~
source .venv/bin/activate
```

install mediapipe

```bash
pip install mediapipe
```

install additional  libraries for Reading webcam input and detecting hand gestures with opencv and moving servos via PCA9685 based on those gestures

```bash
pip install opencv-python numpy adafruit-circuitpython-pca9685 adafruit-circuitpython-servokit adafruit-blinka
```

run the python programme 


```bash
python gesture_control.py
```



