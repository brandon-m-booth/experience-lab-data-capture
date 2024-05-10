#!/usr/bin/env python3
import cv2
from PIL import ImageGrab
import numpy as np 
from screeninfo import get_monitors

# Get user input for the filename
filename = input("Enter the filename for the video (e.g., recorded_video1.mp4): ")

for m in get_monitors():
    x: int = m.x
    y: int = m.y 
    width: int = m.width
    height: int = m.height

fourcc = cv2.VideoWriter_fourcc("m","p","4","v")

# Create the VideoWriter in the current working directory with the user-defined filename
captured_video = cv2.VideoWriter(filename, fourcc, 3.0, (width, height))

while True:
    img = ImageGrab.grab(bbox=(x, y, width, height))

    np_img = np.array(img)

    cvt_img = cv2.cvtColor(np_img, cv2.COLOR_BGR2RGB)

    cv2.imshow("Video Capture", cvt_img)

    captured_video.write(cvt_img)

    key = cv2.waitKey(20)
    if key == 27:
        break

cv2.destroyAllWindows()
