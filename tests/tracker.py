# import the necessary packages
from collections import deque
import numpy as np
import argparse
import cv2
import math
import time
 
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=32, help="max buffer size")
args = vars(ap.parse_args())

greenLower = (34,  40,  40)
greenUpper = (60, 255, 255)

pts = deque(maxlen=args["buffer"])
vs = cv2.VideoCapture(0)

# allow the camera or video file to warm up
time.sleep(1.0)

while True:
    ret, frame = vs.read()

    if frame is None:
	break
 
    # Resize the frame, blur it, and convert it to the HSV color space
    scale = 0.8
    h,w,_ = frame.shape
    width = int(w * scale)
    height = int(h * scale)
    frame = cv2.resize(frame, (w, h), interpolation = cv2.INTER_AREA)

    work = frame
    work = cv2.GaussianBlur(work, (11, 11), 0)
    hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)
 
    # Filter based on our color and remove noise.
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours in the mask.
    im2, cnts, hierarchy = cv2.findContours(mask.copy(),
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
    angle = math.pi / 2
    center = None
    seen = False

    # Only proceed if at least one contour was found.
    if len(cnts) > 0:

	# find the largest contour in the mask, then use it to compute the
	# minimum enclosing circle and centroid
        c = max(cnts, key=cv2.contourArea)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        w,h = rect[1]
        area = w * h

        if area > 1000:
            cv2.drawContours(frame, [box], 0, (0,0,255), 2)
            pts.appendleft(center)
            seen = True

    # Loop over the set of tracked points
    for i in range(1, len(pts)):
        if pts[i - 1] is None or pts[i] is None:
            continue

        thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

    if not seen and pts:
        pts.pop()

    # Display the frames and tracking data.
    cv2.imshow("Frame", frame)
    # cv2.imshow("Mask", mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
	break

# Free all resources.
vs.release()
cv2.destroyAllWindows()
