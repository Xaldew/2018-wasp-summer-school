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
FONT = cv2.FONT_HERSHEY_SIMPLEX

pts = deque(maxlen=args["buffer"])
vs = cv2.VideoCapture(0)

# allow the camera or video file to warm up
time.sleep(1.0)

while True:
    ret, frame = vs.read()

    if frame is None:
	break
 
    # Resize the frame, blur it, and convert it to the HSV color space.
    scale = 0.8
    ih,iw,_ = frame.shape
    width = int(iw * scale)
    height = int(ih * scale)
    frame = cv2.resize(frame, (iw, ih), interpolation = cv2.INTER_AREA)

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
    center = None
    seen = False
    bw,bh = (0,0)

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
        bw,bh = rect[1]
        area = bw * bh

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

    # Compute thrust and steering angles for the latest point.
    angle  = 0.0
    thrust = 0.0
    if pts:
        cx, cy = pts[0]
        a = iw/2 - cx
        b = ih/2
        angle = math.atan(a / float(b))
        thrust = 1.0 - (float(bh) / b)

        s = min(max(0.0, thrust), 1.0)
        nh = ih/2 + (b - b*s)
        viz = np.array([[iw/2, ih], [cx, nh], [iw/2, nh]], np.int32)
        cv2.polylines(frame, [viz], True, (255,255,255), thickness=3)

    # Visualize thrust and steering angle.
    rstr = "Angle: %.2f" % math.degrees(angle)
    cv2.putText(frame, rstr, (10,ih/2), FONT, 0.6, (255,255,255), 1, cv2.LINE_AA)
    rstr = "Thrust: %.2f" % thrust
    cv2.putText(frame, rstr, (10,ih/2 + 20), FONT, 0.6, (255,255,255), 1, cv2.LINE_AA)

    # Display the frames and tracking data.
    cv2.imshow("Frame", frame)
    # cv2.imshow("Mask", mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
	break

# Free all resources.
vs.release()
cv2.destroyAllWindows()
