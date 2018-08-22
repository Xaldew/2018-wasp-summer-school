import argparse
import cv2


class ShapeDetector:
    def __init__(self):
	pass

    def detect(self, c):
	shape = "unidentified"
	peri = cv2.arcLength(c, True)
	approx = cv2.approxPolyDP(c, 0.04 * peri, True)

	if len(approx) == 3:
	    shape = "triangle"
	elif len(approx) == 4:
	    (x, y, w, h) = cv2.boundingRect(approx)
	    ar = w / float(h)
	    shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
	elif len(approx) == 5:
	    shape = "pentagon"
	else:
	    shape = "circle"

	return shape

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="path to the input image")
args = vars(ap.parse_args())

# load the image and resize it to a smaller factor so that
# the shapes can be approximated better
image = cv2.imread(args["image"])
ratio = image.shape[0] / float(image.shape[0])
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

# find contours in the thresholded image and initialize the
# shape detector
im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
sd = ShapeDetector()

# loop over the contours
for c in contours:
    # compute the center of the contour, then detect the name of the
    # shape using only the contour
    M = cv2.moments(c)
    cX = int((M["m10"] / M["m00"]) * ratio)
    cY = int((M["m01"] / M["m00"]) * ratio)
    shape = sd.detect(c)

    # multiply the contour (x, y)-coordinates by the resize ratio,
    # then draw the contours and the name of the shape on the image
    c = c.astype("float")
    c *= ratio
    c = c.astype("int")
    cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
    cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
		0.5, (255, 255, 255), 2)

    # show the output image
    cv2.imshow("Image", image)
    cv2.waitKey(0)
