import cv2
import numpy as np

filename = '/home/summerschool/kiwilogs/Range/opendlv.proxy.ImageReading-0/1534837512972079.png' # 1
# filename = '/home/summerschool/kiwilogs/Range/opendlv.proxy.ImageReading-0/1534837506074929.png' # 2
img = cv2.imread(filename, cv2.IMREAD_COLOR)

# Convert BGR to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# define range of green color in HSV
lower_green = np.array([34,35,160])
upper_green = np.array([60,255,255])
# Threshold the HSV image to get only green colors
mask = cv2.inRange(hsv, lower_green, upper_green)
# Bitwise-AND mask and original image
res = cv2.bitwise_and(img,img, mask= mask)
cv2.imshow('frame',img)
cv2.imshow('mask',mask)
cv2.imshow('res',res)

kernel = np.ones((5,5),np.uint8)
erosion = cv2.erode(mask,kernel,iterations = 2)
dilation = cv2.dilate(erosion,kernel,iterations = 2)

cv2.imshow('erosion',erosion)
cv2.imshow('dilation',dilation)

cv2.waitKey()

cv2.destroyAllWindows()
