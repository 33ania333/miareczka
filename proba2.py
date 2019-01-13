import numpy as np
import cv2 as cv
import math
import przetwarzanie
from matplotlib import pyplot as plt

img = cv.imread("IMG_20181203_231136294.jpg", 3)
# cv.imshow('image', img)
frame_threshold = cv.inRange(img, (0, 100, 100), (80, 225, 225))
# cv.imshow("treszhold", frame_threshold)

img_lines = frame_threshold.copy()
krawedzie = cv.Canny(img_lines, 50, 150, apertureSize=3)
cv.imshow("krawedzie", krawedzie)

lines = cv.HoughLines(krawedzie, 1, np.pi / 180, 50, None, 0, 0)

y_poziome = []
if lines is not None:
    for i in range(0, len(lines)):
        rho = lines[i][0][0]
        theta = lines[i][0][1]
        a = math.cos(theta)
        b = math.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * a))
        pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * a))
        if (np.pi/2 - 0.1) < theta < (np.pi/2 + 0.1):
            cv.line(img, pt1, pt2, (0, 0, 255), 3, cv.LINE_AA)
            y_poziome.append(y0)

y_poziome.sort()
deltay = []
for i in range(0, len(y_poziome)-1):
    deltay.append(math.fabs(y_poziome[i]-y_poziome[i+1]))

piksel_na_cm = np.median(deltay)
piksel_na_mm = piksel_na_cm/10
mm_na_piksel = 1/piksel_na_mm

print(mm_na_piksel)


cv.imshow("linie", img)

k = cv.waitKey(0)

