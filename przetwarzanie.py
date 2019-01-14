import numpy as np
import cv2 as cv
import math
import imutils
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours


def midpoint(pta, ptb):
    return (pta[0] + ptb[0]) * 0.5, (pta[1] + ptb[1]) * 0.5


def takeSecond(elem):
    return elem[1]


def getNiceLimits(img):
    hsv1 = img[int(img.shape[0] / 2)][0]
    hsv2 = img[int(img.shape[0] / 2)][img.shape[1] - 1]
    hsv3 = img[0][int(img.shape[1] / 2)]
    hsv4 = img[0][int(img.shape[1] / 2)]
    margin = 80
    bottom = (np.int(max(min(hsv1[0], hsv2[0], hsv3[0], hsv4[0]) - margin, 0)),
              np.int(max(min(hsv1[1], hsv2[1], hsv3[1], hsv4[1]) - margin, 0)),
              np.int(max(min(hsv1[2], hsv2[2], hsv3[2], hsv4[2]) - margin, 0)))
    top = (np.int(min(max(hsv1[0], hsv2[0], hsv3[0], hsv4[0]) + margin, 255)),
           np.int(min(max(hsv1[1], hsv2[1], hsv3[1], hsv4[1]) + margin, 255)),
           np.int(min(max(hsv1[2], hsv2[2], hsv3[2], hsv4[2]) + margin, 255)))

    return bottom, top


class Measure:
    def __init__(self, image, tittle):
        self.img = image
        self.index = 0
        self.vertical = 0
        self.horizontal = 0
        self.tape = np.zeros(self.img.shape, dtype=np.int8)
        self.object = np.zeros(self.img.shape, dtype=np.int8)
        self.framed = np.zeros(self.img.shape, dtype=np.int8)
        self.name = tittle

    def scale(self):
        hsv = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
        frame_threshold = cv.inRange(hsv, (20, 120, 120), (30, 255, 255))
        # cv.imshow("treszhold", frame_threshold)

        kernel = cv.getStructuringElement(	cv.MORPH_RECT, (5, 5))
        closing = cv.morphologyEx(cv.bitwise_not(frame_threshold), cv.MORPH_OPEN, kernel)
        kernel = cv.getStructuringElement(	cv.MORPH_RECT, (20, 20))
        closing = cv.morphologyEx(closing, cv.MORPH_OPEN, kernel)

        self.tape = closing
        # cv.imshow("miarka", self.tape)

        img_lines = self.img.copy()
        krawedzie = cv.Canny(frame_threshold, 50, 150, apertureSize=3)
        # cv.imshow("kraw", krawedzie)

        lines_ver = cv.HoughLines(krawedzie, 1, np.pi / 360, 100, None, 0, 0)
        tape_orientation_ver = lines_ver[0][0][1]
        tape_orientation_hor = (tape_orientation_ver + np.pi/2) % np.pi

        xy_horizontal = []

        lines_hor = cv.HoughLines(krawedzie, 1, np.pi / 360, 20, None, 0, 0)
        if lines_hor is not None:
            for i in range(0, len(lines_hor)):
                rho = lines_hor[i][0][0]
                theta = lines_hor[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * a))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * a))
                if (tape_orientation_hor - 0.02) < theta < (tape_orientation_hor + 0.02):
                    cv.line(img_lines, pt1, pt2, (0, 0, 255), 3, cv.LINE_AA)
                    xy_horizontal.append((x0, y0))

        xy_horizontal.sort(key=takeSecond)
        xy_horizontal_uniques = []
        xy_horizontal_uniques.append(xy_horizontal[0])
        for i in range(1, len(xy_horizontal) - 1):
            distance = dist.euclidean((xy_horizontal[i-1][0], xy_horizontal[i-1][1]),
                                      (xy_horizontal[i][0], xy_horizontal[i][1]))
            if math.fabs(distance) > 12:
                xy_horizontal_uniques.append(xy_horizontal[i])

        delta_y = []
        for i in range(0, len(xy_horizontal_uniques) - 1):
            distance = dist.euclidean((xy_horizontal_uniques[i][0], xy_horizontal_uniques[i][1]),
                                      (xy_horizontal_uniques[i+1][0], xy_horizontal_uniques[i+1][1]))
            delta_y.append(math.fabs(distance))

        pixel_per_cm = np.median(delta_y)
        pixel_per_mm = pixel_per_cm / 10
        mm_per_pixel = 1 / pixel_per_mm

        self.index = mm_per_pixel
        # cv.imshow("linie", img_lines)

    def extract_object(self):

        hsv = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
        bottom, top = getNiceLimits(hsv)
        background_threshold = cv.inRange(hsv, bottom, top)
        object_threshold = cv.bitwise_not(background_threshold)
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (20, 20))
        object_threshold = cv.morphologyEx(object_threshold, cv.MORPH_CLOSE, kernel)
        # cv.imshow("obiekt", object_threshold)
        self.object = cv.bitwise_and(object_threshold, self.tape)
        # cv.imshow("obiekt", self.object)

    def measure_object(self):
        object_contours = cv.findContours(self.object, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        object_contours = imutils.grab_contours(object_contours)
        (object_contours, _) = contours.sort_contours(object_contours)

        for c in object_contours:
            if cv.contourArea(c) < 20000:
                continue

            orig = self.img.copy()
            box = cv.minAreaRect(c)
            box = cv.boxPoints(box)
            box = np.array(box, dtype="int")

            box = perspective.order_points(box)
            cv.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)

            (tl, tr, br, bl) = box
            (tltrX, tltrY) = midpoint(tl, tr)
            (blbrX, blbrY) = midpoint(bl, br)
            (tlblX, tlblY) = midpoint(tl, bl)
            (trbrX, trbrY) = midpoint(tr, br)

            cv.line(orig, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)), (255, 0, 255), 2)
            cv.line(orig, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)), (255, 0, 255), 2)

            d_vertical = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
            d_horizontal = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
            self.vertical = d_vertical * self.index
            self.horizontal = d_horizontal * self.index

            self.framed = orig
            # cv.imshow("orig", orig)

    def print_image_name_dimensions(self):

        cv.imshow(self.name, self.framed)
        print('height ' + self.name+' ', self.vertical)
        print('width ' + self.name+' ', self.horizontal)

    def save_object_dimensions(self):
        fs_write = cv.FileStorage(self.name + '.yml', cv.FILE_STORAGE_WRITE)
        fs_write.write(self.name, (self.vertical, self.horizontal))
        fs_write.release()

    def run_processing(self):
        self.scale()
        self.extract_object()
        self.measure_object()
        self.print_image_name_dimensions()
        self.save_object_dimensions()