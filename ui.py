import cv2 as cv
import przetwarzanie

img = cv.imread("piornik.jpg", 3)
obraz = przetwarzanie.Measure(img, "piornik")
obraz.run_processing()

img = cv.imread("perfumy.jpg", 3)
obraz = przetwarzanie.Measure(img, "perfumy")
obraz.run_processing()

img = cv.imread("szczur.jpg", 3)
obraz = przetwarzanie.Measure(img, "szczur")
obraz.run_processing()

img = cv.imread("szczotka.jpg", 3)
obraz = przetwarzanie.Measure(img, "szczotka")
obraz.run_processing()

img = cv.imread("zakreslacz1.jpg", 3)
obraz = przetwarzanie.Measure(img, "zakreslacz1")
obraz.run_processing()

img = cv.imread("zakreslacz2.jpg", 3)
obraz = przetwarzanie.Measure(img, "zakreslacz2")
obraz.run_processing()

cv.waitKey(0)

