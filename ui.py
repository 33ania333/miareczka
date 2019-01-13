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

img = cv.imread("myszka.jpg", 3)
obraz = przetwarzanie.Measure(img, "myszka")
obraz.run_processing()

cv.waitKey(0)

# img = cv.imread("nawoz.jpg", 3)
# obraz = przetwarzanie.Measure(img, "nawoz")
# obraz.run_processing()

# img = cv.imread("swieczka.jpg", 3)
# obraz = przetwarzanie.Measure(img, "swieczka")
# obraz.run_processing()
# #
# img = cv.imread("dlugopis.jpg", 3)
# obraz = przetwarzanie.Measure(img, "dlugopis")
# obraz.run_processing()
#
# img = cv.imread("pomadka.jpg", 3)
# obraz = przetwarzanie.Measure(img, "pomadka")
# obraz.run_processing()
