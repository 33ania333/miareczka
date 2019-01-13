import cv2


def resize(filename):
    oriimg = cv2.imread(filename, 3)
    imgScale = 0.5
    newX, newY = oriimg.shape[1] * imgScale, oriimg.shape[0] * imgScale
    newimg = cv2.resize(oriimg, (int(newX), int(newY)))
    # cv2.imshow("Show by CV2", newimg)
    # cv2.waitKey(0)
    cv2.imwrite(filename, newimg)


resize("dlugopis.jpg")
resize("nawoz.jpg")
resize("piornik.jpg")
resize("pomadka.jpg")
resize("swieczka.jpg")
resize("szczur.jpg")