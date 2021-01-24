import cv2
import numpy as np

def blob_detection(img):
    mask = cv2.inRange(img, (0.5, 0, 0), (0.6, 0.45, 0))
    cv2.imshow("Window 3",mask)
    cv2.waitKey(1000)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    cv2.imshow("Window 4",mask)
    cv2.waitKey(2000)
    return 0

def remove_illumination(image):
    image = image / 1.0
    for i in range(len(image)):
        for j in range(len(image[0])):
            g = image[i][j][0]
            b = image[i][j][1]
            r = image[i][j][2]
            total = g+b+r
            image[i][j][0] = g/total
            image[i][j][1] = b/total
            image[i][j][2] = r/total
    return image


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    img = cv2.imread("./images/blue_cross.jpg", 1)
    cv2.imshow("Window 1", img)
    img = remove_illumination(img)
    cv2.imshow("Window 2", img)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()
    blob_detection(img)
