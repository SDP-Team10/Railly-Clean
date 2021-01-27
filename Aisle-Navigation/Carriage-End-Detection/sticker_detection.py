import cv2
import numpy as np

def blob_detection(img):
    mask = cv2.inRange(img, (0.5, 0, 0), (0.6, 0.45, 0))
    cv2.imshow("Window 3",mask)
    cv2.waitKey(1000)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel2, iterations=3)
    M = cv2.moments(mask)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    nu = np.zeros(img.shape)
    nu = cv2.drawContours(nu, contours, -1, (0,0,255), 1)
    cv2.imshow("Window 4",nu)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()
    return mask

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



def matchsticker(image):
    template = cv2.imread("./images/blue_cross.jpg")
    imgray = cv2.cvtColor(template,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(imgray,127,255,0)
    thresh = cv2.bitwise_not(thresh)
    kernel = np.ones((5, 5), np.uint8)
    thresh = cv2.dilate(thresh, kernel, iterations=3)
    cv2.imshow("Window 4",thresh)
    cv2.waitKey(2000)
    kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
    thresh= cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel2, iterations=3)
    template_contours, _ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cnt1 = template_contours[0]
    contours, _ = cv2.findContours(image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cnt2 = contours[0]
    ret = cv2.matchShapes(cnt1,cnt2,1,0.0)
    print(ret)
    if ret < .1:
        return True
    else:
        return False

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    img = cv2.imread("./images/blue_cross.jpg", 1)
    cv2.imshow("Window 1", img)
    img = remove_illumination(img)
    cv2.imshow("Window 2", img)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()
    matchsticker(blob_detection(img))
