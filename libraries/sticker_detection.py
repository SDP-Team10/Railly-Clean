"""Carriage_End controller."""
import cv2
import numpy as np

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# from controller import Robot

# create the Robot instance.
# robot = Robot()

# get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())
# camera = robot.getDevice('camera1')
# camera.enable(timestep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)


def blob_detection(img):
    mask = cv2.inRange(img, (0.0, 0.3, 0.3), (0.1, 0.6, 0.6))
    kernel = np.ones((1, 1), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel2, iterations=2)
    # cv2.imshow("win 1", mask)
    cv2.waitKey(2000)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    nu = np.zeros(img.shape)
    nu = cv2.drawContours(nu, contours, -1, (0, 0, 255), 1)
    cv2.destroyAllWindows()
    return mask


def centroid_detection(cam):
    # img = cam.getImageArray()
    # gbr_img = []
    # for a in range(len(img)):
    #     temp = []
    #     for b in range(len(img[0])):
    #         temp.append([img[b][a][2], img[b][a][1], img[b][a][0]])
    #     gbr_img.append(temp)
    # img = np.array(gbr_img, np.uint8)
    cam.saveImage("../../images/center_image.png", 100)
    img = cv2.imread("../../images/center_image.png")
    img = remove_illumination(img)
    mask = cv2.inRange(img, (0, 0.3, 0.3), (0.1, 0.6, 0.6))
    kernel = np.ones((1, 1), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy)


def camera_point_angle(field_of_view, image_width, point_coordinates):
    angle_pixel_ratio = field_of_view / image_width
    centre_point_x_value = image_width / 2
    angle = (
        point_coordinates[1] - centre_point_x_value
    ) * angle_pixel_ratio  # negative values are returned if the camera points to the right of the sticker and vice versa
    return angle


def matchsticker(image):
    template = cv2.imread("../../images/yellow_sticker.png")
    thresh = cv2.inRange(template, (0.0, 220, 220), (60, 255, 255))
    kernel = np.ones((5, 5), np.uint8)
    thresh = cv2.dilate(thresh, kernel, iterations=3)
    kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel2, iterations=3)
    template_contours, _ = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )
    cnt1 = template_contours[0]
    contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    nu = np.zeros(image.shape)
    nu = cv2.drawContours(nu, contours, -1, (0, 0, 255), 1)
    if len(contours) == 0:
        return False
    cnt2 = contours[0]
    ret = cv2.matchShapes(cnt1, cnt2, 1, 0.0)
    # print(ret)
    if ret < 0.3:
        return True
    else:
        return False


def remove_illumination(image):
    image = image / 1.0
    for i in range(len(image)):
        for j in range(len(image[0])):
            g = image[i][j][0]
            b = image[i][j][1]
            r = image[i][j][2]
            total = g + b + r
            image[i][j][0] = g / total
            image[i][j][1] = b / total
            image[i][j][2] = r / total
    return image


# Press the green button in the gutter to run the script.
def is_carriage_end(cam):
    cam.saveImage("../../images/front_image.png", 100)
    img = cv2.imread("../../images/front_image.png")
    # img = cam.getImageArray()
    # gbr_img = []
    # for a in range(len(img)):
    #     temp = []
    #     for b in range(len(img[0])):
    #         temp.append([img[b][a][2], img[b][a][1], img[b][a][0]])
    #     gbr_img.append(temp)
    # img = np.array(gbr_img, np.uint8)
    # img = cv2.imread("../images/yellow_sticker.png")
    img = remove_illumination(img)
    cv2.destroyAllWindows()
    blob = blob_detection(img)
    matches = matchsticker(blob)
    cv2.destroyAllWindows()
    return matches


count = 0
if __name__ == "__main__":
    cam = None
    centroid_detection(cam)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
# while robot.step(timestep) != -1:
# Read the sensors:
# Enter here functions to read sensor data, like:
#  val = ds.getValue()
#  if count == 100:
#        #print(is_carriage_end(camera))
#   count +=1

# Process sensor data here.

# Enter here functions to send actuator commands, like:
#  motor.setPosition(10.0)

#  pass

# Enter here exit cleanup code.r here exit cleanup code.
