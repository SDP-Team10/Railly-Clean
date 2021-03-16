import os
from pathlib import Path
from typing import Tuple, Union

import cv2
import numpy as np

from .camera_to_3d_coords import convert_img_to_3d

# image = camera.getImageArray(), hfov = camera.getFov()
def button_match(
    image: np.ndarray, hfov: float, pdist: float, cam_res: Tuple[int, int]
) -> Union[Tuple[float, float, float], None]:
    curr_dir = Path(__file__).absolute()
    template_file_loc = curr_dir.parent.parent / "images" / "button_template_1.jpg"
    # r, g, b = cv2.split(image)
    # image_bgr = cv2.merge(np.array([b, g, r], dtype=np.uint8))
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    template = cv2.imread(str(template_file_loc), 0)

    # keypoint matching using SIFT
    finder = cv2.ORB_create(100000)
    kp1, des1 = finder.detectAndCompute(template, None)
    kp2, des2 = finder.detectAndCompute(image_gray, None)
    # des1, des2 = np.float32(des1), np.float32(des2)
    kp_img = cv2.drawKeypoints(template, kp1, None, color=(0, 255, 0), flags=0)
    cv2.imshow('ORB', kp_img)
    cv2.waitKey(0)

    FLANN_INDEX_LSH = 6
    # index_params = dict(algorithm=0, trees=5)
    index_params= dict(algorithm=FLANN_INDEX_LSH, table_number=12, key_size=20, multi_probe_level=2)
    search_params = dict(checks=50)
    
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1, des2, k=2)

    # Low resolution so ratio = 0.85 
    good = []
    for match in matches:
        if len(match) > 1:
            m, n = match    
            if m.distance < 0.85 * n.distance:
                good.append(m)

    if len(good) >= 4:
        src_pts = np.array(
            [kp1[m.queryIdx].pt for m in good], dtype=np.float32
        ).reshape(-1, 1, 2)
        dst_pts = np.array(
            [kp2[m.trainIdx].pt for m in good], dtype=np.float32
        ).reshape(-1, 1, 2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        print(f"M={M}")
        h, w = template.shape[:]
        pts = np.array(
            [[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]], dtype=np.float32
        ).reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts, M)
        image_match = cv2.polylines(image, dst.astype(np.int32), True, 0, 3, cv2.LINE_AA)
        cv2.imshow("match", image_match)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print(f"Not enough matches are found - {len(good)}/{4}")
        return None
    box = cv2.minAreaRect(dst)
    real_coords = convert_img_to_3d(box[0], hfov, pdist, cam_res)
    return real_coords


if __name__ == "__main__":
    img = cv2.imread(filename="/home/apurv/Railly-Clean/images/scene.png")
    print(button_match(img, 0.785, 1.5, (128, 128)))
