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
    r, g, b = cv2.split(image)
    image_bgr = cv2.merge(np.array([b, g, r], dtype=np.uint8))
    image_gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    template = cv2.imread(str(template_file_loc), 0)

    # keypoint matching using SIFT
    finder = cv2.SIFT_create()
    kp1, des1 = finder.detectAndCompute(template, None)
    kp2, des2 = finder.detectAndCompute(image_gray, None)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1, des2, k=2)

    # Low resolution so any match is good
    good = []
    for m, n in matches:
        if m.distance < 1 * n.distance:
            good.append(m)

    if len(matches) > 4:
        src_pts = np.array(
            [kp1[m.queryIdx].pt for m in good], dtype=np.float32
        ).reshape(-1, 1, 2)
        dst_pts = np.array(
            [kp2[m.trainIdx].pt for m in good], dtype=np.float32
        ).reshape(-1, 1, 2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        h, w = template.shape[:]
        pts = np.array(
            [[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]], dtype=np.float32
        ).reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts, M)
        # image_gray = cv2.polylines(image_gray, dst.astype(np.int32), True, 255, 3, cv2.LINE_AA)
        # cv2.imshow("match", image_gray)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
    else:
        print(f"Not enough matches are found - {len(good)}/{4}")
        return None
    box = cv2.minAreaRect(dst)
    real_coords = convert_img_to_3d(box[0], hfov, pdist, cam_res)
    return real_coords


if __name__ == "__main__":
    img = cv2.imread(filename="/home/apurv/Railly-Clean/images/one_pt_5_m_button.png")
    print(button_match(img, 0.785, 1.5, (128, 128)))
