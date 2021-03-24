from typing import Tuple
import numpy as np


def convert_img_to_3d(
    img_pt, hfov: float, pdist: float, cam_res: Tuple[int, int]
) -> Tuple[float, float, float]:
    # 2 * perp_dist * np.sin(hfov/2)
    image_width_m = 2 * pdist * np.sin(hfov / 2)
    m_to_pixel = image_width_m / cam_res[1]
    new_origin = m_to_pixel * cam_res[0] / 2 , m_to_pixel * cam_res[1] / 2
    point_old_origin = (img_pt[0] * m_to_pixel, img_pt[1] * m_to_pixel)
    return (point_old_origin[0] - new_origin[0], point_old_origin[1] - new_origin[1], pdist)
