STICKER = {b"sticker"}


def see_sticker(camera):
    """Function checks if the camera sees any valuable.
    @param camera: webots camera object to use for the recognition
    @returns True if a valuable is detected, False otherwise
    """
    # Check if camera has recognition and is enabled
    if not camera.hasRecognition() or camera.getRecognitionSamplingPeriod() == 0:
        # the camera was not enabled or has not recognition
        return "No recognition on camera"
    # If the camera has recognition enabled, get the list of objects, and check if any of them are the sticker
    return (
        len(
            STICKER
            & {t.get_model() for t in camera.getRecognitionObjects()}
        )
        > 0
    )


def where_sticker(camera):
    # Check if camera has recognition and is enabled
    if not camera.hasRecognition() or camera.getRecognitionSamplingPeriod() == 0:
        # the camera was not enabled or has not recognition
        return "No recognition on camera"
    # If the camera has recognition enabled, find the sticker, and return it's position
    stickers = [
        t for t in camera.getRecognitionObjects() if t.get_model() == b"sticker"
    ]
    if len(stickers) < 0:
        return "no sticker here man"
    else:
        # position relative to the camera - return double[3] - meters xyz
        pos_to_cam = stickers[0].get_position()
        # position on image - returns[2] - pixels xz
        pos_on_pic = stickers[0].get_position_on_image()
        return pos_to_cam
