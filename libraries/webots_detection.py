def see_that(camera, model_name):
    """Function checks if the camera sees any valuable.
    @param camera: webots camera object to use for the recognition
    @returns True if a valuable is detected, False otherwise
    """
    # Check if camera has recogniton and is enabled
    if not camera.hasRecognition() or camera.getRecognitionSamplingPeriod() == 0:
        # the camera was not enabled or has not recognition
        return "No recognition on camera"
    # If the camera has recognition enabled, get the list of objects, and check if any of them are the sticker
    return (
        len([t for t in camera.getRecognitionObjects() if t.get_model() == model_name])
        > 0
    )


def where_that(camera, model_name):
    # Check if camera has recogniton and is enabled
    if not camera.hasRecognition() or camera.getRecognitionSamplingPeriod() == 0:
        # the camera was not enabled or has not recognition
        return "No recognition on camera"
    # If the camera has recognition enabled, find the sticker, and return it's position

    if not see_sticker(camera, model_name):
        return "no " + modelname + " here man"
    else:
        # position relative to the camera - return double[3] - meters xyz
        return stickers[0].get_position()
