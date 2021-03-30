def see_that(camera, model_name):
    """Function checks if the camera sees any of the requested object.
    @param camera: webots camera object to use for the recognition
    @param model_name: name of the object to find in webots
    @returns True if a valuable is detected, False otherwise
    """
    # Check if camera has recogniton and is enabled
    if not camera.hasRecognition() or camera.getRecognitionSamplingPeriod() == 0:
        # the camera was not enabled or has not recognition
        return "No recognition on camera"
    # If the camera has recognition enabled, get the list of objects, and check if any of them are the model searched for
    return (
        len([t for t in camera.getRecognitionObjects() if t.get_model() == model_name])
        > 0
    )


def where_that(camera, model_name):
    """Function gets the coordinates of the object if found.
    @param camera: webots camera object to use for the recognition
    @param model_name: name of the object to find in webots
    @returns 3D coordinates of the object if found, null otherwise
    """
    # Check if camera has recogniton and is enabled
    if not camera.hasRecognition() or camera.getRecognitionSamplingPeriod() == 0:
        # the camera was not enabled or has not recognition
        return "No recognition on camera"

    # If the camera has recognition enabled, find the sticker, and return it's position
    if not see_sticker(camera, model_name):
        print("no " + modelname + " here man")
        return null, null, null
    else:
        # position relative to the camera - return double[3] - meters xyz
        return stickers[0].get_position()
