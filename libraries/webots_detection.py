from libraries.sticker_detection_wb import see_sticker, where_sticker


def see_that(camera, model_name):
    """Function checks if the camera sees any of the requested object.
    @param camera: webots camera object to use for the recognition
    @param model_name: name of the object to find in webots
    @returns True if a valuable is detected, False otherwise
    """
    # Check if camera has recognition and is enabled
    if not camera.hasRecognition() or camera.getRecognitionSamplingPeriod() == 0:
        # the camera was not enabled or has not recognition
        return "No recognition on camera"
    # If the camera has recognition enabled, get the list of objects
    # And check if any of them are the model searched for
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
    # Check if camera has recognition and is enabled
    if not camera.hasRecognition() or camera.getRecognitionSamplingPeriod() == 0:
        # the camera was not enabled or has not recognition
        return "No recognition on camera"

    # If the camera has recognition enabled, find the button, and return it's position
    if not see_that(camera, model_name):
        # print("no " + model_name + " here man")
        return None, None, None
    else:
        # position relative to the camera - return double[3] - meters xyz
        recognised_objects = [t for t in camera.getRecognitionObjects() if t.get_model() == model_name]
        return recognised_objects[0].get_position()
