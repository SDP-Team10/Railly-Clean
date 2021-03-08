# Webots gives model names in ASCII for some reason
VALUABLE_SET = {b"laptop"}


def has_valuable(camera):
    """Function checks if the camera sees any valuable.
    @param camera: webots camera object to use for the recognition
    @returns True if a valuable is detected, False otherwise
    """
    # Check if camera has recogniton and is enabled
    if not camera.hasRecognition() or camera.getRecognitionSamplingPeriod() == 0:
        # the camera was not enabled or has not recognition
        return "No recognition on camera"
    # If the camera has recognition enabled, get the list of objects, and check if any of them are valuables
    return (
        len(VALUABLE_SET & {t.get_model() for t in camera.getRecognitionObjects()}) > 0
    )
