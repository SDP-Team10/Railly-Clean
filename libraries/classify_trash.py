import numpy as np

# if running on RPi
# from tflite_runtime.interpreter import Interpreter
# else
import tensorflow as tf
import tensorflow.lite as tfl
from typing import Tuple

# The image should be read like this:
# camera_data = camera.getImage();
def classify_trash(camera_data: str) -> Tuple[str, float]:
    image = np.frombuffer(camera_data, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
    labels = ("not_trash", "trash")
    interpreter = tfl.Interpreter("../ml_models/model_quant.tflite")
    interpreter.allocate_tensors()
    _, height, width, _ = interpreter.get_input_details()[0]["shape"]
    image = tf.image.resize(image, [height, width])
    input_img = np.expand_dims(image, axis=0).astype(np.uint8)

    output_details = interpreter.get_output_details()[0]
    input_index = interpreter.get_input_details()[0]["index"]
    output_index = interpreter.get_output_details()[0]["index"]

    interpreter.set_tensor(input_index, input_img)
    interpreter.invoke()

    predictions = np.squeeze(interpreter.get_tensor(output_index))
    scale, zero_point = output_details["quantization"]
    predictions = scale * (predictions - zero_point)
    return max(zip(labels, predictions), key=lambda x: x[1])