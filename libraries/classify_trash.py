from pathlib import Path

# if running on RPi
# from tflite_runtime.interpreter import Interpreter
# else
import numpy as np
import tensorflow as tf
import tensorflow.lite as tfl
from typing import Tuple

# The image should be read like this:
# image = camera.getImageArray();
def classify_trash(image) -> Tuple[str, float]:
    curr_dir = Path(__file__).absolute()
    model_path = curr_dir.parent.parent / "ml_models" / "model_quant.tflite"
    print(model_path)
    labels = ("not_trash", "trash")
    interpreter = tfl.Interpreter(model_path=str(model_path))
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
