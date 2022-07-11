from charset_normalizer import detect
import cv2
import mediapipe as mp
import numpy as np
mp_drawing = mp.solutions.drawing_utils
mp_objectron = mp.solutions.objectron

def objectron(path, width, height):

    cap = cv2.VideoCapture(path)

    with mp_objectron.Objectron(static_image_mode=False,
                                max_num_objects=5,
                                min_detection_confidence=0.2,
                                min_tracking_confidence=0.99,
                                model_name='Chair') as objectron:
      while cap.isOpened():
        success, image = cap.read()
        if not success:
          print("Ignoring empty camera frame.")
          continue

        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = objectron.process(image)

        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        array = []
        if results.detected_objects:
            for detected_object in results.detected_objects:
                mp_drawing.draw_landmarks(
                  image, detected_object.landmarks_2d, mp_objectron.BOX_CONNECTIONS)
                mp_drawing.draw_axis(image, detected_object.rotation,
                                     detected_object.translation)
                # cv2.imwrite('img' + path[36] + '.jpg', image)
                subArray = []
                for i in range(9):
                    x = detected_object.landmarks_2d.landmark[i].x
                    y = detected_object.landmarks_2d.landmark[i].y

                    subArray.append([x, y])
                array.append(subArray)
                # break
            break
    cap.release()
    return array