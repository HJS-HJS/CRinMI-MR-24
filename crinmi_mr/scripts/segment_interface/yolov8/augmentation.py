import cv2
import os
import numpy as np


def increase_brightness(img, value):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v = np.clip(v + value, 0, 255)
    hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def change_brightness(img, value=30):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v = cv2.add(v, value)
    v[v > 255] = 255
    v[v < 0] = 0
    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

base_path = os.getcwd()
input_folder = base_path + "/images/train"
output_folder = base_path + "/images/aug"

if not os.path.exists(output_folder):
    os.makedirs(output_folder)

brightness_value = 20  # You can adjust this value to control brightness (positive for increasing, negative for decreasing)

i = 0
for filename in os.listdir(input_folder):
    img_path = os.path.join(input_folder, filename)
    img = cv2.imread(img_path)

    # Increase brightness
    # augmented_img = increase_brightness(img, brightness_value)
    augmented_img = change_brightness(img, value=-100)  # increases

    # Save augmented image
    output_path = os.path.join(output_folder, filename)
    cv2.imwrite(output_path, augmented_img)
    i += 1

cv2.destroyAllWindows()