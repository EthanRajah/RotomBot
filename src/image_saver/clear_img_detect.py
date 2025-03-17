import cv2
import os
import numpy as np


def laplacian_variance(image):
    """Compute the variance of the Laplacian of an image."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def find_sharpest_image(folder_path):
    """Find the sharpest image in a folder based on Laplacian variance."""
    best_image = None
    best_variance = 0
    best_filename = ""

    for filename in os.listdir(folder_path):
        if filename.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".tiff")):
            image_path = os.path.join(folder_path, filename)
            image = cv2.imread(image_path)

            if image is None:
                continue

            variance = laplacian_variance(image)
            print(f"{filename}: Variance = {variance}")

            if variance > best_variance:
                best_variance = variance
                best_image = image
                best_filename = filename

    print(f"Sharpest Image: {best_filename} with variance {best_variance}")
    return best_filename, best_image


# Example Usage
folder_path = r"/home/jetson/Downloads/"
sharpest_filename, sharpest_image = find_sharpest_image(folder_path)

if sharpest_image is not None:
    cv2.imshow("Sharpest Image", sharpest_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
