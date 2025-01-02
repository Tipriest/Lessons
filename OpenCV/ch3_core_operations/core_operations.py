"""
    core operations
"""
import cv2
import numpy as np
if __name__ == "__main__":
    
    # images = np.zeros((10, 10, 1), np.uint8)
    images = cv2.imread("../image/sparrow.JPEG")
    images[0, 0, 0] = 255
    cv2.imshow("image", images)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
