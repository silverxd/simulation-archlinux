import cv2
import numpy as np

class ImageProcessor:
    def __init__(self):
        self.width = None
        self.height = None

    def set_width(self, width):
        self.width = width

    def set_height(self, height):
        self.height = height

    def get_mask(self, image, hue, saturation):
        low = np.array([hue, saturation, 107])
        high = np.array([hue, 255, 255])
        return cv2.inRange(image, low, high)

    def convert_sensor_msgs_image_to_opencv_image(self, sensor_image):
        if self.height is not None and self.width is not None:
            mat = np.zeros((self.height, self.width, 3), np.uint8)
            for i in range(self.height):
                for j in range(0, self.width * 3, 3):
                    for k in range(3):
                        mat[i, j//3, k] = sensor_image[i * self.width + j + k]
            return mat
        return None

    def get_objects(self, image):
        src = self.convert_sensor_msgs_image_to_opencv_image(image)
        if src is None:
            return []
        
        hsv = cv2.cvtColor(src, cv2.COLOR_RGB2HSV)
        mask_red = self.get_mask(hsv, 0, 20)
        mask_blue = self.get_mask(hsv, 240/2, 20)
        res = cv2.bitwise_and(src,src, mask= mask_red)
        res2 = cv2.bitwise_and(src,src, mask= mask_blue)
        dst = cv2.add(res, res2)
        contours, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return_value = []
        if len(contours) > 0 and len(contours[0]) > 5:
            ellipse = cv2.fitEllipse(contours[0])
            cv2.ellipse(src,ellipse,(0,255,0),2)
            center = (int(ellipse[0][0]), int(ellipse[0][1]))
            cv2.circle(src, center, 1, (255, 255, 255), 2)
            return_value.append(center)
        contours, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0 and len(contours[0]) > 5:
            ellipse = cv2.fitEllipse(contours[0])
            cv2.ellipse(src,ellipse,(0,255,0),2)
            center = (int(ellipse[0][0]), int(ellipse[0][1]))
            cv2.circle(src, center, 1, (255, 255, 255), 2)
            return_value.append(center)
        return return_value

if __name__ == "__main__":
    ip = ImageProcessor()
    ip.set_width(1080)
    ip.set_height(1080)
    ip.get_objects([])
