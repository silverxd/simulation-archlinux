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
            return np.ndarray((self.height, self.width, 3), np.uint8, buffer=sensor_image)
        return None

    def get_objects(self, image):
        src = self.convert_sensor_msgs_image_to_opencv_image(image)
        if src is None:
            return []
        cv2.imwrite("/home/iti0201/opencv_orig.png", src)
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        mask_red = self.get_mask(hsv, 0, 20)
        mask_blue = self.get_mask(hsv, 240/2, 20)
        res = cv2.bitwise_and(src,src, mask= mask_red)
        res2 = cv2.bitwise_and(src,src, mask= mask_blue)
        dst = cv2.add(res, res2)
        return_value = []
        contours, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
           if len(contour) > 5:
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            cv2.circle(src, (int(x), int(y)), int(radius), (0,255,0), 2)
            cv2.circle(src, (int(x), int(y)), 1, (255, 255, 255), 2)
            return_value.append(("red sphere", (int(x), int(y)), int(radius)))
        contours, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
           if len(contour) > 5:
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            cv2.circle(src, (int(x), int(y)), int(radius), (0,255,0), 2)
            #ellipse = cv2.fitEllipse(contours[0])
            #cv2.ellipse(src,ellipse,(0,255,0),2)
            #center = (int(ellipse[0][0]), int(ellipse[0][1]))
            cv2.circle(src, (int(x), int(y)), 1, (255, 255, 255), 2)
            return_value.append(("blue sphere", (int(x), int(y)), int(radius)))
        cv2.imwrite("/home/iti0201/opencv.png", src)
        return return_value

if __name__ == "__main__":
    ip = ImageProcessor()
    ip.set_width(1080)
    ip.set_height(1080)
    ip.get_objects([])
