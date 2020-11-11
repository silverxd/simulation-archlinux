import cv2
import numpy as np
import sys

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

    def process_image(self, src, outfile=None):
        src = cv2.medianBlur(src, 17)
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        mask_red = self.get_mask(hsv, 0, 20)
        kernel = np.ones((47, 47), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        cv2.imwrite("/home/iti0201/morph.png", mask_red)
        mask_blue = self.get_mask(hsv, 240/2, 20)
        return_value = []
        contours, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
           if len(contour) > 5:
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            cv2.circle(mask_red,(int(x), int(y)), int(radius), (255, 255, 255), 2)
            return_value.append(("red sphere", (int(x), int(y)), int(radius)))
        contours, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
           if len(contour) > 5:
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            return_value.append(("blue sphere", (int(x), int(y)), int(radius)))
        if outfile is not None:
            cv2.imwrite(outfile, mask_red)
        return return_value

    def get_objects(self, image):
        src = self.convert_sensor_msgs_image_to_opencv_image(image)
        if src is None:
            return []
        return self.process_image(src)

    def read_image_from_file(self, filename):
        src = cv2.imread(cv2.samples.findFile(filename), cv2.IMREAD_COLOR)
        if src is None:
            print ('Error opening image!')
            return None
        return src

    def get_objects_from_file(self, filename, outfile=None):
        src = self.read_image_from_file(filename)
        if src is not None:
            return self.process_image(src, outfile)
        return []

def main(argv):
    if len(argv) < 4:
        print("Usage: opencv.py width height infile outfile")
        return None
    ip = ImageProcessor()
    ip.set_width(argv[0])
    ip.set_height(argv[1])
    print(ip.get_objects_from_file(argv[2], outfile=argv[3]))


if __name__ == "__main__":
    main(sys.argv[1:])
