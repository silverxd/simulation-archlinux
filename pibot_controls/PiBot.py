import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from std_msgs.msg import String
from math import degrees
import image_processor


class Validator:
    @staticmethod
    def _get_validate_percentage_arg_function(name: str, start: int, end: int):
        def validate_percentage_arg(i: int):
            def validate_percentage_wrapper(function):
                def validate_percentage(*args):
                    percentage = args[i]
                    if not start <= percentage <= end:
                        raise ValueError(f"{name.capitalize()} percentage must be in range {start} .. {end}")
                    return function(*args)

                return validate_percentage

            return validate_percentage_wrapper

        return validate_percentage_arg

    @staticmethod
    def _validate_grabber_percentage_arg(nth_arg: int):
        return Validator._get_validate_percentage_arg_function("grabber", 0, 100)(nth_arg)

    @staticmethod
    def _validate_speed_percentage_arg(nth_arg: int):
        return Validator._get_validate_percentage_arg_function("speed", -99, 99)(nth_arg)

    @staticmethod
    def validate_speed_percentage(speed_function):
        return Validator._validate_speed_percentage_arg(1)(speed_function)

    @staticmethod
    def validate_grabber_percentage(grb_function):
        return Validator._validate_grabber_percentage_arg(1)(grb_function)


class PiBot:
    def is_simulation(self):
        return True

    def sleep(self, time_in_seconds):
        rospy.sleep(time_in_seconds)

    def get_time(self):
        return rospy.get_time()

    def make_callback_for_sensor(self, attribute_name):
        def callback(value):
            setattr(self, attribute_name, value.data)

        return callback

    def subscribe_to_line_sensors(self):
        prefix = "/robot/line_sensor/"
        suffix = "/intensity"
        rospy.Subscriber(prefix + "left_first" + suffix, Float32,
                         self.make_callback_for_sensor("third_line_sensor_from_left"))
        rospy.Subscriber(prefix + "left_second" + suffix, Float32,
                         self.make_callback_for_sensor("second_line_sensor_from_left"))
        rospy.Subscriber(prefix + "left_third" + suffix, Float32, self.make_callback_for_sensor("leftmost_line_sensor"))
        rospy.Subscriber(prefix + "right_first" + suffix, Float32,
                         self.make_callback_for_sensor("third_line_sensor_from_right"))
        rospy.Subscriber(prefix + "right_second" + suffix, Float32,
                         self.make_callback_for_sensor("second_line_sensor_from_right"))
        rospy.Subscriber(prefix + "right_third" + suffix, Float32,
                         self.make_callback_for_sensor("rightmost_line_sensor"))

    def subscribe_to_distance_sensors(self):
        prefix = "/robot/ir/"
        suffix = "/value"

        rospy.Subscriber(prefix + "front_left" + suffix, Float64, self.make_callback_for_sensor("front_left_laser"))
        rospy.Subscriber(prefix + "front_middle" + suffix, Float64, self.make_callback_for_sensor("front_middle_laser"))
        rospy.Subscriber(prefix + "front_right" + suffix, Float64, self.make_callback_for_sensor("front_right_laser"))

        rospy.Subscriber(prefix + "rear_left_0" + suffix, Float64,
                         self.make_callback_for_sensor("rear_left_straight_ir"))
        rospy.Subscriber(prefix + "rear_left_45" + suffix, Float64,
                         self.make_callback_for_sensor("rear_left_diagonal_ir"))
        rospy.Subscriber(prefix + "rear_left_90" + suffix, Float64, self.make_callback_for_sensor("rear_left_side_ir"))

        rospy.Subscriber(prefix + "rear_right_0" + suffix, Float64,
                         self.make_callback_for_sensor("rear_right_straight_ir"))
        rospy.Subscriber(prefix + "rear_right_45" + suffix, Float64,
                         self.make_callback_for_sensor("rear_right_diagonal_ir"))
        rospy.Subscriber(prefix + "rear_right_90" + suffix, Float64,
                         self.make_callback_for_sensor("rear_right_side_ir"))

    def subscribe_to_encoders(self):
        prefix = "/robot/wheel/"
        suffix = "/position"

        rospy.Subscriber(prefix + "left" + suffix, Int32, self.make_callback_for_sensor("left_wheel_encoder"))
        rospy.Subscriber(prefix + "right" + suffix, Int32, self.make_callback_for_sensor("right_wheel_encoder"))

    def subscribe_to_imu(self):
        rospy.Subscriber("robot/imu/angle", Float32, self.make_callback_for_sensor("rotation_angle"))

    def __init__(self, robot_nr=1):
        # Init node
        rospy.init_node("pibot", anonymous=True)

        # Camera disabled
        self.camera_enabled = False
        self.camera = {'data': None}

        # Distance sensors
        self.front_left_laser = 0
        self.front_middle_laser = 0
        self.front_right_laser = 0
        self.rear_left_straight_ir = 0
        self.rear_left_diagonal_ir = 0
        self.rear_left_side_ir = 0
        self.rear_right_straight_ir = 0
        self.rear_right_diagonal_ir = 0
        self.rear_right_side_ir = 0

        # Line sensors
        self.leftmost_line_sensor = 0
        self.second_line_sensor_from_left = 0
        self.third_line_sensor_from_left = 0
        self.rightmost_line_sensor = 0
        self.second_line_sensor_from_right = 0
        self.third_line_sensor_from_right = 0

        # Encoders
        self.right_wheel_encoder = 0
        self.left_wheel_encoder = 0

        # Publishers
        self.right_wheel_speed_publisher = rospy.Publisher("/robot/wheel/right/vel_cmd", Float32, queue_size=1)
        self.left_wheel_speed_publisher = rospy.Publisher("/robot/wheel/left/vel_cmd", Float32, queue_size=1)

        self.grabber_height_publisher = rospy.Publisher("/robot/grabber/height_cmd", Float32, queue_size=1)
        self.grabber_close_publisher = rospy.Publisher("/robot/grabber/close_cmd", Float32, queue_size=1)

        # Subscribe
        self.subscribe_to_distance_sensors()
        self.subscribe_to_line_sensors()
        self.subscribe_to_encoders()
        self.subscribe_to_imu()

        # Constants
        self.UPDATE_TIME = 0.005
        self.WHEEL_DIAMETER = 0.03
        self.AXIS_LENGTH = 0.132
        self.CAMERA_RESOLUTION = (1080, 1080)  # Width, height
        self.CAMERA_FIELD_OF_VIEW = (62.2, 48.8)  # Horizontal,vertical

        # Wait for initialisation to finish
        rospy.sleep(2)

        self.rotation_angle = 0
        # Rotation
        self.starting_rotation = self.rotation_angle

    def get_front_left_laser(self):
        return self.front_left_laser

    def get_front_middle_laser(self):
        return self.front_middle_laser

    def get_front_right_laser(self):
        return self.front_right_laser

    def get_front_lasers(self):
        return [self.get_front_left_laser(), self.get_front_middle_laser(), self.get_front_right_laser()]

    def get_rear_left_straight_ir(self):
        return self.rear_left_straight_ir

    def get_rear_left_diagonal_ir(self):
        return self.rear_left_diagonal_ir

    def get_rear_left_side_ir(self):
        return self.rear_left_side_ir

    def get_rear_right_straight_ir(self):
        return self.rear_right_straight_ir

    def get_rear_right_diagonal_ir(self):
        return self.rear_right_diagonal_ir

    def get_rear_right_side_ir(self):
        return self.rear_right_side_ir

    def get_rear_irs(self):
        return [
            self.get_rear_left_side_ir(), self.get_rear_left_diagonal_ir(), self.get_rear_left_straight_ir(),
            self.get_rear_right_straight_ir(), self.get_rear_right_diagonal_ir(), self.get_rear_right_side_ir()
        ]

    def get_distance_sensors(self):
        return self.get_front_lasers() + self.get_rear_irs()

    def get_leftmost_line_sensor(self):
        return self.leftmost_line_sensor

    def get_second_line_sensor_from_left(self):
        return self.second_line_sensor_from_left

    def get_third_line_sensor_from_left(self):
        return self.third_line_sensor_from_left

    def get_rightmost_line_sensor(self):
        return self.rightmost_line_sensor

    def get_second_line_sensor_from_right(self):
        return self.second_line_sensor_from_right

    def get_third_line_sensor_from_right(self):
        return self.third_line_sensor_from_right

    def get_left_line_sensors(self):
        return [self.get_leftmost_line_sensor(), self.get_second_line_sensor_from_left(),
                self.get_third_line_sensor_from_left()]

    def get_right_line_sensors(self):
        return [self.get_rightmost_line_sensor(), self.get_second_line_sensor_from_right(),
                self.get_third_line_sensor_from_right()]

    def get_line_sensors(self):
        return self.get_left_line_sensors() + self.get_right_line_sensors()

    @Validator.validate_speed_percentage
    def set_left_wheel_speed(self, percentage):
        """
        :param percentage: -99 .. 99
        """
        value = Float32()
        value.data = percentage
        self.left_wheel_speed_publisher.publish(value)

    @Validator.validate_speed_percentage
    def set_right_wheel_speed(self, percentage):
        """
        :param percentage: -99 .. 99
        """
        value = Float32()
        value.data = percentage
        self.right_wheel_speed_publisher.publish(value)

    @Validator.validate_speed_percentage
    def set_wheels_speed(self, percentage):
        """
        :param percentage: -99 .. 99
        """
        self.set_left_wheel_speed(percentage)
        self.set_right_wheel_speed(percentage)

    def get_right_wheel_encoder(self):
        return self.right_wheel_encoder

    def get_left_wheel_encoder(self):
        return self.left_wheel_encoder

    @Validator.validate_grabber_percentage
    def set_grabber_height(self, height_percentage):
        value = Float32()
        value.data = height_percentage
        self.grabber_height_publisher.publish(value)

    @Validator.validate_grabber_percentage
    def close_grabber(self, percentage):
        value = Float32()
        value.data = percentage
        self.grabber_close_publisher.publish(value)

    def get_rotation(self):
        return degrees(self.rotation_angle - self.starting_rotation)

    def camera_callback(self, message):
        self.camera['data'] = message.data
        self.camera['width'] = message.width
        self.camera['height'] = message.height
        self.image_processor.set_width(self.camera['width'])
        self.image_processor.set_height(self.camera['height'])

    def subscribe_to_camera(self):
        rospy.Subscriber("/robot/camera/image_raw", Image, self.camera_callback)

    def enable_camera(self):
        if not self.camera_enabled:
            self.objects_publisher = rospy.Publisher("/robot/camera/objects", String, queue_size=1)
            self.image_processor = image_processor.ImageProcessor(self.objects_publisher)
            self.subscribe_to_camera()
            self.camera_enabled = True
            self.sleep(0.2)

    def get_camera_objects(self):
        if not self.camera_enabled:
            self.enable_camera()
        return self.image_processor.get_objects(self.camera['data'], String())
