"""ball_balance_main.py: Uses computer vision to track the location of a ping-pong ball within a hinged track. The main
    control loop detects the ping ball and feeds this into a PID controller. The control output is then
    passed through a COM port to an external microcontroller which can alter the track angle."""

import cv2
import serial
import time
from pid import PID
import aruco_detect
import center_detect_algs
import servo


ARUCO_MARKER_SIZE_MM = 30               # Edge length of ArUco marker in millimeters
ARUCO_TO_CENTER_MM = 60                 # Distance in millimeters the marker center sits from the track center
TRACK_LENGTH_MM = 400                   # The length of the ping-pong ball track in millimeters
TRACK_WIDTH_MM = 56                     # The width of the ping-pong ball track in millimeters
PIXEL_PADDING = 60                      # Pixels added to image cropping to account for image distortion
VID_WIDTH = 1280                        # Width in pixels of the webcam image output
VID_HEIGHT = 720                        # Height in pixels of the webcam image output
SERIAL_PORT = 'COM3'                    # The COM port the microcontroller is connected to
SERIAL_BAUD = 115200                    # The baud rate of the serial interface between the uController and PC
SERVO_PWM_STEPS = 40000                 # The PWM resolution


def fiducial_identify(cap, marker_id, attempts=100):
    """Attempts to detect an ArUco marker within an image. The number of attempts is limited.

    Args:
        cap (cv2.VideoCapture): An opened VideoCapture object
        marker_id (int): The id of the marker to be detected in the image
        attempts (int): The number of times ArUco detection is allowed to fail before an exception is thrown

    Returns:
        An aruco object representing the requested marker

    Raises:
        ValueError: If no ArUco markers are found in the image
        ValueError: If the ArUco marker with provided 'identity' is not found in the image
    """
    found_aruco = None
    attempt_count = attempts

    while found_aruco is None:
        attempt_count -= 1
        read_success, img = cap.read()
        if read_success:
            try:
                found_aruco = aruco_detect.get_marker(img, marker_id)
            except ValueError as e:
                found_aruco = None
                if not attempt_count:
                    raise e
    if found_aruco is None:
        raise ValueError('Image is not being read successfully')
    return found_aruco


def get_track_bounds(cap):
    """Identifies the bounds of the image that the ping-pong ball track falls within.

    Args:
        cap (cv2.VideoCapture): An opened VideoCapture object

    Returns:
        The maximum and minimum x and y bounds that the track falls within
    """
    aruco = fiducial_identify(cap, 0)
    center = aruco.center
    pix_per_mm = aruco.pix_per_mm(ARUCO_MARKER_SIZE_MM)

    # Calculate limits of bounding box that only shows ping-pong ball track
    xmax = round(center[0] + ((TRACK_LENGTH_MM/2) * pix_per_mm) + PIXEL_PADDING)
    xmin = round(center[0] - ((TRACK_LENGTH_MM/2) * pix_per_mm) - PIXEL_PADDING)
    ymax = round(center[1] + (ARUCO_TO_CENTER_MM + (TRACK_WIDTH_MM/2)) * pix_per_mm) + 10
    ymin = round(center[1] + (ARUCO_TO_CENTER_MM - (TRACK_WIDTH_MM/2)) * pix_per_mm) - 10

    # Adjust bounds if they exceed original image size
    xmax = min(xmax, VID_WIDTH)
    xmin = max(xmin, 0)
    ymax = min(ymax, VID_HEIGHT)
    ymin = max(ymin, 0)

    return xmin, xmax, ymin, ymax


# Setup servo range
servo_1 = servo.Servo(SERVO_PWM_STEPS, inverted=True, n90_val=39300, p90_val=35500)

ser = None
cap = None

try:

    # Set up and Open Serial Port
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    ser.close()
    ser.open()

    # Set up and open webcam
    cap = cv2.VideoCapture(0)
    cap.set(3, VID_WIDTH)
    cap.set(4, VID_HEIGHT)

    xmin, xmax, ymin, ymax = get_track_bounds(cap)

    # Calculate indices of track center
    x_center = round((xmax - xmin)/2)
    y_center = round((ymax - ymin)/2)

    # Setup PID
    pid_1 = PID(setpoint=x_center, kp=2, ki=0.08, kd=1, output_max=servo_1.angle_to_out(-45),
                output_min=servo_1.angle_to_out(45), windup_prot=True)
    pid_1.set_output(servo_1.center)
    ser.write(str.encode(str(servo_1.center)))

    t1 = time.perf_counter()

    while True:
        read_success, img = cap.read()
        if read_success:
            img = img[ymin:ymax, xmin:xmax]
            cv2.circle(img, (x_center, y_center), 5, (255, 0, 0), 3)
            center = center_detect_algs.hough_circle_center(img, show_img=True)

            if center is not None:
                t2 = time.perf_counter()
                t_elapsed = t2 - t1
                t1 = t2
                pid_1.update(center[0], t_elapsed)
                ser.write(str.encode(str(round(pid_1.curr_output))))

            cv2.waitKey(1)

except ValueError as e:
    print(e)

finally:
    if cap is not None:
        cap.release()
    if ser is not None:
        ser.close()
