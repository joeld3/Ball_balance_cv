""" center_detect.py: Contains different algorithms for finding a circle within an image."""

import cv2
import numpy as np


def contour_center(img, show_img = False):
    """Uses color filtering to mask an image and then detects a contour within the image.

    Notes:
        This function assumes the image is filtered effectively so only a single contour is found in the image.  A
        separate script must first be used to obtain the proper filtering parameters.

    Args:
        img (numpy.array): The image to be analyzed
        show_img (bool): Allows for displaying the image.

    Returns:
        The x,y coordinates of the discovered contour, else None

    """
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    hmin = 0
    hmax = 35
    smin = 0
    smax = 89
    vmin = 148
    vmax = 255
    area_threshold = 800

    lower = np.array([hmin, smin, vmin])
    upper = np.array([hmax, smax, vmax])
    mask = cv2.inRange(imgHSV, lower, upper)
    img_result = cv2.bitwise_and(img, img, mask=mask)

    imgGray = cv2.cvtColor(img_result, cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.medianBlur(imgGray, 7, 4)
    imgCanny = cv2.Canny(imgBlur, 50, 50)

    contours, hierarchy = cv2.findContours(imgCanny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > area_threshold:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02*peri, True)
            x, y, w, h = cv2.boundingRect(approx)
            center = (round(x + (w/2)), round(y + (h/2)))
            if show_img:
                cv2.circle(img, center, 5, (0, 100, 100))
                cv2.imshow('img', img)
            return center

    return None


def hough_circle_center(img, show_img = False):
    """Uses the Hough Circle method to detect a circle within an image.

    Args:
        img (numpy.array): The image to be analyzed
        show_img (bool): Allows for displaying the image.

    Returns:
        The x,y coordinates of the discovered circle if only one circle was found, else None

    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grayblur = cv2.medianBlur(gray, 3)
    circles = cv2.HoughCircles(grayblur, cv2.HOUGH_GRADIENT, 1.5, 100, param1=100, param2=60, minRadius=20,
                               maxRadius=60)

    if circles is not None:
        if len(circles) == 1:
            center = (circles[0][0][0], circles[0][0][1])
            if show_img:
                cv2.circle(img, center, 5, (0, 100, 100))
                cv2.imshow('img',img)
            return center
    else:
        return None
