""" aruco_detect.py: Finds an ArUco marker from an image and creates an Aruco object """

import cv2
import math


class Aruco():
    """An object representing an ArUco marker

    Attributes:
        identity (int): Id of the ArUco marker based on the marker dictionary in "get_marker"
        corners (list of (int,int)): List of the corner coordinates of the ArUco marker
        center (int,int): The x,y coordinates of the center of the ArUco marker

    """

    def __init__(self, identity, corners):
        """Creates a aruco object

        Args:
            identity (int): The id number of the ArUco marker in its corresponding dictionary
            corners (list of (int,int)): List of the corner coordinates of the ArUco marker
        """
        self.identity = identity
        self.corners = corners
        self.center = self.get_marker_center()

    def get_marker_center(self):
        """Gets the center of the ArUco marker

        Returns:
            The x,y coordinates of the center of the ArUco marker within the image
        """
        x_coord = round((self.corners[0][0] + self.corners[2][0]) / 2)
        y_coord = round((self.corners[0][1] + self.corners[2][1]) / 2)
        return x_coord, y_coord

    def pix_per_mm(self, marker_size=30):
        """Creates a conversion factor relating pixels to millimeters.

        Args:
            marker_size (int): The physical edge size of the ArUco marker in millimeters.

        Returns:
            The number of pixels that represent a millimeter of real space.
        """
        p1 = self.corners[0]
        p2 = self.corners[1]
        hyp = math.sqrt((p1[0] - p2[0])**2 + (p1[1]-p2[1])**2)
        return hyp/marker_size


def get_marker(img, identity=0):
    """Searches an image and returns an aruco object if an aruco marker with the provided identity is found

    Args:
        img (numpy.array): An image containing a ArUco marker
        identity (int): The id number of the marker to look for in the image

    Returns:
        Aruco: An Aruco object with the provided identity

    Raises:
        ValueError: If no ArUco markers are found in the image
        ValueError: If the ArUco marker with provided 'identity' is not found in the image
    """
    dict_4x4_50 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters_create()
    [corners, ids, rejected] = cv2.aruco.detectMarkers(img, dict_4x4_50, parameters=aruco_params)

    try:
        ids = list(ids[0])
        index = ids.index(identity)
        return Aruco(identity, corners[0][index])
    except ValueError:
        raise ValueError(f'ArUco marker with identity={identity} not found in image')
    except TypeError:
        raise ValueError('No ArUco markers found in the image')



