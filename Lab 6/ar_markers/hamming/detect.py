import cv2

from numpy import array, rot90
import numpy as np
from math import fabs

from ar_markers.hamming.coding import decode, extract_hamming_code
from ar_markers.hamming.marker import MARKER_SIZE, HammingMarker

BORDER_COORDINATES = [
    [0, 0], [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6], [1, 0], [1, 6], [2, 0], [2, 6], [3, 0],
    [3, 6], [4, 0], [4, 6], [5, 0], [5, 6], [6, 0], [6, 1], [6, 2], [6, 3], [6, 4], [6, 5], [6, 6],
]

ORIENTATION_MARKER_COORDINATES = [[1, 1], [1, 5], [5, 1], [5, 5]]

WARPED_SIZE = 49


def get_marker_pose(marker, marker_size, camK):
    obj_points = np.array([(0,0,0), (0,marker_size,0), (marker_size,marker_size,0),\
        (marker_size,0,0)], dtype='float32')
    img_points = np.array([marker.contours[0][0], marker.contours[1][0], \
        marker.contours[2][0], marker.contours[3][0]], dtype='float32')
    return cv2.solvePnP(obj_points, img_points, camK, np.array([0,0,0,0], \
        dtype='float32'))

def rotate_contour(contour, persp_transf, rot_num):

    # contour_first_warpped must at wrapped origin since getPerspectiveTransform use contour
    # note that get_turn_number gives conter-clock, we use clock here
    contour_list = contour.tolist();
    for i in range(4 - rot_num):
        contour_list.insert(0, contour_list.pop())

    return np.array(contour_list, dtype='int32')


def validate_and_get_turn_number(marker):
    # first, lets make sure that the border contains only zeros
    for crd in BORDER_COORDINATES:
        if marker[crd[0], crd[1]] != 0.0:
            raise ValueError('Border contians not entirely black parts.')
    # search for the corner marker for orientation and make sure, there is only 1
    orientation_marker = None
    for crd in ORIENTATION_MARKER_COORDINATES:
        marker_found = False
        if marker[crd[0], crd[1]] == 1.0:
            marker_found = True
        if marker_found and orientation_marker:
            raise ValueError('More than 1 orientation_marker found.')
        elif marker_found:
            orientation_marker = crd
    if not orientation_marker:
        raise ValueError('No orientation marker found.')

    rotation = 0
    if orientation_marker == [1, 5]:
        rotation = 1
    elif orientation_marker == [5, 5]:
        rotation = 2
    elif orientation_marker == [5, 1]:
        rotation = 3

    return rotation


def detect_markers(img, marker_size, camK):
    width, height, _ = img.shape
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    
    #normalize image, map brightest pixel to 255 and smalles pixel val to 0.
    '''
    [minVal, maxVal, minLoc, maxLoc] = cv2.minMaxLoc(gray)
    mult = 255/(maxVal-minVal)
    normalized_image = gray - minVal
    normalized_image = np.uint8(mult * normalized_image)
    '''
    

    edges = cv2.Canny(gray, 10, 100)
    #cv2.imshow("edges", edges)
    #cv2.waitKey(1)
    
    contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2:]

    # We only keep the long enough contours
    min_contour_length = min(width, height) / 50
    contours = [contour for contour in contours if len(contour) > min_contour_length]
    canonical_marker_coords = array(((0, 0),
                                     (WARPED_SIZE - 1, 0),
                                     (WARPED_SIZE - 1, WARPED_SIZE - 1),
                                     (0, WARPED_SIZE - 1)),
                                     dtype='float32')

    markers_list = []
    for contour in contours:
        approx_curve = cv2.approxPolyDP(contour, len(contour) * 0.01, True)
        if not (len(approx_curve) == 4 and cv2.isContourConvex(approx_curve)):
            continue

        sorted_curve = array(cv2.convexHull(approx_curve, clockwise=False),
                             dtype='float32')
        persp_transf = cv2.getPerspectiveTransform(sorted_curve, canonical_marker_coords)
        warped_img = cv2.warpPerspective(img, persp_transf, (WARPED_SIZE, WARPED_SIZE))
        warped_gray = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)

        _, warped_bin = cv2.threshold(warped_gray, 127, 255, cv2.THRESH_BINARY)
        marker = warped_bin.reshape(
            [MARKER_SIZE, WARPED_SIZE // MARKER_SIZE, MARKER_SIZE, WARPED_SIZE // MARKER_SIZE]
        )
        marker = marker.mean(axis=3).mean(axis=1)
        marker[marker < 127] = 0
        marker[marker >= 127] = 1

        try:
            # rotate marker by checking which corner is white
            turn_number = validate_and_get_turn_number(marker)
            marker = rot90(marker, k=turn_number)

            #cv2.imshow("bin", warped_bin)
            #cv2.waitKey(10)
            #cv2.imshow("warped_marker", rot90(warped_bin, k=turn_number))
            #cv2.waitKey(10)
            
            # get id
            hamming_code = extract_hamming_code(marker)
            marker_id = int(decode(hamming_code), 2)

        except ValueError:
            continue

        # rotate corner list
        rotated_contour = rotate_contour(sorted_curve, persp_transf, turn_number)
        detected_marker = HammingMarker(id=marker_id, contours=rotated_contour, size=marker_size)

        # get pose and update detected marker
        pose_results = get_marker_pose(detected_marker, detected_marker.size, camK)
        if pose_results[0]:
            detected_marker.rvec = pose_results[1]
            detected_marker.tvec = pose_results[2]
        else:
            # cannot find pose using contours
            continue

        markers_list.append(detected_marker)

    return markers_list
