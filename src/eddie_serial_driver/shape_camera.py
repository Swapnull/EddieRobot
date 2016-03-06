#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import sys
from std_msgs.msg import String

def detect_shape(in_contour):

    shape_perimeter = cv2.arcLength(in_contour, True)
    shape_approx_polygon = cv2.approxPolyDP(in_contour, 0.04 * shape_perimeter, True)

    # Determine the shape based on the number of vertices
    if len(shape_approx_polygon) == 3:
        shape_name = "triangle"

    elif len(shape_approx_polygon) == 4:
        shape_name = "square"

    elif len(shape_approx_polygon) == 5:
        shape_name = "pentagon"

    elif len(shape_approx_polygon) == 6:
        shape_name = "hexagon"

    else:
        shape_name = "circle"

    return shape_name

def compute_shape_center(in_contour):

    moment = cv2.moments(in_contour)
    shapeX = int((moment["m10"] / moment["m00"]))
    shapeY = int((moment["m01"] / moment["m00"]))

    return shapeX, shapeY


def detect_shape_and_return_frame(origin_frame, low_thresh, up_thresh):

     # Innit shape type to non detected and the center of each frame
    shape_type = "No shape detected"
    rows, columns, _ = origin_frame.shape
    image_centerX = columns / 2
    image_centerY = rows / 2

    # Change to HSV color space
    hsv_frame = cv2.cvtColor(origin_frame, cv2.COLOR_BGR2HSV)

    # Use the theshold we defined to get only that color
    mask_frame = cv2.inRange(hsv_frame, low_thresh, up_thresh)

    # Get rid of smaller blobs in the mask by eroding & dilating
    mask_frame = cv2.erode(mask_frame, None, iterations=2)
    mask_frame = cv2.dilate(mask_frame, None, iterations=2)

    # Find the contors in the image
    frame_contours = cv2.findContours(mask_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

     # Only execute this code if at least one contour is detected in the video feed
    if len(frame_contours) > 0:

        # Take the largest contour in the mask and use it to create an approximate shape polygon
        largest_contour = max(frame_contours, key=cv2.contourArea)
        shape_type = detect_shape(largest_contour)
        shapeX, shapeY = compute_shape_center(largest_contour)

        cv2.drawContours(origin_frame, [largest_contour], -1, (0, 255, 0), 2)
        cv2.putText(origin_frame, shape_type, (shapeX, shapeY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0 , 0,255), 2)

    else:
        cv2.putText(origin_frame, shape_type, (image_centerX, image_centerY), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2)

    return shape_type, origin_frame

if __name__ == "__main__":

    cam_num = None
    if len(sys.argv) > 1 :
        # Use the argument to specify camera number
        cam_num = int(sys.argv[1])

    # Open up a Video Capture object to get video from the camera
    cap = cv2.VideoCapture(cam_num)
    pub = rospy.Publisher('shape_name', String, queue_size=30)
    rospy.init_node('shape_camera', anonymous=True)

    rate = rospy.Rate(10)

    # Define color thresholds
    upper_threshold = np.array([176, 255, 255])
    lower_threshold = np.array([100, 202, 255])

    # Crete a window to show the visualisation
    cv2.namedWindow('visualisation')

    while(1):

        # Grab a frame from the camera store it in origin_frame
        acquired_successfully, origin_frame = cap.read()

        # If the frame was not acquired then stop program
        if not acquired_successfully:
            break;

        # Give detect_shape_and_return_frame the frame, upper and lower color thresholds, it returns the frame
        # with contours overlaid on the image and the shape that was detected
        shape_name, result_frame = detect_shape_and_return_frame(origin_frame, lower_threshold, upper_threshold)

        cv2.imshow('visualisation', result_frame)
        pub.publish(shape_name)
        print(shape_name)

        rate.sleep()

        # Halt for 5ms and wait for a keypress , this is critical without it the visualisation window won't show
        k = cv2.waitKey(1) & 0xFF
        # Exit if escape key is pressed
        if k == 27:
            break

    cap.release()

    cv2.destroyAllWindows()
