#!/usr/bin/env python
"""
Example of SIFT based object detection.  Code is mostly borrowed from

http://docs.opencv.org/3.1.0/db/d27/tutorial_py_table_of_contents_feature2d.html

"""
import cv2
import numpy as np

def camera_loop():
    """
    """
    width = 640
    height = 480

    # Extract features from the target image
    target = cv2.imread('mug.png')
    target_gray = cv2.cvtColor(target, cv2.COLOR_BGR2GRAY)
    sift = cv2.xfeatures2d.SIFT_create()

    # find the keypoints and descriptors with SIFT
    kp_target, des_target = sift.detectAndCompute(target_gray, None)

    # Set up the capture object
    capture = cv2.VideoCapture(0)
    capture.set(3, width)
    capture.set(4, height)
    width = capture.get(3)
    height = capture.get(4)


    # Set up the keypoint matcher.
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    MIN_MATCH_COUNT = 10

    while True:
        _, img = capture.read()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kp, des = sift.detectAndCompute(gray, None)

        matches = flann.knnMatch(des_target, des, k=2)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good.append(m)

        img_show = np.array(img)
        
        if len(good) > MIN_MATCH_COUNT:
            src_pts = np.float32([kp_target[m.queryIdx].pt for m in good]).reshape(-1,1,2)
            dst_pts = np.float32([kp[m.trainIdx].pt for m in good]).reshape(-1,1,2)

            M, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

            h, w, _ = target.shape
            pts = np.float32([[0 , 0],
                              [0, h - 1],
                              [w - 1, h - 1],
                              [w - 1, 0]]).reshape(-1, 1, 2)
            dst = cv2.perspectiveTransform(pts, M)

            img_show = cv2.polylines(img_show, [np.int32(dst)],
                                     True, 255, 3, cv2.LINE_AA)
           
        else:
            error_msg = "Not enough matches are found - {}/{}"
            print error_msg.format(len(good), MIN_MATCH_COUNT)

        # Draw the keypoint matches
        img_show = cv2.drawMatches(target, kp_target, img_show, kp,
                                   good, img_show, flags=2)

        cv2.imshow("Correspondences", img_show)
        
        c = cv2.waitKey(33)

if __name__ == "__main__":
    camera_loop()
