import cv2
import numpy as np
import matplotlib.pyplot as plt
from dt_apriltags import Detector

img = cv2.imread('test_image.png', cv2.IMREAD_GRAYSCALE)

at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

tags = at_detector.detect(img, estimate_tag_pose=False, camera_params=None, tag_size=None)

color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
#plt.imshow(color_img)

for tag in tags:
    for idx in range(len(tag.corners)):
        cv2.line(color_img, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

    cv2.putText(color_img, str(tag.tag_id),
                org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.8,
                color=(0, 0, 255))
    
    translation = tag.pose_t #translation vector/position of tag in camera frame (x,y,z axes)
    distance = tag.pose_t[2] #z-axis for distance
    rotation = tag.pose_R
    tag_id = tag.tag_id
    corners = tag.corners #x,y coordinates of 4 corners detected
    center = tag.center #x,y coordinates of center of tag detected
    decision_margin = tag.decision_margin #how confident it is in the detection (higher = more confident)
    
    print(f"Tag ID: {tag_id}")
    print(f"Corners: {corners}")
    print(f"Center: {center}")
    #print(f"Decision Margin: {decision_margin}")
    print(f"Translation: {translation}")
    print(f"Distance: {distance}")
    #print(f"Rotation: {rotation}")
    print("---")

plt.imshow(color_img)