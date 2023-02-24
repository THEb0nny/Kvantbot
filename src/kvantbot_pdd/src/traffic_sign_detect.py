import os
import rospy
import time
import cv2
import numpy as np
import imutils

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

from time import sleep
from std_msgs.msg import String

pub_image = rospy.Publisher('sign_image', Image, queue_size = 1)
pub_sign = rospy.Publisher('sign', String, queue_size = 1)
cvBridge = CvBridge()

img_size = 150
img_rot_angle = 0.06

dir_path = os.path.dirname(os.path.realpath(__file__))
dir_path = dir_path.replace('kvantbot_pdd/src', 'kvantbot_pdd/')
template_stop = cv2.resize(cv2.imread(dir_path + 'template/stop/stop.jpg'), (img_size, img_size))
template_reverse = cv2.resize(cv2.imread(dir_path + 'template/reverse/reverse.jpg'), (img_size, img_size))
template_left = cv2.resize(cv2.imread(dir_path + 'template/left/left.jpg'), (img_size, img_size))
template_right = cv2.resize(cv2.imread(dir_path + 'template/right/right.jpg'), (img_size, img_size))

template_stops, template_reverses, template_lefts, template_rights = [], [], [], []

threshold = 0.85
color_stop = (0, 0, 255)
color_reverse = (255, 255, 255)
color_left = (255, 0, 0)
color_right = (0, 255, 255)

i = 0

def rotate(img_, angle_y):
    angle_x, angle_z = 0, 0
    angle_x = np.deg2rad(angle_x)
    angle_y = np.deg2rad(angle_y)
    angle_z = np.deg2rad(angle_z)
    rotation_matrix_x = np.array([[1, 0, 0],
                                  [0, np.cos(angle_x), -np.sin(angle_x)],
                                  [0, np.sin(angle_x), np.cos(angle_x)]])
    rotation_matrix_y = np.array([[np.cos(angle_y), 0, np.sin(angle_y)],
                                  [0, 1, 0],
                                  [-np.sin(angle_y), 0, np.cos(angle_y)]])
    rotation_matrix_z = np.array([[np.cos(angle_z), -np.sin(angle_z), 0],
                                  [np.sin(angle_z), np.cos(angle_z), 0],
                                  [0, 0, 1]])
    rotation_matrix = np.dot(rotation_matrix_x, np.dot(rotation_matrix_y, rotation_matrix_z))
    center = (img_.shape[1] // 2, img_.shape[0] // 2)
    translation_matrix = np.array([[1, 0, -center[0]],
                                   [0, 1, -center[1]],
                                   [0, 0, 1]])
    back_translation_matrix = np.array([[1, 0, center[0]],
                                        [0, 1, center[1]],
                                        [0, 0, 1]])
    rotation_matrix = np.dot(back_translation_matrix, np.dot(rotation_matrix, translation_matrix))
    rotated_img = cv2.warpPerspective(img_, rotation_matrix, (img_.shape[1], img_.shape[0]))
    gray_img = cv2.cvtColor(rotated_img, cv2.COLOR_BGR2GRAY)
    ret, mask = cv2.threshold(gray_img, 1, 255, cv2.THRESH_BINARY)
    rotated_img[np.where(mask == 0)] = [255, 255, 255]
    return rotated_img


def resize(min_, max_, step):
    template_s_arr = [template_stop, rotate(template_stop, img_rot_angle), rotate(template_stop, -img_rot_angle)]
    template_rev_arr = [template_reverse, rotate(template_reverse, img_rot_angle), rotate(template_reverse, -img_rot_angle)]
    template_l_arr = [template_left, rotate(template_left, img_rot_angle), rotate(template_left, -img_rot_angle)]
    template_r_arr = [template_right, rotate(template_right, img_rot_angle), rotate(template_right, -img_rot_angle)]

    for scale in np.linspace(min_, max_, step)[::-1]:
        for i in range(1):
            template_stops.append(imutils.resize(template_s_arr[i], width=int(template_stop.shape[1] * scale)))
            template_reverses.append(imutils.resize(template_rev_arr[i], width=int(template_reverse.shape[1] * scale)))
            template_lefts.append(imutils.resize(template_l_arr[i], width=int(template_left.shape[1] * scale)))
            template_rights.append(imutils.resize(template_r_arr[i], width=int(template_right.shape[1] * scale)))


def detect_sign(img_, template, i, str, color):
    resized = template[i]
    res = cv2.matchTemplate(img_, resized, cv2.TM_CCOEFF_NORMED)
    loc = np.where(res >= threshold)
    w, h = resized.shape[:2]
    if len(loc[0]) != 0:
        for pt in zip(*loc[::-1]):
            cv2.rectangle(img_, pt, (pt[0] + w, pt[1] + h), color, 2)
        return str
    return 'none'

def cbImageProjection(data):
    global i
    time_loop_start = time.time()
    img = cvBridge.imgmsg_to_cv2(data, "bgr8")
    img = cv2.rotate(img, cv2.ROTATE_180)
    #cv2.namedWindow('img', 0)
    #cv2.resizeWindow('img', 320, 240)

    result = detect_sign(img, template_reverses, i, 'reverse', color_reverse)
    if result == 'none':
        result = detect_sign(img, template_lefts, i, 'left', color_left)
    if result == 'none':
        result = detect_sign(img, template_rights, i, 'right', color_right)
    if result == 'none':
        result = detect_sign(img, template_stops, i, 'stop', color_stop)

    #cv2.imshow('1', template_stops[i])
    #cv2.imshow('img', img)
    #cv2.waitKey(1)

    # print(result)

    if result == 'none':
        i += 1
    if i >= len(template_stops):
        i = 0

    sign_msg = String()
    sign_msg.data = result
    pub_sign.publish(sign_msg)
    img = cv2.rotate(img, cv2.ROTATE_180) # Перевернуть и отзеркалить
    pub_image.publish(cvBridge.cv2_to_imgmsg(img, "bgr8"))
    loop_time = round((time.time() - time_loop_start) * 1000)
    #print("result: ", result + ", loop_time: ", round(loop_time, 2))
    # time.sleep(0.5)


if __name__ == '__main__':
    rospy.init_node('sign_detection')
    resize(0.6, 1.2, 10)
    sum_image = rospy.Subscriber('/usb_cam/image_raw', Image, cbImageProjection, queue_size = 1)
    while not rospy.is_shutdown():
        try:
            rospy.sleep(0.1)
        except KeyboardInterrupt:
            break
