import time
import sys
import cv2
from cv2 import aruco
from aruco.helpersAruco import *
import numpy as np
import urllib.request
from aruco.kalmanFilter import KalmanFilter


class MyAruco():
    std = False

    # camera url
    url = 'http://192.168.0.64/capture'
    # url = 'http://192.168.138.131:8080/stream?topic=/webcam/image_raw'

    if std:
        cap = cv2.VideoCapture(0)
        cap.set(3,1280)
        cap.set(4,1024)
        

    # --- Define Tag
    marker_size = 5  # - [cm]
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    font = cv2.FONT_HERSHEY_PLAIN
    parameters = aruco.DetectorParameters_create()

    # camera calibration
    path = sys.path[0]
    # print(path)
    if std != True:
        camera_matrix = np.loadtxt(
            (path + '\\aruco\cameraMatrix.txt'), delimiter=',')
        camera_distortion = np.loadtxt(
            path + '\\aruco\cameraDistortion.txt', delimiter=',')
    else:
        # virtual camera
        camera_matrix = np.array([467.74270306499267, 0.0, 320.5, 0.0,
                                  467.74270306499267, 240.5, 0.0, 0.0, 1.0]).reshape(3, 3)
        camera_distortion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    # print(camera_matrix)
    # print(camera_distortion)

    markerX = 0
    markerY = 0
    markerZ = 0
    markerPitch = 0
    markerRoll = 0
    markerYaw = 0

    cameraX = 0
    cameraY = 0
    cameraZ = 0
    cameraPitch = 0
    cameraRoll = 0
    cameraYaw = 0

    markerFound = False

    x = 0
    y = 0

    x_centerPixel = 0
    y_centerPixel = 0

    KF = KalmanFilter(0.1, [0, 0])
    kf_state = KF.predict().astype(np.int32)

    def detectMarker(self, id_to_find):
        tickmark = cv2.getTickCount()
        self.kf_state = self.KF.predict().astype(np.int32)

        if self.std:
            ret, frame = self.cap.read()

        else:
            imgResp = urllib.request.urlopen(self.url)
            imgNp = np.array(bytearray(imgResp.read()), dtype=np.uint8)
            frame = cv2.imdecode(imgNp, cv2.IMREAD_COLOR)

        height, width = frame.shape[:2]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self.aruco_dict, parameters=self.parameters,
                                                     cameraMatrix=self.camera_matrix, distCoeff=self.camera_distortion)

        cv2.rectangle(frame, (0, 0), (180, 45), (100, 255, 100), -1)
        if ids is not None and ids[0] == id_to_find:

            x_sum = corners[0][0][0][0] + corners[0][0][1][0] + \
                corners[0][0][2][0] + corners[0][0][3][0]
            y_sum = corners[0][0][0][1] + corners[0][0][1][1] + \
                corners[0][0][2][1] + corners[0][0][3][1]

            self.x_centerPixel = x_sum*.25
            self.y_centerPixel = y_sum*.25

            # -- ret = [rvec, tvec, ?]
            # -- array of rotation and position of each marker in camera frame
            # -- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
            # -- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
            ret = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.camera_distortion)

            # -- Unpack the output, get only the first
            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

            # -- Draw the detected marker and put a reference frame over it

            aruco.drawAxis(frame, self.camera_matrix,
                           self.camera_distortion, rvec, tvec, 10)

            self.markerX = tvec[0]
            self.markerY = tvec[1]
            self.markerZ = tvec[2]

            # -- Print the tag position in camera frame
            str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f" % (
                self.markerX, self.markerY, self.markerZ)
            cv2.putText(frame, str_position, (0, 10), self.font,
                        0.5, (0, 0, 0), 1, cv2.LINE_AA)

            # -- Obtain the rotation matrix tag->camera
            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T

            # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(
                R_flip*R_tc)

            self.markerRoll = math.degrees(roll_marker)
            self.markerPitch = math.degrees(pitch_marker)
            self.markerYaw = math.degrees(yaw_marker)

            # -- Print the marker's attitude respect to camera frame
            str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (self.markerRoll, self.markerPitch,
                                                                          self.markerYaw)
            cv2.putText(frame, str_attitude, (0, 20), self.font,
                        0.5, (0, 0, 0), 1, cv2.LINE_AA)

            # -- Now get Position and attitude f the camera respect to the marker
            pos_camera = -R_tc*np.matrix(tvec).T

            self.cameraX = pos_camera[0]
            self.cameraY = pos_camera[1]
            self.cameraZ = pos_camera[2]

            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f" % (
                self.cameraX, self.cameraY, self.cameraZ)
            cv2.putText(frame, str_position, (0, 30), self.font,
                        0.5, (0, 0, 0), 1, cv2.LINE_AA)

            # -- Get the attitude of the camera respect to the frame
            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(
                R_flip*R_tc)

            self.cameraPitch = math.degrees(pitch_camera)
            self.cameraRoll = math.degrees(roll_camera)
            self.cameraYaw = math.degrees(yaw_camera)

            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (self.cameraRoll, self.cameraPitch,
                                                                          self.cameraYaw)
            cv2.putText(frame, str_attitude, (0, 40), self.font,
                        0.5, (0, 0, 0), 1, cv2.LINE_AA)

            self.KF.update(np.expand_dims(
                (self.x_centerPixel, self.y_centerPixel), axis=-1))
            self.markerFound = True

        else:
            self.markerFound = False

        # --- Display the frame
        # frame = cv2.undistort(frame, camera_matrix, camera_distortion, None, None)
        # aruco.drawDetectedMarkers(frame, corners)

        # y += a * (x - y)
        # a=0.5
        # self.x=a*(self.markerX-self.x)
        # self.y=a*(self.markerY-self.y)

        # frame= cv2.circle(frame, (int(self.x),int(self.y)), 10, (255,0,0), 1)
        frame = cv2.line(frame, (0, int(height/2)),
                         (width, int(height/2)), (255, 255, 255), 1)
        frame = cv2.line(frame, (int(width/2), 0),
                         (int(width/2), height), (0, 0, 255), 1)

        fps = cv2.getTickFrequency()/(cv2.getTickCount()-tickmark)
        cv2.putText(frame, "FPS: {:05.2f}".format(
            fps), (0, 60), self.font, 1, (255, 0, 0), 2)

        # draw circle in the middle
        cv2.circle(frame, (int(self.kf_state[0]), int(
            self.kf_state[1])), 2, (255, 255, 255), 5)
        cv2.arrowedLine(frame,
                        (int(self.kf_state[0]), int(self.kf_state[1])),
                        (int(self.kf_state[0] + self.kf_state[2]),
                         int(self.kf_state[1] + self.kf_state[3])),
                        color=(255, 255, 0),
                        thickness=1,
                        tipLength=0.2)  # draw arrow of movement

        self.KF.dt = cv2.getTickCount()-tickmark

        cv2.imshow('frame', frame)
        k = cv2.waitKey(1)
