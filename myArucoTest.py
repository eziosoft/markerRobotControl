from aruco.myAruco import *


my_aruco = MyAruco()
my_aruco.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)


while True:
    tickmark = cv2.getTickCount()
    my_aruco.detectMarker(0)

    state = my_aruco.kf_state

    if my_aruco.markerFound:
        print("%f %f, %f %f %f %f | %f" % (my_aruco.x_centerPixel,
              my_aruco.y_centerPixel, state[0], state[1], state[2], state[3], my_aruco.KF.dt))
    else:
        print("- -, %f %f %f %f | %f" % (state[0], state[1], state[2], state[3], my_aruco.KF.dt))
