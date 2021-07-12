# Aruco marker tracking with Kalman Filter

import numpy
from Mqtt import *
from robotNavigation import *
from aruco.myAruco import *


my_aruco = MyAruco(False)
my_aruco.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

WPS = []

# WPS =  [(256 , 65),
#  (262 , 63),
#  (262 , 60),
#  (263 , 60),
#  (262 , 60),
#  (262 , 61),
#  (262 , 62),
#  (263 , 63),
#  (264 , 66),
#  (265 , 70),
#  (266 , 75),
#  (266 , 79),
#  (266 , 84),
#  (266 , 90),
#  (265 , 97),
#  (264 ,102),
#  (261 ,108),
#  (257 ,113),
#  (254 ,117),
#  (251 ,120),
#  (246 ,123),
#  (242 ,125),
#  (240 ,126),
#  (239 ,125),
#  (237 ,126),
#  (232 ,128),
#  (230 ,129),
#  (226 ,130),
#  (221 ,131),
#  (214 ,133),
#  (205 ,135),
#  (197 ,137),
#  (190 ,138),
#  (183 ,137),
#  (175 ,135),
#  (167 ,133),
#  (160 ,132),
#  (153 ,131),
#  (143 ,133),
#  (135 ,135),
#  (127 ,137),
#  (120 ,139),
#  (113 ,142),
#  (106 ,145),
#  ( 98 ,152),
#  ( 93 ,156),
#  ( 87 ,161),
#  ( 82 ,165),
#  ( 77 ,169),
#  ( 73 ,173),
#  ( 70 ,178),
#  ( 68 ,183),
#  ( 67 ,186),
#  ( 68 ,190),
#  ( 69 ,194),
#  ( 70 ,197),
#  ( 71 ,198),
#  ( 73 ,201),
#  ( 75 ,202),
#  ( 79 ,207),
#  ( 83 ,210),
#  ( 87 ,210),
#  ( 90 ,210),
#  ( 93 ,209),
#  ( 97 ,206),
#  (102 ,205),
#  (107 ,205),
#  (112 ,205),
#  (117 ,205),
#  (121 ,205),
#  (126 ,206),
#  (131 ,207),
#  (138 ,208),
#  (147 ,210),
#  (153 ,211),
#  (159 ,212),
#  (164 ,211),
#  (169 ,211),
#  (175 ,211),
#  (180 ,211),
#  (185 ,211),
#  (188 ,211),
#  (196 ,210),
#  (208 ,209),
#  (217 ,208),
#  (225 ,208),
#  (232 ,206),
#  (240 ,203),
#  (245 ,201),
#  (247 ,200),
#  (249 ,199),
#  (250 ,198),
#  (252 ,197),
#  (253 ,196),
#  (255 ,194),
#  (258 ,188),
#  (260 ,183),
#  (261 ,180),
#  (262 ,177),
#  (264 ,167),
#  (265 ,160),
#  (266 ,156),
#  (266 ,149),
#  (267 ,142),
#  (267 ,134),
#  (267 ,126),
#  (266 ,119),
#  (266 ,115),
#  (266 ,109),
#  (266 ,103),
#  (265 , 96),
#  (264 , 88),
#  (262 , 83),
#  (260 , 79),
#  (258 , 74),
#  (256 , 69),
#  (252 , 65),
#  (248 , 63),
#  (244 , 62),
#  (240 , 63),
#  (238 , 63),
#  (235 , 65),
#  (230 , 68),
#  (223 , 75),
#  (218 , 80),
#  (216 , 88),
#  (214 , 95),
#  (212 ,101),
#  (208 ,109),
#  (203 ,117),
#  (198 ,124),
#  (193 ,129),
#  (188 ,132),
#  (180 ,134),
#  (172 ,136),
#  (166 ,137),
#  (160 ,138),
#  (152 ,139),
#  (146 ,140),
#  (141 ,139),
#  (136 ,138),
#  (131 ,137),
#  (129 ,137),
#  (127 ,136),
#  (123 ,133),
#  (118 ,129),
#  (113 ,124),
#  (109 ,121),
#  (106 ,118),
#  (102 ,115),
#  ( 99 ,113),
#  ( 97 ,112),
#  ( 94 ,109),
#  ( 91 ,107),
#  ( 88 ,105),
#  ( 85 ,104),
#  ( 84 ,104),
#  ( 84 ,106),
#  ( 86 ,110),
#  ( 86 ,114),
#  ( 85 ,121),
#  ( 84 ,126),
#  ( 83 ,135),
#  ( 83 ,144),
#  ( 83 ,149),
#  ( 84 ,156),
#  ( 85 ,160),
#  ( 85 ,165),
#  ( 86 ,170),
#  ( 88 ,177),
#  ( 90 ,183),
#  ( 93 ,188),
#  ( 95 ,191),
#  ( 98 ,194),
#  (101 ,196),
#  (104 ,198),
#  (110 ,201),
#  (117 ,204),
#  (123 ,206),
#  (128 ,208),
#  (132 ,208),
#  (137 ,208),
#  (143 ,207),
#  (150 ,204),
#  (155 ,202),
#  (160 ,199),
#  (164 ,195),
#  (169 ,192),
#  (173 ,188),
#  (177 ,184),
#  (179 ,180),
#  (181 ,173),
#  (184 ,167),
#  (187 ,163),
#  (191 ,158),
#  (195 ,154),
#  (200 ,151),
#  (205 ,149),
#  (210 ,150),
#  (217 ,151),
#  (222 ,154),
#  (227 ,158),
#  (234 ,165),
#  (244 ,173),
#  (253 ,174),
#  (260 ,172),
#  (265 ,166),
#  (269 ,160),
#  (271 ,152),
#  (273 ,139),
#  (273 ,129),
#  (272 ,120),
#  (271 ,109),
#  (270 ,103),
#  (269 , 96),
#  (268 , 91),
#  (267 , 87),
#  (266 , 84),
#  (266 , 81),
#  (266 , 80),
#  (266 , 81),
#  (266 , 81),
#  (266 , 82),
#  (266 , 82),
#  (266 , 83),
#  (266 , 83),
#  (266 , 84)]



def onMouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print('x = %d, y = %d' % (x, y))
        # my_aruco.target = (x, y)
        WPS.append((x,y))
        my_aruco.points = numpy.array(WPS).reshape((-1,1,2)).astype(numpy.int32)


cv2.namedWindow('frame')
cv2.setMouseCallback('frame', onMouse)


connectMQTT()

i = 0
while True:
    tickmark = cv2.getTickCount()
    my_aruco.detectMarker(0)

    state = my_aruco.kf_state

    r_X = 0
    r_Y = 0
    r_H = 0

    

   

    if my_aruco.markerFoundDelayed:
        r_X = state[0]  # my_aruco.x_centerPixel  # state[0]
        r_Y = state[1]  # my_aruco.y_centerPixel  # state[1]
        r_H = my_aruco.markerYaw

        c1,c2, WPreached, distance_to_target = navigate(
            r_X, r_Y, r_H, my_aruco.target[0], my_aruco.target[1])
        mqtt_control_robot(c1, c2, 100, 100)

        if distance_to_target < 30.:
            # circleSize = 50
            # ix = math.sin(i)*circleSize + my_aruco.center_frameX
            # iy = math.cos(i)*circleSize + my_aruco.center_frameY
            if len(WPS)>0:
                my_aruco.target = (WPS[i][0],WPS[i][1])

            i += 1
            if i > len(WPS)-1:
                i = 0

        # print(distance_to_target)
    else:
        # print("")
        mqtt_control_robot(100, 100, 100, 100)
