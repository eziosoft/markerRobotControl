
# USAGE
# from KalmanFilter import KalmanFilter

# KF=KalmanFilter(0.1, [0, 0])

# state = KF.predict().astype(np.int32)
# cv2.circle(frame, (int(state[0]), int(state[1])), 2, (0, 255, 0), 5) #draw circle in the middle
# cv2.arrowedLine(frame,
#                 (state[0], state[1]), (state[0]+state[2], state[1]+state[3]),
#                 color=(0, 255, 0),
#                 thickness=3,
#                 tipLength=0.2) #draw arrow of movement
# #if marker found
# KF.update(np.expand_dims((my_aruco.x_centerPixel, my_aruco.y_centerPixel), axis=-1))
import numpy as np


class KalmanFilter(object):
    dt = 0.1

    def __init__(self, dt, point):
        self.dt = dt

        # Vecteur d'etat initial
        self.E = np.matrix([[point[0]], [point[1]], [0], [0]])

        # Matrice de transition
        self.A = np.matrix([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        # Matrice d'observation, on observe que x et y
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])

        self.Q = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        self.R = np.matrix([[1, 0],
                            [0, 1]])

        self.P = np.eye(self.A.shape[1])

    def predict(self):
        self.E = np.dot(self.A, self.E)
        # Calcul de la covariance de l'erreur
        self.P = np.dot(np.dot(self.A, self.P), self.A.T)+self.Q
        return self.E

    def update(self, z):
        # Calcul du gain de Kalman
        S = np.dot(self.H, np.dot(self.P, self.H.T))+self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        # Correction / innovation
        self.E = np.round(self.E+np.dot(K, (z-np.dot(self.H, self.E))))
        I = np.eye(self.H.shape[1])
        self.P = (I-(K*self.H))*self.P

        return self.E
