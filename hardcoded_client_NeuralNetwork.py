#Import scikit modules
from sklearn.neural_network import MLPRegressor
from sklearn.model_selection import train_test_split
import numpy as np

# Import other modules needed
import time as tm
import traceback as tb
import math as mt
import sys as ss
import os
import socket as sc

ss.path += [os.path.abspath(relPath) for relPath in ('..',)]

import socket_wrapper as sw
import parameters as pm


class NeuralNetwork:
    def __init__(self, sampleFileName):
        self.sampleFileName = sampleFileName

        # Open samplefile and read the file line for line. Each line will be added as an array in self.samples
        with open(self.sampleFileName) as self.sampleFile:
            self.samples = np.array([[float(word) for word in line.split()]
                                    for line in self.sampleFile.readlines()])

            # Create two variables X and y
            X = self.samples[:, 0:16]
            y = self.samples[:, -1]

            # Split the dataset
            X_train, X_test, y_train, y_test = train_test_split(X, y, random_state=1)

            # Train function
            self.regr = MLPRegressor(random_state=1, max_iter=5000, hidden_layer_sizes=500, verbose=True).fit(X_train, y_train)


class HardcodedClient:
    def __init__(self, sampleFileName):
        self.sampleFileName = sampleFileName
        self.steeringAngle = 0
        self.neuralNetwork = NeuralNetwork(self.sampleFileName)

        with sc.socket(*sw.socketType) as self.clientSocket:
            self.clientSocket.connect(sw.address)
            self.socketWrapper = sw.SocketWrapper(self.clientSocket)
            self.halfApertureAngle = False

            while True:
                self.input()
                self.logTraining()
                self.sweep()
                self.output()
                tm.sleep(0.02)


    #120 values lidar hoek. Half Aperture Angle is de helft van 120 : 16 | 16 sector angles van 7,5
    def input(self):
        sensors = self.socketWrapper.recv()

        if not self.halfApertureAngle:
            self.halfApertureAngle = sensors['halfApertureAngle']
            self.sectorAngle = 2 * self.halfApertureAngle / pm.lidarInputDim
            self.halfMiddleApertureAngle = sensors['halfMiddleApertureAngle']

        if 'lidarDistances' in sensors:
            self.lidarDistances = sensors['lidarDistances']
        else:
            self.sonarDistances = sensors['sonarDistances']

    def lidarSweep(self):
        # Make array sample variable. The -1 will take the last value from the array. So afterwards there are in total 16 values
        arraysample = self.sample[:-1]

        # Predict the steering angle
        predict = self.neuralNetwork.regr.predict([arraysample])[0]

        # The predicted steering angle will be used to steer.
        self.steeringAngle = predict
        self.targetVelocity = pm.getTargetVelocity(predict)

    def sonarSweep(self):
        pass

    def sweep(self):
        if hasattr(self, 'lidarDistances'):
            self.lidarSweep()
        else:
            self.sonarSweep()

    # Stuurt auto aan
    def output(self):
        actuators = {
            'steeringAngle': self.steeringAngle,
            'targetVelocity': self.targetVelocity
        }

        #Send variable to car to drive
        self.socketWrapper.send(actuators)

    def logLidarTraining(self):
        # Set all 16 values of 20
        self.sample = [pm.finity for entryIndex in range(pm.lidarInputDim + 1)]

        # Get sector index
        for lidarAngle in range(-self.halfApertureAngle, self.halfApertureAngle):
            # Round sectorIndex
            sectorIndex = round(lidarAngle / self.sectorAngle)

            self.sample[sectorIndex] = min(self.sample[sectorIndex], self.lidarDistances[lidarAngle])

    def logTraining(self):
        if hasattr(self, 'lidarDistances'):
            self.logLidarTraining()
        else:
            pass

HardcodedClient(pm.sampleFileName)
