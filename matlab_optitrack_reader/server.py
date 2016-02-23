#!/usr/bin/python

from natnet import NatNetClient
from matlab_bridge.python_interface import PythonInterface
from tools import inverse_transform, multiply_transform
from threading import Thread, RLock
from numpy import allclose


class MatlabOptitrackServer(object):
    def __init__(self, eef_id, world_id, rate=200):
        self.eef_previous_pose = [[0, 0, 0], [0, 0, 0, 0]]
        self.world_previous_pose = [[0, 0, 0], [0, 0, 0, 0]]
        self.eef_pose = [[0, 0, 0], [0, 0, 0, 0]]
        self.world_pose = [[0, 0, 0], [0, 0, 0, 0]]
        self.lock = RLock()
        self.thread = None
        self.rate = rate

        # Setting up the connection with Matlab
        self.bridge = PythonInterface()

        # Setting up the connection with Optitrack
        print("[Matlab-Optitrack] Setting up the connection...")
        self.client = NatNetClient()
        print("[Matlab-Optitrack] Attempt to receive a single frame...")
        self.client.receive_frame().unpack_data()  # Just check once that we receive data from optitrack/NatNet
        print("[Matlab-Optitrack] server connected and ready to handle requests!")

    def _threaded_update(self):
        while True:
            data = self.client.receive_frame().unpack_data()
            with self.lock:
                self.eef_previous_pose = self.eef_pose
                self.world_previous_pose = self.world_pose
                self.eef_pose = self.get_pose(eef_id, data)
                self.world_pose = self.get_pose(world_id, data)
            # No sleep needed, data availability is blocking

    @staticmethod
    def get_pose(id, data):
        """
        Returns the pose of Rigid body id from data, None if not found
        """
        for rb in data['rb']:
            if rb['id'] == id:
                return rb['position'], rb['orientation']
        return None

    def is_world_visible(self):
        return not (allclose(self.world_previous_pose[0], self.world_pose[0]) and\
                    allclose(self.world_previous_pose[1], self.world_pose[1]))

    def is_eef_visible(self):
        return not (allclose(self.eef_previous_pose[0], self.eef_pose[0]) and\
                    allclose(self.eef_previous_pose[1], self.eef_pose[1]))

    def start(self):
        self.thread = Thread(target=self._threaded_update)
        self.thread.setDaemon(True)
        self.thread.start()

        while True:
            self.bridge.read()
            print("[Matlab-Optitrack] Received request for object {} with respect to {}".format(eef_id, world_id))
            relative_pose_eef = [[0, 0, 0], [0, 0, 0, 0]]
            with self.lock:
                if self.eef_pose is None or self.world_pose is None:
                    print("[Matlab-Optitrack] ERROR End effector is {}, World is {}, sending a vector of zeros".format('unknown' if self.eef_pose is None else 'known',
                                                                                                                       'unknown' if self.world_pose is None else 'known'))
                elif not self.is_eef_visible() or not self.is_world_visible():
                    print("[Matlab-Optitrack] Objects are known but not visible, sending a vector of zeros")
                else:
                    relative_pose_eef = multiply_transform(inverse_transform(self.world_pose), self.eef_pose)
                    print("[Matlab-Optitrack] Sending relative pose {}".format(str(relative_pose_eef)))
            self.bridge.send(relative_pose_eef)

if __name__ == '__main__':
    eef_id, world_id = 3, 4  # ID of rigid bodies (check in Motive > Rigid Body properties)
    MatlabOptitrackServer(eef_id, world_id).start()
