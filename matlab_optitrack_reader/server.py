#!/usr/bin/python

from natnet import NatNetClient
from matlab_bridge.python_interface import PythonInterface
from tools import inverse_transform, multiply_transform
from threading import Thread, RLock


class MatlabOptitrackServer(object):
    def __init__(self, eef_id, world_id, rate=200):
        self.eef_previous_pose = [[0, 0, 0], [0, 0, 0, 0]]
        self.world_previous_pose = [[0, 0, 0], [0, 0, 0, 0]]
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
            eef = self.get_pose(eef_id, data)
            world = self.get_pose(world_id, data)
            with self.lock:
                self.eef_previous_pose = eef
                self.world_previous_pose = world
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

    def start(self):
        self.thread = Thread(target=self._threaded_update)
        self.thread.setDaemon(True)
        self.thread.start()

        while True:
            self.bridge.read()
            print("[Matlab-Optitrack] Received request for object {} with respect to {}".format(eef_id, world_id))
            with self.lock:
                eef = self.eef_previous_pose
                world = self.world_previous_pose

            if eef is None or world is None:
                print("[Matlab-Optitrack] ERROR End effector is {}, World is {}, sending a vector of zeros".format('not visible' if eef is None else 'visible',
                                                                                                                   'not visible' if world is None else 'visible'))
                relative_pose_eef = [[0, 0, 0], [0, 0, 0, 0]]
            else:
                relative_pose_eef = multiply_transform(inverse_transform(world), eef)
                print("[Matlab-Optitrack] Sending relative pose {}".format(str(relative_pose_eef)))
            self.bridge.send(relative_pose_eef)

if __name__ == '__main__':
    eef_id, world_id = 3, 4  # ID of rigid bodies (check in Motive > Rigid Body properties)
    MatlabOptitrackServer(eef_id, world_id).start()
