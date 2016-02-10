#!/usr/bin/python

from natnet import NatNetClient
from matlab_bridge.python_interface import PythonInterface
from tools import inverse_transform, multiply_transform
from tempfile import gettempdir
from os.path import join

# Setting up the connection with Matlab
bridge = PythonInterface()
bridge.matlab_flag = join(gettempdir(), 'flagMatlabFinished.txt')
bridge.python_flag = join(gettempdir(), 'flagPythonFinished.txt')
bridge.matlab_file = join(gettempdir(), 'matlab_file.json')
bridge.python_file = join(gettempdir(), 'python_file.json')

# Setting up the connection with Optitrack
client = NatNetClient()
client.receive_frame().unpack_data()['markersets'] # Just check once that we receive data from optitrack/NatNet

print("[Matlab-Optitrack] server ready to handle requests")
print("Temp dir is {}".format(gettempdir()))

# Objects to serve pose of
eef_id, world_id = 3, 4  # ID of rigid bodies (check in Motive > Rigid Body properties)

def get_pose(id, data):
    """
    Returns the pose of Rigid body id from data, None if not found
    """
    for rb in data['rb']:
        if rb['id'] == id:
            return rb['position'], rb['orientation']
    return None

while True:
    bridge.read()
    print("[Matlab-Optitrack] Received request for object {} with respect to {}".format(eef_id, world_id))
    data = client.receive_frame().unpack_data()
    eef = get_pose(eef_id, data)
    world = get_pose(world_id, data)
    if eef is None or world is None:
        print("[Matlab-Optitrack] ERROR End effector is {}, World is {}, sending a vector of zeros".format('not visible' if eef is None else 'visible',
                                                                                                           'not visible' if world is None else 'visible'))
        relative_pose_eef = [[0, 0, 0], [0, 0, 0, 0]]
    else:
        relative_pose_eef = multiply_transform(inverse_transform(world), eef)
        print("[Matlab-Optitrack] Sending relative pose {}".format(str(relative_pose_eef)))
    bridge.send(relative_pose_eef)
    
