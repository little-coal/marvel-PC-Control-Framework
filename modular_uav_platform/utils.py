import struct
import numpy as np


def bin2int(int_bin):
    return struct.unpack('>I', int_bin)[0]

def nparray2bin(a):
    return a.tobytes()

def bin2nparray(b):
    return np.frombuffer(b, dtype=float).reshape(13)

def int2bin(integer):
    return struct.pack(">I", integer)