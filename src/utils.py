from os import makedirs
from os.path import exists
import shutil
import cv2
import numpy as np

class Intrinsics():
    def __init__(self, width, height, fx, fy, ppx, ppy):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.ppx = ppx
        self.ppy = ppy

def make_clean_folder(path_folder):
    if not exists(path_folder):
        makedirs(path_folder)
    else:
        user_input = input("%s not empty. Overwrite? (y/n) : " % path_folder)
        if user_input.lower() == 'y':
            shutil.rmtree(path_folder)
            makedirs(path_folder)
        else:
            exit()

def cv2imshow(color, depth, depth_c):
    depth = cv2.resize(depth, (640, 320))
    depth_c = cv2.resize(depth_c, (640, 320))
    color = cv2.resize(color, (640, 320))

    depth = cv2.rotate(depth, cv2.ROTATE_90_COUNTERCLOCKWISE)
    depth_c = cv2.rotate(depth_c, cv2.ROTATE_90_COUNTERCLOCKWISE)
    color = cv2.rotate(color, cv2.ROTATE_90_COUNTERCLOCKWISE)

    depth = get_colorized_depth(depth)
    depth_c = get_colorized_depth(depth_c)
    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
    combined_image = np.concatenate((color, depth, depth_c), axis=1)
    cv2.imshow('color, depth and corrected depth images', combined_image)
    while True:
        if cv2.waitKey(1) == 27:
            break
    cv2.destroyAllWindows()

def degree(x):
    return np.pi*(x)/180

def get_colorized_depth(depth):
    return cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.09), cv2.COLORMAP_JET)
