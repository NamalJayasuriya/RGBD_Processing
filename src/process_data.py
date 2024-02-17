import json
import open3d as o3d
from os import listdir
import numpy as np
import cv2
from sklearn.linear_model import LinearRegression
from utils import *

def load_data(path):
    info =  json.load(open(f'{path}/intrinsics.json'))
    intr_m = info["intrinsic_matrix"]
    intrinsics = Intrinsics(info["width"], info["height"], intr_m[0], intr_m[4], intr_m[2], intr_m[5])
    depth_scale = info["depth_scale"]
    angle = info["angle"]
    max_d = info["max_d"]

    cimgs = listdir(f"{path}/color")
    dimgs = listdir(f"{path}/depth")
    color, depth = [], []

    for i in range(len(cimgs)):
        cimg = cv2.imread(f"{path}/color/{cimgs[i]}")
        cimg =  cv2.cvtColor(cimg, cv2.COLOR_BGR2RGB)
        dimg = cv2.imread(f"{path}/depth/{dimgs[i]}", cv2.IMREAD_UNCHANGED)
        color.append(cimg)
        depth.append(dimg)
    return color, depth, intrinsics, depth_scale, angle, max_d

def o3d_pointcloud(color, depth, intrinsics, max_d=4, v_size=0.0, rotation=[0,0,0], depth_unit=1000, vis=True):
    intrinsic = o3d.camera.PinholeCameraIntrinsic(1280, 720,
                                                  intrinsics.fx,intrinsics.fy,
                                                  intrinsics.ppx,
                                                  intrinsics.ppy)
    flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
    # print(intrinsic)
    pcd = o3d.geometry.PointCloud()
    color = o3d.geometry.Image(color)
    depth = o3d.geometry.Image(depth)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color,
        depth,
        depth_scale=depth_unit,
        depth_trunc=max_d,
        convert_rgb_to_intensity=False)

    temp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
    temp.transform(flip_transform)
    pcd.points = temp.points
    pcd.colors = temp.colors
    if v_size>0:
        pcd = pcd.voxel_down_sample(v_size)
    R = pcd.get_rotation_matrix_from_xzy((degree(rotation[0]), degree(rotation[1]), degree(rotation[2])))
    pcd.rotate(R, center=(0, 0, 0))
    if vis:
        o3d.visualization.draw_geometries([pcd])
    return pcd

def correct_depth_error(depth_img):
    #ground truth depths
    depth_gt = np.array([69.2, 99.5, 156.0, 194.5, 253.8, 296.5])
    #measured depths
    depth_m = np.array([64.438, 102.774, 164.697, 208.097, 277.413, 329.621])
    #linear regresson
    depth_m_reshaped = depth_m[:, np.newaxis]  # Reshape depth_m to have 2 dimensions
    model = LinearRegression().fit(depth_m_reshaped, depth_gt)
    m, b = model.coef_, model.intercept_
    #applying
    return (m*depth_img+b).astype(np.float32)


if __name__ == '__main__':
    DATA_PATH="../data/image_files/R2_G3_R_10_12"
    DATA_PATH="../data/image_files/R4_G3_R"
    colors, depths, intrinsics, depth_scale, angle, max_d = load_data(DATA_PATH)
    for i, (color, depth) in enumerate(zip(colors, depths)):
        depth_c = correct_depth_error(depth)
        cv2imshow(color, depth, depth_c)
        pcd = o3d_pointcloud(color, depth_c, intrinsics, max_d=max_d, v_size=0.001, rotation=[0,90,angle], depth_unit=1/depth_scale, vis=True)

