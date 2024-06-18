# RGBD Processing
## Setup python environment

Create conda environment or Python virtual environment with following packages. The project is tested with a conda environment with python 3.8.
```
cv2, numpy, shutil, open3d, json, sklearn, pyrealsense2
```

## Download data

Download the Rosbag file(.bag) and Json file that contains metadata that wanted to process. Move the both .bag file and .json file to path "data/bag_files" under the project root.
The full dataset can be downloaded at [PC4C_CAPSI: Image Data of Capsicum plant growth in Protected Horticulture](https://rds.westernsydney.edu.au/Institutes/HIE/2024/Jayasuriya_N/).
The data are organised under three directories related to three environments and under each directory data are zipped by month.

## Extract image frames

Extracting the image data of the downloaded bag file, supports extracting, color images, depth images, depth scale, and camera intrinsic parameters.
parameters  for cropping series and number frames to extracted can be set in the json file with the same file name as the corresponding .bag file. To set different start and end of frames to be extracted, .rosbag can be played using Realsense viewer software and manually identify the corresponding frame numbers in image frame information.
In the image frame extraction code, a few post-processing filters: decimationa, spacial, and temporal filters from pyrealsense library has been applied. The RGB images are saved as .jpg files and depth images are saved as .png files by preserving the actual depth values which are 16-bit integers.

Steps:
1. Set the filename of downloaded  Rosbag or Json filename (without file extention) to "load_rosbag" function call under the main method of extract_frame.py
2. Change the image frame extraction parameters in the corresponding Json file if needed.
3. Run the "extract_frame.py" python script. 

File can be run using terminal as follows:

Open terminal/powershell and change current dirrect to project root
```
cd src
python extract_frame.py
```

## Process and visualize data

We provide loading extracted images, correcting depth error, and generating 3d scene.
Depth error correction is done using a regression model which built using another collected small data set. See the relavant bublication for more informations.
Following image shows example of per frame visualization that we provided:- A: color image, B:depth image, C: corrected depth image, D: reconstructed 3D scene.

![img.png](img.png)


#### Downlading data 

Please find our RGBD data set of capsicum crop growth [here](to update) and the paper of data description [PC4C_CAPSI](https://www.sciencedirect.com/science/article/pii/S0168169924000607).

#### Cite Our Published Data:
```
@article{jayasuriya2024pc4c_capsi,
  title={PC4C_CAPSI: Image Data of Capsicum plant growth in Protected Horticulture},
  author={Jayasuriya, Namal and Ghannoum, Oula and Hu, Wen and Klause, Norbert and Lianf Weiguang and Guo, Yi},
  journal={Data in Brief},
  volume={},
  pages={},
  year={2024},
  publisher={Elsevier}
}
```

#### Related Publication:
Jayasuriya, N.; Guo, Y.; Hu, W.; Ghannoum, O. Machine Vision Based Plant Height Estimation for Protected Crop Facilities. Computers and Electronics in Agriculture 2024, 218, 108669, doi:10.1016/j.compag.2024.108669.
(https://www.sciencedirect.com/science/article/pii/S0168169924000607)
