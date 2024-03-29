## SUN3DGroundtruthOptimizer

This folder contains a tool designed for extracting a subset of [SUN3D dataset](https://sun3d.cs.princeton.edu/data/) and optimzing groundtruth value using bundle adjustment. 

- Why we optimize the groundtruth?
- "The ground truth poses provided are estimated by visual tracking with loop closure and so are relatively accurate in a global sense, but **have misalignments at the frame level**." [[1]](#1)

## Detailed Optimiztion Process

1. Extract **roughly one frame per second** from the video to avoid too repetitive training samples. [[2]](#2)

2. Apply a standard **SIFT** keypoint detector to identify images that are likely to be noisy and **discard** images in which contains **less than 50** SIFT keypoints. [[2]](#2)

3. Extract **SIFT** features and use the **provided poses as initial guesses for *Gobal bundle adjustment* on all extracted landmark and poses**.

4. Calculate the **relative pose** between every two consecutive frames based on the optimized poses.

5. Extract **SuperPoint** features to construct an **ICP problem**, and use the **relative poses in last step as the initial guesses for** *local bundle adjustment* to further refine the relative poses. The purpose of using different image feature is to take other pixels and depth measurements into consideration.

Step 3, step 4, step 5 are different from directly using SIFT keypoint to optimize the relative poses as in [[1]](#1).

## Dependencies

- OpenCV
```
sudo apt install libopencv-dev
```
- Dependencies for Sophus:
```
sudo apt install libeigen3-dev libspdlog-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
```
- [{fmt}](https://github.com/fmtlib/fmt)
```
cd 3rdParty/
git clone https://github.com/fmtlib/fmt.git

```
- libtorch
```
# the script will check and install it for you.
./ExtractSubset.sh
```

## Usage
1. List the sequences you want to process.
2. Run script ExtractSubset.sh.

## Github Reference
- [PrincetonVision/SUN3DCppReader](https://github.com/PrincetonVision/SUN3DCppReader/tree/master)

- [christian-rauch/super_point_inference](https://github.com/christian-rauch/super_point_inference)

## Paper References
<a id="1"></a>[1] J. Tang, L. Ericson, J. Folkesson, and P. Jensfelt, “GCNv2: Efficient Correspondence Prediction for Real-Time SLAM,” IEEE Robot. Autom. Lett., pp. 1–1, 2019, doi: 10.1109/LRA.2019.2927954.

<a id="2"></a>[2] J. Tang, J. Folkesson, and P. Jensfelt, “Sparse2Dense: From direct sparse odometry to dense 3D reconstruction.” arXiv, Mar. 21, 2019. Accessed: Mar. 09, 2024. [Online]. Available: http://arxiv.org/abs/1903.09199
