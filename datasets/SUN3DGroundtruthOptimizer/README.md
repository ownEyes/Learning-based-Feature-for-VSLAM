## SUN3DGroundtruthOptimizer

This folder contains a tool designed for extracting a subset of SUN3D dataset (https://sun3d.cs.princeton.edu/data/) and optimzing groundtruth value using bundle adjustment. 

- Why we optimize the groundtruth?
- "The ground truth poses provided are estimated by visual tracking with loop closure and so are relatively accurate in a global sense, but **have misalignments at the frame level**." [[1]](#1)

## Detailed Optimiztion Process

1. Extract **roughly one frame per second** from the video to avoid too repetitive training samples. [[2]](#2)

2. Apply a standard **SIFT** keypoint detector to identify images that are likely to be noisy and **discard** images in which contains **less than 50** SIFT keypoints. [[2]](#2)

3. Extract **SIFT** features and use the **provided poses as initial guesses for bundle adjustment to update the relative pos**e of each frame pair. [[1]](#1)
## Paper References
<a name="1"></a>[1] J. Tang, L. Ericson, J. Folkesson, and P. Jensfelt, “GCNv2: Efficient Correspondence Prediction for Real-Time SLAM,” IEEE Robot. Autom. Lett., pp. 1–1, 2019, doi: 10.1109/LRA.2019.2927954.

<a name="2"></a>[2] J. Tang, J. Folkesson, and P. Jensfelt, “Sparse2Dense: From direct sparse odometry to dense 3D reconstruction.” arXiv, Mar. 21, 2019. Accessed: Mar. 09, 2024. [Online]. Available: http://arxiv.org/abs/1903.09199
