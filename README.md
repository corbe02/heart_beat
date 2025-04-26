# Surgical Video Feature Analysis

This project processes a surgical video to extract features, analyze their motion using optical flow, classify them as static or dynamic, and triangulate them

## Workflow

### 1. Video Frame Processing
- The video is processed one frame at a time using the `OpticalFlowPose` class.
- Each frame undergoes preprocessing and feature extraction to prepare for motion tracking.

### 2. Feature Extraction (`Feature_extractor` class)
- **Preprocessing**: 
  - The left and right images are separated, and adaptive histogram equalization is applied to improve contrast.
- **Feature Detection**: 
  - The **Good Features to Track (GFTT)** method is used to detect key points in both images.
- **Feature Description**: 
  - **ORB (Oriented FAST and Rotated BRIEF)** is used to generate feature descriptors for each detected keypoint.
- **Feature Matching**: 
  - The `Feature_extractor` uses the **Good Match** function in the `Triangulation` class to filter matches based on their distance
  
### 3. Triangulation (`Triangulation` class)
- **Projection Matrix Calculation**: 
  - After matching features, the `Triangulation` class computes the projection matrix to align the left and right views.
- **Triangulation**:
  - The triangulation process is carried out, which helps compute the 3D positions of features across frames.

### 4. Optical Flow (`OpticalFlow` class)
- **Lucas-Kanade Optical Flow**:
  - The `OpticalFlow` class tracks the motion of the features across consecutive frames using the Lucas-Kanade method.
- **Point Selection**:
  - The points that are correctly tracked in both the left and right images are selected for further analysis.

### 5. Final Image Publishing (`OpticalFlowPose` class)
- After successfully tracking and processing the features, the `OpticalFlowPose` class publishes the final image with the tracked features over a ROS topic.

## How to Run
1. Start the ROS core:
   ```bash
   roscore
   ```
2. Open the ROS image viewer to visualize video frames:
   ```bash
   rqt_image_view
   ```
3. Launch the camera pose estimation node:
   ```bash
   roslaunch heart_beat_proj camera_pose_estimation.launch
   ```
4. Launch the video processing node:
   ```bash
   roslaunch heart_beat_proj opticalflowpose.launch
   ```


Original flow (calling OpticalFlow::computeOpticalFlow in the OpticalFlowPose callback) was this:
## Workflow
1. The video is loaded and processed frame by frame.
2. Features are extracted and tracked using optical flow.
3. Features are analyzed to determine whether they are static or dynamic.
4. Voronoi diagrams and Delaunay triangulations are generated to visualize feature relationships.



