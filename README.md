# Surgical Video Feature Analysis

This project processes a surgical video to extract features, analyze their motion using optical flow, classify them as static or dynamic, and visualize them using Voronoi diagrams and Delaunay triangulations.

## Features
- **Feature Detection**: Identifies key points in the surgical video.
- **Optical Flow Analysis**: Tracks motion of the features over time.
- **Feature Classification**: Labels features as static or dynamic.
- **Visualization**: Constructs Voronoi diagrams and Delaunay triangulations for feature visualization.

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
   roslaunch heart_beat_proj load_video.launch
   ```

## Workflow
1. The video is loaded and processed frame by frame.
2. Features are extracted and tracked using optical flow.
3. Features are analyzed to determine whether they are static or dynamic.
4. Voronoi diagrams and Delaunay triangulations are generated to visualize feature relationships.



