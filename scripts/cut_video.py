import cv2

# Open the input video
input_video = "video.mp4"
output_video = "output_video.mp4"

# Open the video file
cap = cv2.VideoCapture(input_video)

# Get the FPS (frames per second) and frame count
fps = cap.get(cv2.CAP_PROP_FPS)
frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

# Define the number of frames to remove from the start (e.g., 100 frames)
frames_to_remove = 5000

# Open the output video writer (you must specify codec, FPS, and frame size)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # for .mp4 files
out = cv2.VideoWriter(output_video, fourcc, fps, 
                      (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), 
                       int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))

# Skip the first 'frames_to_remove' frames
for _ in range(frames_to_remove):
    ret, frame = cap.read()
    if not ret:
        break

# Write the remaining frames to the new video
while True:
    ret, frame = cap.read()
    if not ret:
        break
    out.write(frame)

# Release the video objects
cap.release()
out.release()

print("Video saved as", output_video)
