## Advanced Lane Finding
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
This repository contains code for a project, which has been done as a part of [Udacity's Self Driving Car Nano Degree Program](https://www.udacity.com/drive). The goal is to write a software pipeline to identify the road lane boundaries in a video.

The Steps
---

The steps of this project are listed below. For detailed coding, Advanced Lane Finding project.ipynb can be considered.

### Distortion Correction

The images for camera calibration are stored in the folder called "camera_cal". The camera matrix and distortion co-efficients are computed in order to undistort the image.

![png](writeup/output_6_0.png)


### Gradients and color thresholds

The following color thresholds have been applied:

R & G channel thresholds in order to detect yellow lanes
L channel threshold for taking into account edges in shadows
S channel threshold to distinguish white & yellow lanes

![png](writeup/output_10_0.png)


### Perspective Transform

After examining a sample image, the vertices has been extracted in order to perform a perspective transform. The polygon with these vertices is drawn on the image for visualization. Destination points were chosen in a way that straight lanes appear more or less parallel in the transformed image.

![png](writeup/output_12_0.png)

### Sliding Window Search

After applying histogram method, a sliding window search were performed based on the position of two lines obtained by histogram. 10 windows of width 100 pixels have been used in this method.

![png](writeup/output_16_1.png)

### Searching around a previously detected line

After applying the window search, searching around a margin of fifty pixels of previously detected lane lines has been done in this part.

![png](writeup/output_18_1.png)

### Inverse Transform

In this part:
* The lane line will be painted
* An inverse perspective transform will be performed
* The precessed image with the original image will be combined

![png](writeup/output_22_1.png)

### Example Result

The pipeline was applied to a test image . The original image and the processed image are shown side by side.

![png](writeup/output_26_0.png)

The Video
---
The pipeline is applied to a video, which is stored in the main folder.

The Challenge
---
The "challenge_video_output.mp4" video, stored, is an extra challenge for testing the pipeline under some challenging conditions. 


Discussion
---

## Problems and Challenges

### Gradient & Color Thresholding

* Gradient and color channnel thresholding was a challenging part for me
* The lanes lines in the challenge and harder challenge videos were extremely difficult to detect. They were either too bright or too dull. R & G channel thresholding and L channel thresholding were used in this section.

### Bad Frames

* The challenge video has a section where the vehicle travels underneath a tunnel and no lanes are detected
* To tackle this problem, averaging over the previous well detected frames were resorted
* The lanes in the challenge video change in color, shape and direction. Color threholds were used to tackle this issue. Finally, using R, G channels and L channel thresholds were applied.

## Points of failure & Areas of Improvement

The pipeline seems to fail for the harder challenge video. This video has sharper turns and at very short intervals. The following actions could be considered for improvement:

* As this video has a sharper turns and lengths of a lane is shorter than the previous one, smaller section in perspective transform can be chosen in order to take a better perspective transform
* In order to improve the performance of this code in implementing in harder challenge video, number of frames could be reduces due to fast change of the shape and direction of lanes