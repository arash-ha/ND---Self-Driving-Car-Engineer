# **Finding Lane Lines on the Road** 

## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file. But feel free to use some other method and submit a pdf if you prefer.

---

**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)

[image1]: ./examples/grayscale.jpg "Grayscale"

---

### Reflection


### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 5 steps.

1)	Converting to grayscale: we process the image or frame within the video by changing colorful image into grayscale. Each pixel can be represented with a single 8-bit number (0~255).
2)	Smoothing: to prevent sharp change in the image, we use Gaussian blurring in this project. We do so to avoid noise because we need to detect an edge in the next step. Smoothing can remarkably enhance the accuracy of edge detection later.
3)	Edge Detection: the lane line has a different color from its neighboring region. It is quite natural to detect edges in the image, which are more likely to be the lane lines. Here we use classic Canny Edge Detection.
4)	Region finding of Interest: there are too many edges found from the previous step. To narrow down to region that more likely have a lane line, we need to restrict our search within a more small portion of the image called the region of interest. 
5)	Line finding from points: the previous step generates many points consisting of edges. To detect a continuous lane line, we need to find a line from those points. The most classic approach is to use Hough Transform and find the intersection of different curves in Hough Space.

If you'd like to include images to show how the pipeline works, here is how to include an image: 

![alt text][image1]


### 2. Identify potential shortcomings with your current pipeline


1)One potential shortcoming in this project is that the detection method works very well on the straight road, however, this method may not accurately detect lane lines on curvy roads or roads with a steep slope. 

2)Another shortcoming could be a view-blocking problem. This method may have an issue in some complicated scenarios when the car’s view can be blocked by other cars or front car is right on the edge. So that the camera is not able to capture the shape of lane marks and it may fail in detection of lane lines.



### 3. Suggest possible improvements to your pipeline

A possible improvement would be to ...

Another potential improvement could be to ...

Possible improvements could be considered in the following:
1)The first possible improvement would be done in order to find lane lines on curvy roads. To be able to do this method, it is needed to use higher-order polynomial such as y=ax^2+bx+c, y=ax^3+bx^2+cx+d or even higher which constants can be computed in a real time based on captured images. But we need to be careful about computational complexity because it is in real-time and we cannot use high order models.
2)Another issue regarding finding lane lines is to resolve the view-blocking problem by using reinforcement learning technique.  Since the self-driving car has roads information, it can use that available information to infer for predicting lane in the next few seconds and reinforce its inference using the real situation as feedback.

