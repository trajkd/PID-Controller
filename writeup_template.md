## Writeup

---

**PID Controller Project**

[//]: # (Image References)

[video1]: ./project_video_result.mp4 "Project Video Result"

## [Rubric](https://review.udacity.com/#!/rubrics/1972/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Effects of the P, I, D components

#### 1. Describe the effect each of the P, I, D components had in your implementation. Is it what you expected?
As explained in the lesson and further detailed in [this article](https://medium.com/intro-to-artificial-intelligence/pid-controller-udacitys-self-driving-car-nanodegree-c4fd15bdc981): 
> - The P, or "proportional", component causes the car to steer proportional (and opposite) to the car’s distance from the lane center (CTE) - if the car is far to the right it steers hard to the left, if it’s slightly to the left it steers slightly to the right. 
> - The D, or "differential", component counteracts the P component’s tendency to ring and overshoot the center line. A properly tuned D parameter will cause the car to approach the center line smoothly without ringing. This means when the car turned enough to reduce the error, it will help not to overshoot through the x axis. 
> - The I, or "integral", component counteracts a bias in the CTE which prevents the P-D controller from reaching the center line. This bias can take several forms, such as a steering drift, but I believe that in this particular implementation the I component particularly serves to reduce the CTE around curves.
> And combining these we can get the PID controller to control the steering value.

At first I didn't expect that my implementation would ever gonna work. But when I got the first working implementation I was impressed from the result. Nonetheless I still expected a more stable driving.

!["Project Video Result"][video1]

### Hyperparameters

#### 1. Describe how the final hyperparameters were chosen.
Parameter optimisation was done manually and using the Twiddle algorithm. I followed the pseudocode from the class and translated it to C++ line by line making sure I initialized the coefficients within reasonable ranges. The initialization was performed manually via suggestions from the student's forum. Twiddle contributed to the further tweaking of the coefficients.