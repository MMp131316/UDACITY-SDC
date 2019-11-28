#**Finding line Lines on the Road** 

##Writeup Template

---

**Finding line Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds line lines on the road
* Reflect on your work in a written report


---

### Reflection

###1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.
Given an input coloured image representing a traffic scenario, the algorithm finds the lines on the road. 
The general steps of the algorithm are:
    1. Reading the image
    2. Converting the image to grayscale
    3. Blurring with a small Gaussian kernel
    4. Performing edge detection using the Canny algorithm
    5. Creating a region of interest
    6. Finding the lines using the Hough transform 
    7. Drawing the lines on the road.
The draw lines function has been modified in order to obtain better results. 
The modifications include:
    1. The splitting of the lines in left and right lines based on the line angle.
    A set of threshold intervals have been defined. In my current implementation lines between
    30 and 80 degrees belong to the right line and lines between 100 and 160 belong to the left line.
    2. After separating the lines into left and right I compute an average of the coordinates x and y
    as well as an average for the slope (m). Using these averaged values I compute the new value for
    the left and right x and b 
    3. There are situations in which the current lines are not detected correctly. For example because of shadows on the
    road or all sorts of other gradients that would make our line detection spike suddenly in an unpredicted position. 
    For this reason in the current implementation I have included previous variables for left and right lines. 
    In case an abnormal situation occurs and the difference between current values and previous values is larger than a threshold
    or in case the computed value is not a number, the current values are set to have the previous values.
    4. The implementation also includes a weighted update of the left and right x values. This update procedure takes
    into account the value of the current line and the value of the previously identified lines. The weight(the amount 
    we consider the current line to have an impact on the final result) has been set to 80%, the previous line weight amounts only to 20%
    5. Finally the results are drawn onto the image.
    
###2. Identify potential shortcomings with your current pipeline

A possible shortcoming would be in case there are sudden changes of direction or line. Because of the previous values,
there would still be an amount of the previous values lines included in the current line which have nothing to do with the 
new position. An improvement would be to detect such situations or to predict when they could happen.
Another shortcoming can be caused by the fixed thresholds. We have used fixed thresholds for canny, Hough and also for the line
angles these values should be made adaptively. Another possible workaround would be to use another type of edge detector or filter
to better emphasize the edges.


###3. Suggest possible improvements to your pipeline

1. As extra work I have tried to implement the line detection process using the image vanishing point in c++.I have ran my algorithm on some images acquired with a manta camera that I received from my university. So we can filter the lines and only consider as possible lines the lines that intersect in the vanishing point, which are closest to my current position. I have added the c++ code to the zip and a small movie with the frames. 
2. Another improvement would be to add tracking to the lines.
3. One important improvement which was not mentioned is the following: We have considered straight lines in the current project, however the lines in the real world are not straight....they can be curvy and bend. A possible improvement would be not to fit straight lines in the Hough algorithms but use some sort of quadric higher degree polynomial.
