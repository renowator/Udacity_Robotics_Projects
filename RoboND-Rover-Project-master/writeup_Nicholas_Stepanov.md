## Project: Search and Sample Return
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/map_example.png

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  


### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
find_stone(img, yellow_thresh) identifies golden stones applies thresh for yellow color and returns binary image
find_obs(omg, o_thresh) identifies obstacles applies dark thresh to identify obstacles and returns binary image
![alt text][image1]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 
And another! 

process_image(img) applies perspective transform using standard source and destination, identifies navigable terrain, obstacles, stone samples, then calculates the world map position of these and projects it on ground_tuth map, with navigable terrain in blue, obstacles in red and stones in green.
![alt text][image2]
### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

perception_step() uses basic functions tested in jupiter notebook to perspective transform the camera image, identify navigable terrain, obstacles and stones and put them on the map. Mapping will not update with pitch or roll being different from 0 by more than 1. The rover navigation angles and distances are updated with every image.

In decision_step() I made one slight change to the steering angles to be between -10 and 15 to let the rover “stick to the mountain” more easily.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup. 

After running simulator at Good quality and lowest resolution I get an average of 50% mapping and 70% fidelity, but these are different with each run. The rover does not have an “unstuck” mechanism and will remain stuck (between rocks) if it will go there. Sometimes the rover happens to follow a circle whenever the navigable terrain is wide enough to do so. To fix that, the angle rover picks could be calculated with respect to the terrain already visited (less weighted) vs not visited (more weighted). Modifying the code to detect samples and move closer to them decreased the mapping percentage and increased the total time, so I did not include it in this submission. 



**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

To improve this project I would work more on mapping, getting unstuck, trying to travel through the whole map, as well as sample pick up mechanism.
Improved mapping will make it easier to identify places that could be a “trap” for the rover and help him avoid these. 
For picking up samples i would add a new mode to Rover object which would enable anytime the camera sees a rock (there are nonzero pixels in the find_rock binary image) and navigate toward that rock until the near_sample trigger is on.



![alt text][image3]


