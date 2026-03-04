Two initial, working code examples. Both of them set up a NetworkTable that publishes 3 values:  
1. 'post_detected' - Boolean, True if post detected
2. 'post_x' - Number, horizontal centre of the post-mask in pixels (0 is left-side, 480 is max/right-side of image)
3. 'post_depth' - Number, horizontal distance from camera (depth) in metres

At the time of writing, the plan is to use this camera and code for final alignment during climbing stage. The turret-targeting cameras and April-tags will be used to get "close" to the post (30 to 40cm away). Once the "Post Detection" NetworkTable boolen "post_detected" is TRUE, the RoboRIO climbing code will switch to using the "Post Detection" NetworkTable data, "post_x" and "post_depth" to accurately position so it can attach and climb.

We initially expected nothing in the field of vision other than the post (and the wall, which is >50cm behind the climber post) during final alignment. However, this is no longer the case.  
  
UPDATE: the robot will approach the climber from the side, so other robots are assumed to be in view, within 50cm. So colour-masking and minimal object recognition have been added.

New script created to replace post-xz-* scripts:  
&nbsp;&nbsp;&nbsp;&nbsp;post-HSV-Depth-nt.py  
This adds HSV colour masking after the initial distance-clipping mask (>50cm is ignored), mask-cleaning, and uses contours to determine which candidate is most "post-like" based on aspect ratio (taller and thinner). 

To determine HSV values use script:  
&nbsp;&nbsp;&nbsp;&nbsp;ColRngTrackBar.py

Previous script info:
The two post detection algorithms differ, but both scripts set a 50cm clipping distance (everything further away than 50cm is ignored).

1. post-xz-nt.py  
This script is very simple with no object recognition: if there's a pixel, it's assumed to be part of the post. So, the centre of the post is just the average of all the x-coordinates of the pixels.

3. post-xz-nt-objrecon.py  
This script starts with the above script, but adds very simple object recognition: anything that is larger than a few pixels has a quick dimension check: if the height is at least twice the width, then it will be assumed to be a post. Then the centre of the post is just the average of the x-pixels.
