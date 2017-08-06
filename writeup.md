## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png
[DH_parameters]: ./misc_images/DH_parameters.png
[UDRF_frames]: ./misc_images/UDRF_file_frames.png
[KR210_DH_params]: ./misc_images/KR210_DH_params.png
[Theta1]: ...


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Determine the DH Parameters

The content of the URDF file shows the following:

Link | Position (x,y,z) in m | Joint Axis | Frame Rotation (rpy) | Min Rotation (deg) | Max Rotation (deg)
Base | (0, 0, 0) | (0, 0, 0) | (0, 0, 0) | N/A | N/A
1 | (0, 0, 0.33) | (0, 0, 1) | (0, 0, 0) | -185 | 185
2 | (0.35, 0, 0.42) | (0, 1, 0) | (0, 0, 0) | -45 | 85
3 | (0, 0, 1.25) | (0, 1, 0) | (0, 0, 0) | -210 | 65
4 | (0.96, 0, -0.54) | (1, 0, 0) | (0, 0, 0) | -350 | 350
5 | (0.54, 0, 0) | (0, 1, 0) | (0, 0, 0) | -125 | 125
6 | (0.193, 0, 0) | (1, 0, 0) | (0, 0, 0) | -350 | 350
7 (Gripper) | (0.0375, 0, 0) | N/A | (0, 0, 0) | N/A | N/A

Where positions are with respect to the preceding link in the preceding link's coordinate frame. See the image below:
![KR210 UDRF Frames][UDRF_frames]

To derive the modified DH parameters, I used definitions as shown in the image below:
![Modified DH Parameters Definitions][DH_parameters]

Where:
 -\alpha<sub>​i−1</sub>​​  (twist angle) = angle between ​hat{Z}<sub>i-1</sub> and hat{Z}<sub>i</sub> measured about hat{X}<sub>i-1</sub>in a right-hand sense
 -a<sub>i-1</sub> (link length) = distance from hat{Z}<sub>i-1</sub> to hat{Z}<sub>i</sub> measured along hat{X}<sub>i-1</sub> where hat{X}<sub>i-1</sub> is perpendicular to both hat{Z}<sub>i-1</sub> and hat{Z}<sub>i</sub>
 -d<sub>i</sub> (link offset) = signed distance from hat{X}<sub>i</sub> measured along hat{Z}<sub>i</sub>
 -\theta<sub>i</sub> (joint angle) = angle between hat{X}<sub>i-1</sub> and hat{X}<sub>i</sub> measured about hat{Z}<sub>i</sub> in a right-hand sense

For the Kuka KR210, the DH parameter layout looks as follows:
![KR210 DH Parameters][KR210_DH_params]

i | a<sub>i-1</sub> (m) | \alpha<sub>i-1</sub> (deg) | d<sub>i</sub> (m) | \theta<sub>i</sub> (deg)
1 | 0 | 0 | 0.75 | \theta<sub>i</sub>
2 | 0.35 | -90 | 0 | \theta<sub>2</sub>-90
3 | 1.25 | 0 | 0 | \theta<sub>3</sub>
4 | -0.054 | -90 | 1.5 | \theta<sub>4</sub>
5 | 0 | 90 | 0 | \theta<sub>5</sub>
6 | 0 | -90 | 0 | \theta<sub>6</sub>
7 (gripper) | 0 | 0 | 0.303 | 0

#### 2. Create Individual Coordinate Frame Transforms

Using the DH parameter table above, I generated transformation matrices from joint i-1 to i using the following relationship defined within the method `_body_fixed_transformation()`:

```python
transform = Matrix([[            cos(theta),           -sin(theta),            0,              a],
                    [ sin(theta)*cos(alpha), cos(theta)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
                    [ sin(theta)*sin(alpha), cos(theta)*sin(alpha),   cos(alpha),   cos(alpha)*d],
                    [                     0,                     0,            0,              1]])
```

Where
 -`theta` = \theta<sub>i</sub>
 -`alpha` = \alpha<sub>i-1</sub>
 -`a` = a<sub>i-1</sub>
 -`d` = d<sub>i</sub>

To build the higher level transformations, I used the method `build_transformations()` as defined below:

```python
def _buildTransforms(self, s):
    T = {}
    for i in range(1,8):
        T[(i-1,i)] = self._body_fixed_transformation(s, i)
        if i>1:
            T[(0,i)] = T[(0,i-1)]*T[(i-1,i)]

    T[(3,6)] = T[(3,4)]*T[(4,5)]*T[(5,6)]

    return T
```

In addition to the joint-to-joint transformations, I also defined methods to calculate the intrinsic rotation matrix given the axis of rotation and angle of rotation (in degrees) as follows:

```python
def _rot(self, axis, q):
    '''
    Return rotation matrix about specified axis
    given rotation angle q (in radians).
    '''

    q *= pi/180.

    if axis == 'X':
        R = [[ 1,      0,       0],
             [ 0, cos(q), -sin(q)],
             [ 0, sin(q),  cos(q)]]
    elif axis == 'Y':
        R = [[  cos(q), 0, sin(q)],
             [       0, 1,      0],
             [ -sin(q), 0, cos(q)]]
    elif axis == 'Z':
        R = [[ cos(q), -sin(q), 0],
             [ sin(q),  cos(q), 0],
             [      0,       0, 1]]
    else:
        raise RuntimeError('{} is not a valid axis. Options are "X", "Y", or "Z".'.format(axis))

    return Matrix(R)
```

This method is needed to calcuate R<sub>corr</sub>, which accounts for the misalignment between the gripper frame in the URDF file and the frame used in the contruction of the DH parameters:

```python
#rotate 180 degrees about z, then -90 degrees about y
self.R_corr = self._rot('Z',180)*self._rot('Y',-90)
```

#### 3. Calculate Joint Angles from End Effector Position and Orientation

The inverse kinematics problem is broken into two sub-problems: finding the wrist center and finding the orientation of the wrist.

For each inverse kinematics calculation iteration, ROS feeds the IK server the end effector (gripper) position (P) and orientation (q) wrt the base frame.

I converted the quaternion to a rotation matrix (R<sub>rpy</sub>) using the `quaternion_matrix()` function from the tf module in my `_return_Rrpy()` method as follows:

```python
def _returnRrpy(self, quaternion):
    #EE rotation matrix
    Trpy = tf.transformations.quaternion_matrix(quaternion)
    Rrpy = Matrix(Trpy[:3,:3])

    #apply correction to orientation to account for 
    #different coord definitions in udrf versus DH params
    Rrpy = Rrpy*self.R_corr

    return Rrpy
```

##### Finding the Wrist Center

To find the wrist center, I used fairly straight-forward trigonometry:

 1. \theta<sub>1</sub>:

 \theta<sub>1</sub> = atan2(WC<sub>y</sub>, WC<sub>x</sub>

 ![Calculating Theta 1][Theta 1]

 2. \theta<sub>2</sub> and \theta<sub>3</sub>:

 To calculate \theta<sub>2</sub> and \theta<sub>3</sub>, I used the law of cosines combined with the construction shown below.

 ![Theta 2 and 3 Construction][Law of Cosines]

 The construction shows:
  *Sides:
   *link 2 (length a<sub>2</sub>)
   *\vec{r<sub>24</sub>}, a vector from the joint 2 frame to the joint 4 frame
   *link 3 (as drawn directly from link 3 frame to the link 4 frame)
  *Angles:
   *a, the angle between \vec{r<sub>24</sub>} and link 2
   *b, the angle between link 2 and link 3
   *angle \vec{r<sub>24</sub>}, the angle from the xy plane (base link) to \vec{r<sub>24</sub>}

 The following equations are used to generate the correct values for \theta<sub>2</sub> and \theta<sub>3</sub>:
  *l<sub>3</sub> = \sqrt{d<sub>4</sub><sup>2</sup> + a<sub>3</sub><sup>2</sup>}


And here's another image! 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


