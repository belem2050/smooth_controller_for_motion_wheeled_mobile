# smooth_controller_for_motion_wheeled_mobile

## A Smooth Control Law for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environment

#### This ROS2 package is a controller that takes goal pose, the robot current pose  and computes the velocity command respectfully to some parameters, at which the robot moves with a smooth graceful motion.

#### This software is the implementation of [the article](https://web.eecs.umich.edu/~kuipers/papers/Park-icra-11.pdf) published by [Jong Jin Park](https://www.linkedin.com/in/jong-jin-park-b4885920/) and [Benjamin Kuipers](https://www.linkedin.com/in/benjamin-kuipers-48663214/) at University of Michigan.

### Good to know
Although, there are some implementations of this article such as the one within nav2 ccontrolers plugins, this one is a standalone and is an improved implementation based on an existing ROS1 python version which could be found [here](https://github.com/b51/diff_wheeled_smooth_ctrl).
