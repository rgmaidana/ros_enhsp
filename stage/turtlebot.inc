define kinect ranger
(
  sensor
  (
    range [0.3 15.0]
    fov 275.0
    samples 3034
  )
  # generic model properties
  color "black"
  size [ 0.84 2.1 0.42 ]
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [ 0.0 0.0 0.0 0.0 0.0 0.0 ]
  
  size [ 2.2 2.2 1.5 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "blue"
  
  kinect(pose [ -0.08 0.0 -0.088 0.0 ])
)
