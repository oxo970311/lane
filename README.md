Lane Detection (OpenCV-based)

Perspective Transform (Bird’s-Eye View)
Applies a homography transform to obtain a top-down view of the road,
simplifying lane geometry and reducing perspective distortion.

Color & Gradient Thresholding
Uses color space filtering and edge detection to isolate lane markings
from the road surface under varying lighting conditions.

Histogram-based Lane Base Detection
Computes a horizontal pixel intensity histogram to estimate initial
left and right lane positions.

Sliding Window Search
Tracks lane pixels vertically using adaptive sliding windows,
enabling robust lane line extraction even with partial occlusions.

Polynomial Curve Fitting
Fits a second-order polynomial to detected lane pixels
for smooth lane modeling and centerline estimation.

Lane Center & Steering Angle Estimation
Calculates lateral offset and heading angle relative to the lane center
to support steering control logic.
