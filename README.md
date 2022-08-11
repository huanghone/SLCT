# Pixel-wise weighted region-based 3D object tracking using contour constraints

Region-based methods are currently achieving state-of-the-art performance for monocular 3D object tracking. However, they are still prone to fail in case of partial occlusions and ambiguous colors. We propose a novel region-based method to tackle these problems. The key idea is to derive a pixel-wise weighted region-based cost function using contour constraints.We verify the effectiveness and efficiency of our method on challenging public datasets. Experiments demonstrate that our method outperforms the recent state-of-the-art region-based methods in complex scenarios, especially in the presence of partial occlusions and ambiguous colors.

### Preview Video

[![PG'20 supplementary video.](https://img.youtube.com/vi/IntD-laemgU/0.jpg)](https://www.youtube.com/watch?v=IntD-laemgU)


### Related Papers

* **Pixel-Wise Weighted Region-Based 3D Object Tracking using Contour Constraints**
*Hong Huang, Fan Zhong, Xueying Qin*, TVCG '21


# How To Use

The general usage of the algorithm is demonstrated in a small example command line application provided in `run_on_video.cc`. Here the pose of a single 3D model is refined with respect to a given example image. The extension to actual pose tracking and using multiple objects should be straight foward based on this example. Simply replace the example image with the live feed from a camera or a video and add your own 3D models instead.

For the best performance when using your own 3D models, please **ensure that each 3D model consists of a maximum of around 4000 - 7000 vertices and is equally sampled across the visible surface**. This can be enforced by using a 3D mesh manipulation software such as MeshLab (http://www.meshlab.net/) or OpenFlipper (https://www.openflipper.org/).


# Dataset

To test the algorithm you can for example use the corresponding dataset available for download at: https://github.com/huanghone/RBOTE


# License

SLCT is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html.
