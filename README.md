# Stereo-Image-Proessing
Calculates the depth of the scene from pair of stereo images based on Popular Block matching technique.

The project involves following steps:

1) Subscribes to pair of Stereo images from Camera topics
2) Rectify and undistort the images.
3) Time Synchronize the pair of images
4) Disparity calculations based on Block Macthing and Semi Global block mathcing algorithm.
5) Publish the pointcloucloud map created as ROS Topic.
