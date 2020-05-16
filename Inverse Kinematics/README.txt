Name: Joe Yu-Ho Chang
OS: Visual Studio 2017

Extra Credits:
1.Dual Quaternion Skinning 
  -> skinning.cpp line 102 - 171
  (Default is Linear Blend Skinning)
  -> To switch between DQS/LBS, press 'z'
  (Recommendation: At start choose to use LBS/DQS and don't switch between them)
  -> Downloaded Vega and used the quaternion library.

2.pseudoinverse IK
  -> IK.cpp line 248 - 268
  (Default is Damped Least Square)
  -> To switch between pseudoinverse/DLS, press 'c'
  (Recommendation: At start choose to use pseudoinverse/DLS and don't switch between them)
  -> Don't move the handle extremely to prevent blow up

Note:
Release Mode performs better for FPS
To activate/deactivate screenshot, press 'x'. The pictures will be saved in the working (model) directory. (Default is armadillo folder)