# Full-Body-Inverse-Kinematics
Inverse Kinematics Method in RBX.Lua

## Foreword
This file entails a personal side project that I've been working on using the Roblox game engine. I implemented a customized inverse kinematics method that is flexible for multiple situations, this example is slightly modified for my specific needs but can easily be translated into other languages. It has an easily customizable table of constraints and includes support for a 10 joint character model.

## Algorithm
The algorithm finds the necessary Euler angles required in each plane (x,y,z) by using the geometric definition for arctangent. The elbow joint is calculated using the Law of Cosines. Once the angles are found, constraints are applied. In addition, I've added a method to achieve more realistic motion by using the ratio between the yaw and roll angles; as the yaw angle approaches its constraint, the roll angle is "absorbed" into the pitch angle. This means that the roll and pitch angle are inversely related.
