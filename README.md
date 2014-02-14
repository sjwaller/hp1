hp1
===

HP1 Robot - ROS (Hydro / Catkin)

Website: http://sjwaller.com/robotics/hp1

Links
---
* http://free-dee.org/3d-head-tracking-in-video/
* https://github.com/ashokzg/ecp/blob/master/src/CamShiftDestTracker.cpp

Nodes
---

__HP1__
* subscribe:input
* subscribe:poi
* do:ai(model)
  * do:state(model)
  * do:behaviour(model)
* publish:model

__Movement__
* subscribe:model
* do:ik
* publish:motors

__Vision__
* subscribe:camera/image_raw
* subscribe:camera/camera_info
* publish:hp1/vision_roi
* publish:hp1/vision_angle
* publish:hp1/vision_state

__Control__
* subscribe:joy
* publish:input
