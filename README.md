hp1
===

HP1 Robot - ROS (Hydro / Catkin)

Website: http://sjwaller.com/robotics/hp1


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
* subscribe:camera
* publish:poi

__Control__
* subscribe:joy
* publish:input
