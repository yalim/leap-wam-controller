
(cl:in-package :asdf)

(defsystem "leap_wam_controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "LeapGoalPose" :depends-on ("_package_LeapGoalPose"))
    (:file "_package_LeapGoalPose" :depends-on ("_package"))
  ))