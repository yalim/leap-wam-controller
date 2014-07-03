; Auto-generated. Do not edit!


(cl:in-package leap_wam_controller-srv)


;//! \htmlinclude LeapGoalPose-request.msg.html

(cl:defclass <LeapGoalPose-request> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass LeapGoalPose-request (<LeapGoalPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LeapGoalPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LeapGoalPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name leap_wam_controller-srv:<LeapGoalPose-request> is deprecated: use leap_wam_controller-srv:LeapGoalPose-request instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <LeapGoalPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leap_wam_controller-srv:pose-val is deprecated.  Use leap_wam_controller-srv:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LeapGoalPose-request>) ostream)
  "Serializes a message object of type '<LeapGoalPose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LeapGoalPose-request>) istream)
  "Deserializes a message object of type '<LeapGoalPose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LeapGoalPose-request>)))
  "Returns string type for a service object of type '<LeapGoalPose-request>"
  "leap_wam_controller/LeapGoalPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LeapGoalPose-request)))
  "Returns string type for a service object of type 'LeapGoalPose-request"
  "leap_wam_controller/LeapGoalPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LeapGoalPose-request>)))
  "Returns md5sum for a message object of type '<LeapGoalPose-request>"
  "1d3c815fa4fd2c21ffd36bbca1e530bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LeapGoalPose-request)))
  "Returns md5sum for a message object of type 'LeapGoalPose-request"
  "1d3c815fa4fd2c21ffd36bbca1e530bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LeapGoalPose-request>)))
  "Returns full string definition for message of type '<LeapGoalPose-request>"
  (cl:format cl:nil "geometry_msgs/Pose pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LeapGoalPose-request)))
  "Returns full string definition for message of type 'LeapGoalPose-request"
  (cl:format cl:nil "geometry_msgs/Pose pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LeapGoalPose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LeapGoalPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LeapGoalPose-request
    (cl:cons ':pose (pose msg))
))
;//! \htmlinclude LeapGoalPose-response.msg.html

(cl:defclass <LeapGoalPose-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LeapGoalPose-response (<LeapGoalPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LeapGoalPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LeapGoalPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name leap_wam_controller-srv:<LeapGoalPose-response> is deprecated: use leap_wam_controller-srv:LeapGoalPose-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <LeapGoalPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leap_wam_controller-srv:result-val is deprecated.  Use leap_wam_controller-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LeapGoalPose-response>) ostream)
  "Serializes a message object of type '<LeapGoalPose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LeapGoalPose-response>) istream)
  "Deserializes a message object of type '<LeapGoalPose-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LeapGoalPose-response>)))
  "Returns string type for a service object of type '<LeapGoalPose-response>"
  "leap_wam_controller/LeapGoalPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LeapGoalPose-response)))
  "Returns string type for a service object of type 'LeapGoalPose-response"
  "leap_wam_controller/LeapGoalPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LeapGoalPose-response>)))
  "Returns md5sum for a message object of type '<LeapGoalPose-response>"
  "1d3c815fa4fd2c21ffd36bbca1e530bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LeapGoalPose-response)))
  "Returns md5sum for a message object of type 'LeapGoalPose-response"
  "1d3c815fa4fd2c21ffd36bbca1e530bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LeapGoalPose-response>)))
  "Returns full string definition for message of type '<LeapGoalPose-response>"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LeapGoalPose-response)))
  "Returns full string definition for message of type 'LeapGoalPose-response"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LeapGoalPose-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LeapGoalPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LeapGoalPose-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LeapGoalPose)))
  'LeapGoalPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LeapGoalPose)))
  'LeapGoalPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LeapGoalPose)))
  "Returns string type for a service object of type '<LeapGoalPose>"
  "leap_wam_controller/LeapGoalPose")