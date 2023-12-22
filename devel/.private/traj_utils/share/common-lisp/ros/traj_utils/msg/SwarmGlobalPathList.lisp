; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude SwarmGlobalPathList.msg.html

(cl:defclass <SwarmGlobalPathList> (roslisp-msg-protocol:ros-message)
  ((guard_drone_id
    :reader guard_drone_id
    :initarg :guard_drone_id
    :type cl:fixnum
    :initform 0)
   (path_num
    :reader path_num
    :initarg :path_num
    :type cl:fixnum
    :initform 0)
   (swarm_global_path_x
    :reader swarm_global_path_x
    :initarg :swarm_global_path_x
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (swarm_global_path_y
    :reader swarm_global_path_y
    :initarg :swarm_global_path_y
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (swarm_global_path_z
    :reader swarm_global_path_z
    :initarg :swarm_global_path_z
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (swarm_global_path_scale
    :reader swarm_global_path_scale
    :initarg :swarm_global_path_scale
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SwarmGlobalPathList (<SwarmGlobalPathList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwarmGlobalPathList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwarmGlobalPathList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<SwarmGlobalPathList> is deprecated: use traj_utils-msg:SwarmGlobalPathList instead.")))

(cl:ensure-generic-function 'guard_drone_id-val :lambda-list '(m))
(cl:defmethod guard_drone_id-val ((m <SwarmGlobalPathList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:guard_drone_id-val is deprecated.  Use traj_utils-msg:guard_drone_id instead.")
  (guard_drone_id m))

(cl:ensure-generic-function 'path_num-val :lambda-list '(m))
(cl:defmethod path_num-val ((m <SwarmGlobalPathList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:path_num-val is deprecated.  Use traj_utils-msg:path_num instead.")
  (path_num m))

(cl:ensure-generic-function 'swarm_global_path_x-val :lambda-list '(m))
(cl:defmethod swarm_global_path_x-val ((m <SwarmGlobalPathList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:swarm_global_path_x-val is deprecated.  Use traj_utils-msg:swarm_global_path_x instead.")
  (swarm_global_path_x m))

(cl:ensure-generic-function 'swarm_global_path_y-val :lambda-list '(m))
(cl:defmethod swarm_global_path_y-val ((m <SwarmGlobalPathList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:swarm_global_path_y-val is deprecated.  Use traj_utils-msg:swarm_global_path_y instead.")
  (swarm_global_path_y m))

(cl:ensure-generic-function 'swarm_global_path_z-val :lambda-list '(m))
(cl:defmethod swarm_global_path_z-val ((m <SwarmGlobalPathList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:swarm_global_path_z-val is deprecated.  Use traj_utils-msg:swarm_global_path_z instead.")
  (swarm_global_path_z m))

(cl:ensure-generic-function 'swarm_global_path_scale-val :lambda-list '(m))
(cl:defmethod swarm_global_path_scale-val ((m <SwarmGlobalPathList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:swarm_global_path_scale-val is deprecated.  Use traj_utils-msg:swarm_global_path_scale instead.")
  (swarm_global_path_scale m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwarmGlobalPathList>) ostream)
  "Serializes a message object of type '<SwarmGlobalPathList>"
  (cl:let* ((signed (cl:slot-value msg 'guard_drone_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'path_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'swarm_global_path_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'swarm_global_path_x))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'swarm_global_path_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'swarm_global_path_y))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'swarm_global_path_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'swarm_global_path_z))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'swarm_global_path_scale))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'swarm_global_path_scale))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwarmGlobalPathList>) istream)
  "Deserializes a message object of type '<SwarmGlobalPathList>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'guard_drone_id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'path_num) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'swarm_global_path_x) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'swarm_global_path_x)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'swarm_global_path_y) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'swarm_global_path_y)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'swarm_global_path_z) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'swarm_global_path_z)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'swarm_global_path_scale) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'swarm_global_path_scale)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwarmGlobalPathList>)))
  "Returns string type for a message object of type '<SwarmGlobalPathList>"
  "traj_utils/SwarmGlobalPathList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwarmGlobalPathList)))
  "Returns string type for a message object of type 'SwarmGlobalPathList"
  "traj_utils/SwarmGlobalPathList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwarmGlobalPathList>)))
  "Returns md5sum for a message object of type '<SwarmGlobalPathList>"
  "015139dac170a0f9242e5a66432e43ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwarmGlobalPathList)))
  "Returns md5sum for a message object of type 'SwarmGlobalPathList"
  "015139dac170a0f9242e5a66432e43ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwarmGlobalPathList>)))
  "Returns full string definition for message of type '<SwarmGlobalPathList>"
  (cl:format cl:nil "int16 guard_drone_id~%int16 path_num~%~%float32[] swarm_global_path_x~%float32[] swarm_global_path_y~%float32[] swarm_global_path_z~%float32[] swarm_global_path_scale~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwarmGlobalPathList)))
  "Returns full string definition for message of type 'SwarmGlobalPathList"
  (cl:format cl:nil "int16 guard_drone_id~%int16 path_num~%~%float32[] swarm_global_path_x~%float32[] swarm_global_path_y~%float32[] swarm_global_path_z~%float32[] swarm_global_path_scale~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwarmGlobalPathList>))
  (cl:+ 0
     2
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'swarm_global_path_x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'swarm_global_path_y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'swarm_global_path_z) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'swarm_global_path_scale) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwarmGlobalPathList>))
  "Converts a ROS message object to a list"
  (cl:list 'SwarmGlobalPathList
    (cl:cons ':guard_drone_id (guard_drone_id msg))
    (cl:cons ':path_num (path_num msg))
    (cl:cons ':swarm_global_path_x (swarm_global_path_x msg))
    (cl:cons ':swarm_global_path_y (swarm_global_path_y msg))
    (cl:cons ':swarm_global_path_z (swarm_global_path_z msg))
    (cl:cons ':swarm_global_path_scale (swarm_global_path_scale msg))
))
