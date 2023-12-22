; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude LocalGoal.msg.html

(cl:defclass <LocalGoal> (roslisp-msg-protocol:ros-message)
  ((drone_id
    :reader drone_id
    :initarg :drone_id
    :type cl:fixnum
    :initform 0)
   (global_traj_id
    :reader global_traj_id
    :initarg :global_traj_id
    :type cl:fixnum
    :initform 0)
   (lg_pos_x
    :reader lg_pos_x
    :initarg :lg_pos_x
    :type cl:float
    :initform 0.0)
   (lg_pos_y
    :reader lg_pos_y
    :initarg :lg_pos_y
    :type cl:float
    :initform 0.0)
   (lg_pos_z
    :reader lg_pos_z
    :initarg :lg_pos_z
    :type cl:float
    :initform 0.0)
   (lg_vel_x
    :reader lg_vel_x
    :initarg :lg_vel_x
    :type cl:float
    :initform 0.0)
   (lg_vel_y
    :reader lg_vel_y
    :initarg :lg_vel_y
    :type cl:float
    :initform 0.0)
   (lg_vel_z
    :reader lg_vel_z
    :initarg :lg_vel_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass LocalGoal (<LocalGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LocalGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LocalGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<LocalGoal> is deprecated: use traj_utils-msg:LocalGoal instead.")))

(cl:ensure-generic-function 'drone_id-val :lambda-list '(m))
(cl:defmethod drone_id-val ((m <LocalGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:drone_id-val is deprecated.  Use traj_utils-msg:drone_id instead.")
  (drone_id m))

(cl:ensure-generic-function 'global_traj_id-val :lambda-list '(m))
(cl:defmethod global_traj_id-val ((m <LocalGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:global_traj_id-val is deprecated.  Use traj_utils-msg:global_traj_id instead.")
  (global_traj_id m))

(cl:ensure-generic-function 'lg_pos_x-val :lambda-list '(m))
(cl:defmethod lg_pos_x-val ((m <LocalGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:lg_pos_x-val is deprecated.  Use traj_utils-msg:lg_pos_x instead.")
  (lg_pos_x m))

(cl:ensure-generic-function 'lg_pos_y-val :lambda-list '(m))
(cl:defmethod lg_pos_y-val ((m <LocalGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:lg_pos_y-val is deprecated.  Use traj_utils-msg:lg_pos_y instead.")
  (lg_pos_y m))

(cl:ensure-generic-function 'lg_pos_z-val :lambda-list '(m))
(cl:defmethod lg_pos_z-val ((m <LocalGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:lg_pos_z-val is deprecated.  Use traj_utils-msg:lg_pos_z instead.")
  (lg_pos_z m))

(cl:ensure-generic-function 'lg_vel_x-val :lambda-list '(m))
(cl:defmethod lg_vel_x-val ((m <LocalGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:lg_vel_x-val is deprecated.  Use traj_utils-msg:lg_vel_x instead.")
  (lg_vel_x m))

(cl:ensure-generic-function 'lg_vel_y-val :lambda-list '(m))
(cl:defmethod lg_vel_y-val ((m <LocalGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:lg_vel_y-val is deprecated.  Use traj_utils-msg:lg_vel_y instead.")
  (lg_vel_y m))

(cl:ensure-generic-function 'lg_vel_z-val :lambda-list '(m))
(cl:defmethod lg_vel_z-val ((m <LocalGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:lg_vel_z-val is deprecated.  Use traj_utils-msg:lg_vel_z instead.")
  (lg_vel_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LocalGoal>) ostream)
  "Serializes a message object of type '<LocalGoal>"
  (cl:let* ((signed (cl:slot-value msg 'drone_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'global_traj_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lg_pos_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lg_pos_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lg_pos_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lg_vel_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lg_vel_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lg_vel_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LocalGoal>) istream)
  "Deserializes a message object of type '<LocalGoal>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drone_id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'global_traj_id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lg_pos_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lg_pos_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lg_pos_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lg_vel_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lg_vel_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lg_vel_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LocalGoal>)))
  "Returns string type for a message object of type '<LocalGoal>"
  "traj_utils/LocalGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocalGoal)))
  "Returns string type for a message object of type 'LocalGoal"
  "traj_utils/LocalGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LocalGoal>)))
  "Returns md5sum for a message object of type '<LocalGoal>"
  "a8f0ef3ec042e7d8948442e97ca38913")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LocalGoal)))
  "Returns md5sum for a message object of type 'LocalGoal"
  "a8f0ef3ec042e7d8948442e97ca38913")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LocalGoal>)))
  "Returns full string definition for message of type '<LocalGoal>"
  (cl:format cl:nil "int16 drone_id~%int16 global_traj_id~%~%float32 lg_pos_x~%float32 lg_pos_y~%float32 lg_pos_z~%~%float32 lg_vel_x~%float32 lg_vel_y~%float32 lg_vel_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LocalGoal)))
  "Returns full string definition for message of type 'LocalGoal"
  (cl:format cl:nil "int16 drone_id~%int16 global_traj_id~%~%float32 lg_pos_x~%float32 lg_pos_y~%float32 lg_pos_z~%~%float32 lg_vel_x~%float32 lg_vel_y~%float32 lg_vel_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LocalGoal>))
  (cl:+ 0
     2
     2
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LocalGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'LocalGoal
    (cl:cons ':drone_id (drone_id msg))
    (cl:cons ':global_traj_id (global_traj_id msg))
    (cl:cons ':lg_pos_x (lg_pos_x msg))
    (cl:cons ':lg_pos_y (lg_pos_y msg))
    (cl:cons ':lg_pos_z (lg_pos_z msg))
    (cl:cons ':lg_vel_x (lg_vel_x msg))
    (cl:cons ':lg_vel_y (lg_vel_y msg))
    (cl:cons ':lg_vel_z (lg_vel_z msg))
))
