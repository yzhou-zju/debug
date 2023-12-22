;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::LocalGoal)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'LocalGoal (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::LOCALGOAL")
  (make-package "TRAJ_UTILS::LOCALGOAL"))

(in-package "ROS")
;;//! \htmlinclude LocalGoal.msg.html


(defclass traj_utils::LocalGoal
  :super ros::object
  :slots (_drone_id _global_traj_id _lg_pos_x _lg_pos_y _lg_pos_z _lg_vel_x _lg_vel_y _lg_vel_z ))

(defmethod traj_utils::LocalGoal
  (:init
   (&key
    ((:drone_id __drone_id) 0)
    ((:global_traj_id __global_traj_id) 0)
    ((:lg_pos_x __lg_pos_x) 0.0)
    ((:lg_pos_y __lg_pos_y) 0.0)
    ((:lg_pos_z __lg_pos_z) 0.0)
    ((:lg_vel_x __lg_vel_x) 0.0)
    ((:lg_vel_y __lg_vel_y) 0.0)
    ((:lg_vel_z __lg_vel_z) 0.0)
    )
   (send-super :init)
   (setq _drone_id (round __drone_id))
   (setq _global_traj_id (round __global_traj_id))
   (setq _lg_pos_x (float __lg_pos_x))
   (setq _lg_pos_y (float __lg_pos_y))
   (setq _lg_pos_z (float __lg_pos_z))
   (setq _lg_vel_x (float __lg_vel_x))
   (setq _lg_vel_y (float __lg_vel_y))
   (setq _lg_vel_z (float __lg_vel_z))
   self)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:global_traj_id
   (&optional __global_traj_id)
   (if __global_traj_id (setq _global_traj_id __global_traj_id)) _global_traj_id)
  (:lg_pos_x
   (&optional __lg_pos_x)
   (if __lg_pos_x (setq _lg_pos_x __lg_pos_x)) _lg_pos_x)
  (:lg_pos_y
   (&optional __lg_pos_y)
   (if __lg_pos_y (setq _lg_pos_y __lg_pos_y)) _lg_pos_y)
  (:lg_pos_z
   (&optional __lg_pos_z)
   (if __lg_pos_z (setq _lg_pos_z __lg_pos_z)) _lg_pos_z)
  (:lg_vel_x
   (&optional __lg_vel_x)
   (if __lg_vel_x (setq _lg_vel_x __lg_vel_x)) _lg_vel_x)
  (:lg_vel_y
   (&optional __lg_vel_y)
   (if __lg_vel_y (setq _lg_vel_y __lg_vel_y)) _lg_vel_y)
  (:lg_vel_z
   (&optional __lg_vel_z)
   (if __lg_vel_z (setq _lg_vel_z __lg_vel_z)) _lg_vel_z)
  (:serialization-length
   ()
   (+
    ;; int16 _drone_id
    2
    ;; int16 _global_traj_id
    2
    ;; float32 _lg_pos_x
    4
    ;; float32 _lg_pos_y
    4
    ;; float32 _lg_pos_z
    4
    ;; float32 _lg_vel_x
    4
    ;; float32 _lg_vel_y
    4
    ;; float32 _lg_vel_z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _drone_id
       (write-word _drone_id s)
     ;; int16 _global_traj_id
       (write-word _global_traj_id s)
     ;; float32 _lg_pos_x
       (sys::poke _lg_pos_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lg_pos_y
       (sys::poke _lg_pos_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lg_pos_z
       (sys::poke _lg_pos_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lg_vel_x
       (sys::poke _lg_vel_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lg_vel_y
       (sys::poke _lg_vel_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lg_vel_z
       (sys::poke _lg_vel_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _drone_id
     (setq _drone_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _global_traj_id
     (setq _global_traj_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _lg_pos_x
     (setq _lg_pos_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lg_pos_y
     (setq _lg_pos_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lg_pos_z
     (setq _lg_pos_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lg_vel_x
     (setq _lg_vel_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lg_vel_y
     (setq _lg_vel_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lg_vel_z
     (setq _lg_vel_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get traj_utils::LocalGoal :md5sum-) "a8f0ef3ec042e7d8948442e97ca38913")
(setf (get traj_utils::LocalGoal :datatype-) "traj_utils/LocalGoal")
(setf (get traj_utils::LocalGoal :definition-)
      "int16 drone_id
int16 global_traj_id

float32 lg_pos_x
float32 lg_pos_y
float32 lg_pos_z

float32 lg_vel_x
float32 lg_vel_y
float32 lg_vel_z

")



(provide :traj_utils/LocalGoal "a8f0ef3ec042e7d8948442e97ca38913")


