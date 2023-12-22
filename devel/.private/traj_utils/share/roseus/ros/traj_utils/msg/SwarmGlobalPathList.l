;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::SwarmGlobalPathList)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'SwarmGlobalPathList (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::SWARMGLOBALPATHLIST")
  (make-package "TRAJ_UTILS::SWARMGLOBALPATHLIST"))

(in-package "ROS")
;;//! \htmlinclude SwarmGlobalPathList.msg.html


(defclass traj_utils::SwarmGlobalPathList
  :super ros::object
  :slots (_guard_drone_id _path_num _swarm_global_path_x _swarm_global_path_y _swarm_global_path_z _swarm_global_path_scale ))

(defmethod traj_utils::SwarmGlobalPathList
  (:init
   (&key
    ((:guard_drone_id __guard_drone_id) 0)
    ((:path_num __path_num) 0)
    ((:swarm_global_path_x __swarm_global_path_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:swarm_global_path_y __swarm_global_path_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:swarm_global_path_z __swarm_global_path_z) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:swarm_global_path_scale __swarm_global_path_scale) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _guard_drone_id (round __guard_drone_id))
   (setq _path_num (round __path_num))
   (setq _swarm_global_path_x __swarm_global_path_x)
   (setq _swarm_global_path_y __swarm_global_path_y)
   (setq _swarm_global_path_z __swarm_global_path_z)
   (setq _swarm_global_path_scale __swarm_global_path_scale)
   self)
  (:guard_drone_id
   (&optional __guard_drone_id)
   (if __guard_drone_id (setq _guard_drone_id __guard_drone_id)) _guard_drone_id)
  (:path_num
   (&optional __path_num)
   (if __path_num (setq _path_num __path_num)) _path_num)
  (:swarm_global_path_x
   (&optional __swarm_global_path_x)
   (if __swarm_global_path_x (setq _swarm_global_path_x __swarm_global_path_x)) _swarm_global_path_x)
  (:swarm_global_path_y
   (&optional __swarm_global_path_y)
   (if __swarm_global_path_y (setq _swarm_global_path_y __swarm_global_path_y)) _swarm_global_path_y)
  (:swarm_global_path_z
   (&optional __swarm_global_path_z)
   (if __swarm_global_path_z (setq _swarm_global_path_z __swarm_global_path_z)) _swarm_global_path_z)
  (:swarm_global_path_scale
   (&optional __swarm_global_path_scale)
   (if __swarm_global_path_scale (setq _swarm_global_path_scale __swarm_global_path_scale)) _swarm_global_path_scale)
  (:serialization-length
   ()
   (+
    ;; int16 _guard_drone_id
    2
    ;; int16 _path_num
    2
    ;; float32[] _swarm_global_path_x
    (* 4    (length _swarm_global_path_x)) 4
    ;; float32[] _swarm_global_path_y
    (* 4    (length _swarm_global_path_y)) 4
    ;; float32[] _swarm_global_path_z
    (* 4    (length _swarm_global_path_z)) 4
    ;; float32[] _swarm_global_path_scale
    (* 4    (length _swarm_global_path_scale)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _guard_drone_id
       (write-word _guard_drone_id s)
     ;; int16 _path_num
       (write-word _path_num s)
     ;; float32[] _swarm_global_path_x
     (write-long (length _swarm_global_path_x) s)
     (dotimes (i (length _swarm_global_path_x))
       (sys::poke (elt _swarm_global_path_x i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _swarm_global_path_y
     (write-long (length _swarm_global_path_y) s)
     (dotimes (i (length _swarm_global_path_y))
       (sys::poke (elt _swarm_global_path_y i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _swarm_global_path_z
     (write-long (length _swarm_global_path_z) s)
     (dotimes (i (length _swarm_global_path_z))
       (sys::poke (elt _swarm_global_path_z i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _swarm_global_path_scale
     (write-long (length _swarm_global_path_scale) s)
     (dotimes (i (length _swarm_global_path_scale))
       (sys::poke (elt _swarm_global_path_scale i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _guard_drone_id
     (setq _guard_drone_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _path_num
     (setq _path_num (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32[] _swarm_global_path_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _swarm_global_path_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _swarm_global_path_x i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _swarm_global_path_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _swarm_global_path_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _swarm_global_path_y i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _swarm_global_path_z
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _swarm_global_path_z (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _swarm_global_path_z i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _swarm_global_path_scale
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _swarm_global_path_scale (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _swarm_global_path_scale i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get traj_utils::SwarmGlobalPathList :md5sum-) "015139dac170a0f9242e5a66432e43ee")
(setf (get traj_utils::SwarmGlobalPathList :datatype-) "traj_utils/SwarmGlobalPathList")
(setf (get traj_utils::SwarmGlobalPathList :definition-)
      "int16 guard_drone_id
int16 path_num

float32[] swarm_global_path_x
float32[] swarm_global_path_y
float32[] swarm_global_path_z
float32[] swarm_global_path_scale

")



(provide :traj_utils/SwarmGlobalPathList "015139dac170a0f9242e5a66432e43ee")


