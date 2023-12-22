;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::RemapLocalGoalList)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'RemapLocalGoalList (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::REMAPLOCALGOALLIST")
  (make-package "TRAJ_UTILS::REMAPLOCALGOALLIST"))

(in-package "ROS")
;;//! \htmlinclude RemapLocalGoalList.msg.html


(defclass traj_utils::RemapLocalGoalList
  :super ros::object
  :slots (_guard_drone_id _assignment _remap_lg_pos_x _remap_lg_pos_y _remap_lg_pos_z _remap_lg_vel_x _remap_lg_vel_y _remap_lg_vel_z ))

(defmethod traj_utils::RemapLocalGoalList
  (:init
   (&key
    ((:guard_drone_id __guard_drone_id) 0)
    ((:assignment __assignment) (make-array 0 :initial-element 0 :element-type :integer))
    ((:remap_lg_pos_x __remap_lg_pos_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:remap_lg_pos_y __remap_lg_pos_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:remap_lg_pos_z __remap_lg_pos_z) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:remap_lg_vel_x __remap_lg_vel_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:remap_lg_vel_y __remap_lg_vel_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:remap_lg_vel_z __remap_lg_vel_z) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _guard_drone_id (round __guard_drone_id))
   (setq _assignment __assignment)
   (setq _remap_lg_pos_x __remap_lg_pos_x)
   (setq _remap_lg_pos_y __remap_lg_pos_y)
   (setq _remap_lg_pos_z __remap_lg_pos_z)
   (setq _remap_lg_vel_x __remap_lg_vel_x)
   (setq _remap_lg_vel_y __remap_lg_vel_y)
   (setq _remap_lg_vel_z __remap_lg_vel_z)
   self)
  (:guard_drone_id
   (&optional __guard_drone_id)
   (if __guard_drone_id (setq _guard_drone_id __guard_drone_id)) _guard_drone_id)
  (:assignment
   (&optional __assignment)
   (if __assignment (setq _assignment __assignment)) _assignment)
  (:remap_lg_pos_x
   (&optional __remap_lg_pos_x)
   (if __remap_lg_pos_x (setq _remap_lg_pos_x __remap_lg_pos_x)) _remap_lg_pos_x)
  (:remap_lg_pos_y
   (&optional __remap_lg_pos_y)
   (if __remap_lg_pos_y (setq _remap_lg_pos_y __remap_lg_pos_y)) _remap_lg_pos_y)
  (:remap_lg_pos_z
   (&optional __remap_lg_pos_z)
   (if __remap_lg_pos_z (setq _remap_lg_pos_z __remap_lg_pos_z)) _remap_lg_pos_z)
  (:remap_lg_vel_x
   (&optional __remap_lg_vel_x)
   (if __remap_lg_vel_x (setq _remap_lg_vel_x __remap_lg_vel_x)) _remap_lg_vel_x)
  (:remap_lg_vel_y
   (&optional __remap_lg_vel_y)
   (if __remap_lg_vel_y (setq _remap_lg_vel_y __remap_lg_vel_y)) _remap_lg_vel_y)
  (:remap_lg_vel_z
   (&optional __remap_lg_vel_z)
   (if __remap_lg_vel_z (setq _remap_lg_vel_z __remap_lg_vel_z)) _remap_lg_vel_z)
  (:serialization-length
   ()
   (+
    ;; int16 _guard_drone_id
    2
    ;; uint32[] _assignment
    (* 4    (length _assignment)) 4
    ;; float32[] _remap_lg_pos_x
    (* 4    (length _remap_lg_pos_x)) 4
    ;; float32[] _remap_lg_pos_y
    (* 4    (length _remap_lg_pos_y)) 4
    ;; float32[] _remap_lg_pos_z
    (* 4    (length _remap_lg_pos_z)) 4
    ;; float32[] _remap_lg_vel_x
    (* 4    (length _remap_lg_vel_x)) 4
    ;; float32[] _remap_lg_vel_y
    (* 4    (length _remap_lg_vel_y)) 4
    ;; float32[] _remap_lg_vel_z
    (* 4    (length _remap_lg_vel_z)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _guard_drone_id
       (write-word _guard_drone_id s)
     ;; uint32[] _assignment
     (write-long (length _assignment) s)
     (dotimes (i (length _assignment))
       (write-long (elt _assignment i) s)
       )
     ;; float32[] _remap_lg_pos_x
     (write-long (length _remap_lg_pos_x) s)
     (dotimes (i (length _remap_lg_pos_x))
       (sys::poke (elt _remap_lg_pos_x i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _remap_lg_pos_y
     (write-long (length _remap_lg_pos_y) s)
     (dotimes (i (length _remap_lg_pos_y))
       (sys::poke (elt _remap_lg_pos_y i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _remap_lg_pos_z
     (write-long (length _remap_lg_pos_z) s)
     (dotimes (i (length _remap_lg_pos_z))
       (sys::poke (elt _remap_lg_pos_z i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _remap_lg_vel_x
     (write-long (length _remap_lg_vel_x) s)
     (dotimes (i (length _remap_lg_vel_x))
       (sys::poke (elt _remap_lg_vel_x i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _remap_lg_vel_y
     (write-long (length _remap_lg_vel_y) s)
     (dotimes (i (length _remap_lg_vel_y))
       (sys::poke (elt _remap_lg_vel_y i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _remap_lg_vel_z
     (write-long (length _remap_lg_vel_z) s)
     (dotimes (i (length _remap_lg_vel_z))
       (sys::poke (elt _remap_lg_vel_z i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _guard_drone_id
     (setq _guard_drone_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint32[] _assignment
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _assignment (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _assignment i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32[] _remap_lg_pos_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _remap_lg_pos_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _remap_lg_pos_x i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _remap_lg_pos_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _remap_lg_pos_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _remap_lg_pos_y i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _remap_lg_pos_z
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _remap_lg_pos_z (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _remap_lg_pos_z i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _remap_lg_vel_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _remap_lg_vel_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _remap_lg_vel_x i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _remap_lg_vel_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _remap_lg_vel_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _remap_lg_vel_y i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _remap_lg_vel_z
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _remap_lg_vel_z (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _remap_lg_vel_z i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get traj_utils::RemapLocalGoalList :md5sum-) "94d1962f185de6a5a18e3c7557706328")
(setf (get traj_utils::RemapLocalGoalList :datatype-) "traj_utils/RemapLocalGoalList")
(setf (get traj_utils::RemapLocalGoalList :definition-)
      "int16 guard_drone_id

uint32[] assignment

float32[] remap_lg_pos_x
float32[] remap_lg_pos_y
float32[] remap_lg_pos_z

float32[] remap_lg_vel_x
float32[] remap_lg_vel_y
float32[] remap_lg_vel_z
")



(provide :traj_utils/RemapLocalGoalList "94d1962f185de6a5a18e3c7557706328")


