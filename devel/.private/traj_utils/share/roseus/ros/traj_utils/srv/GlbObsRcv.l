;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::GlbObsRcv)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'GlbObsRcv (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::GLBOBSRCV")
  (make-package "TRAJ_UTILS::GLBOBSRCV"))
(unless (find-package "TRAJ_UTILS::GLBOBSRCVREQUEST")
  (make-package "TRAJ_UTILS::GLBOBSRCVREQUEST"))
(unless (find-package "TRAJ_UTILS::GLBOBSRCVRESPONSE")
  (make-package "TRAJ_UTILS::GLBOBSRCVRESPONSE"))

(in-package "ROS")





(defclass traj_utils::GlbObsRcvRequest
  :super ros::object
  :slots (_req ))

(defmethod traj_utils::GlbObsRcvRequest
  (:init
   (&key
    ((:req __req) nil)
    )
   (send-super :init)
   (setq _req __req)
   self)
  (:req
   (&optional (__req :null))
   (if (not (eq __req :null)) (setq _req __req)) _req)
  (:serialization-length
   ()
   (+
    ;; bool _req
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _req
       (if _req (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _req
     (setq _req (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass traj_utils::GlbObsRcvResponse
  :super ros::object
  :slots (_result ))

(defmethod traj_utils::GlbObsRcvResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional (__result :null))
   (if (not (eq __result :null)) (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass traj_utils::GlbObsRcv
  :super ros::object
  :slots ())

(setf (get traj_utils::GlbObsRcv :md5sum-) "7f352b44c251978f0843cd60887550cf")
(setf (get traj_utils::GlbObsRcv :datatype-) "traj_utils/GlbObsRcv")
(setf (get traj_utils::GlbObsRcv :request) traj_utils::GlbObsRcvRequest)
(setf (get traj_utils::GlbObsRcv :response) traj_utils::GlbObsRcvResponse)

(defmethod traj_utils::GlbObsRcvRequest
  (:response () (instance traj_utils::GlbObsRcvResponse :init)))

(setf (get traj_utils::GlbObsRcvRequest :md5sum-) "7f352b44c251978f0843cd60887550cf")
(setf (get traj_utils::GlbObsRcvRequest :datatype-) "traj_utils/GlbObsRcvRequest")
(setf (get traj_utils::GlbObsRcvRequest :definition-)
      "bool req
---
bool result

")

(setf (get traj_utils::GlbObsRcvResponse :md5sum-) "7f352b44c251978f0843cd60887550cf")
(setf (get traj_utils::GlbObsRcvResponse :datatype-) "traj_utils/GlbObsRcvResponse")
(setf (get traj_utils::GlbObsRcvResponse :definition-)
      "bool req
---
bool result

")



(provide :traj_utils/GlbObsRcv "7f352b44c251978f0843cd60887550cf")


