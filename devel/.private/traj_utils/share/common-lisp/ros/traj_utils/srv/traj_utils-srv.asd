
(cl:in-package :asdf)

(defsystem "traj_utils-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GlbObsRcv" :depends-on ("_package_GlbObsRcv"))
    (:file "_package_GlbObsRcv" :depends-on ("_package"))
  ))