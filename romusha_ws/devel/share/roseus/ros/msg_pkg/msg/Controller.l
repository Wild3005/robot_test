;; Auto-generated. Do not edit!


(when (boundp 'msg_pkg::Controller)
  (if (not (find-package "MSG_PKG"))
    (make-package "MSG_PKG"))
  (shadow 'Controller (find-package "MSG_PKG")))
(unless (find-package "MSG_PKG::CONTROLLER")
  (make-package "MSG_PKG::CONTROLLER"))

(in-package "ROS")
;;//! \htmlinclude Controller.msg.html


(defclass msg_pkg::Controller
  :super ros::object
  :slots (_velx _vely _angvel ))

(defmethod msg_pkg::Controller
  (:init
   (&key
    ((:velx __velx) 0.0)
    ((:vely __vely) 0.0)
    ((:angvel __angvel) 0.0)
    )
   (send-super :init)
   (setq _velx (float __velx))
   (setq _vely (float __vely))
   (setq _angvel (float __angvel))
   self)
  (:velx
   (&optional __velx)
   (if __velx (setq _velx __velx)) _velx)
  (:vely
   (&optional __vely)
   (if __vely (setq _vely __vely)) _vely)
  (:angvel
   (&optional __angvel)
   (if __angvel (setq _angvel __angvel)) _angvel)
  (:serialization-length
   ()
   (+
    ;; float32 _velx
    4
    ;; float32 _vely
    4
    ;; float32 _angvel
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _velx
       (sys::poke _velx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vely
       (sys::poke _vely (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angvel
       (sys::poke _angvel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _velx
     (setq _velx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vely
     (setq _vely (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angvel
     (setq _angvel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get msg_pkg::Controller :md5sum-) "db828269e36b1326ea6e28d784881f34")
(setf (get msg_pkg::Controller :datatype-) "msg_pkg/Controller")
(setf (get msg_pkg::Controller :definition-)
      "float32 velx
float32 vely
float32 angvel
")



(provide :msg_pkg/Controller "db828269e36b1326ea6e28d784881f34")


