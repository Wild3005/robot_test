; Auto-generated. Do not edit!


(cl:in-package msg_pkg-msg)


;//! \htmlinclude Controller.msg.html

(cl:defclass <Controller> (roslisp-msg-protocol:ros-message)
  ((velx
    :reader velx
    :initarg :velx
    :type cl:float
    :initform 0.0)
   (vely
    :reader vely
    :initarg :vely
    :type cl:float
    :initform 0.0)
   (angvel
    :reader angvel
    :initarg :angvel
    :type cl:float
    :initform 0.0))
)

(cl:defclass Controller (<Controller>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Controller>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Controller)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msg_pkg-msg:<Controller> is deprecated: use msg_pkg-msg:Controller instead.")))

(cl:ensure-generic-function 'velx-val :lambda-list '(m))
(cl:defmethod velx-val ((m <Controller>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_pkg-msg:velx-val is deprecated.  Use msg_pkg-msg:velx instead.")
  (velx m))

(cl:ensure-generic-function 'vely-val :lambda-list '(m))
(cl:defmethod vely-val ((m <Controller>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_pkg-msg:vely-val is deprecated.  Use msg_pkg-msg:vely instead.")
  (vely m))

(cl:ensure-generic-function 'angvel-val :lambda-list '(m))
(cl:defmethod angvel-val ((m <Controller>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_pkg-msg:angvel-val is deprecated.  Use msg_pkg-msg:angvel instead.")
  (angvel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Controller>) ostream)
  "Serializes a message object of type '<Controller>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vely))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angvel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Controller>) istream)
  "Deserializes a message object of type '<Controller>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vely) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angvel) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Controller>)))
  "Returns string type for a message object of type '<Controller>"
  "msg_pkg/Controller")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Controller)))
  "Returns string type for a message object of type 'Controller"
  "msg_pkg/Controller")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Controller>)))
  "Returns md5sum for a message object of type '<Controller>"
  "db828269e36b1326ea6e28d784881f34")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Controller)))
  "Returns md5sum for a message object of type 'Controller"
  "db828269e36b1326ea6e28d784881f34")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Controller>)))
  "Returns full string definition for message of type '<Controller>"
  (cl:format cl:nil "float32 velx~%float32 vely~%float32 angvel~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Controller)))
  "Returns full string definition for message of type 'Controller"
  (cl:format cl:nil "float32 velx~%float32 vely~%float32 angvel~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Controller>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Controller>))
  "Converts a ROS message object to a list"
  (cl:list 'Controller
    (cl:cons ':velx (velx msg))
    (cl:cons ':vely (vely msg))
    (cl:cons ':angvel (angvel msg))
))
