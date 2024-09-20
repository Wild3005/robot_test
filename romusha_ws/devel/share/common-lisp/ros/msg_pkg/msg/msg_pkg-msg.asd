
(cl:in-package :asdf)

(defsystem "msg_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Controller" :depends-on ("_package_Controller"))
    (:file "_package_Controller" :depends-on ("_package"))
  ))