
(cl:in-package :asdf)

(defsystem "create_fundamentals-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SensorPacket" :depends-on ("_package_SensorPacket"))
    (:file "_package_SensorPacket" :depends-on ("_package"))
    (:file "Encoder" :depends-on ("_package_Encoder"))
    (:file "_package_Encoder" :depends-on ("_package"))
  ))