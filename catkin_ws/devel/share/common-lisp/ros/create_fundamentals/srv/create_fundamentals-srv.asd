
(cl:in-package :asdf)

(defsystem "create_fundamentals-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "StoreSong" :depends-on ("_package_StoreSong"))
    (:file "_package_StoreSong" :depends-on ("_package"))
    (:file "Leds" :depends-on ("_package_Leds"))
    (:file "_package_Leds" :depends-on ("_package"))
    (:file "DiffDrive" :depends-on ("_package_DiffDrive"))
    (:file "_package_DiffDrive" :depends-on ("_package"))
    (:file "ResetEncoders" :depends-on ("_package_ResetEncoders"))
    (:file "_package_ResetEncoders" :depends-on ("_package"))
    (:file "PlaySong" :depends-on ("_package_PlaySong"))
    (:file "_package_PlaySong" :depends-on ("_package"))
  ))