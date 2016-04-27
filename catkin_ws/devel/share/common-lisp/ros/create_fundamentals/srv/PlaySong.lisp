; Auto-generated. Do not edit!


(cl:in-package create_fundamentals-srv)


;//! \htmlinclude PlaySong-request.msg.html

(cl:defclass <PlaySong-request> (roslisp-msg-protocol:ros-message)
  ((number
    :reader number
    :initarg :number
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PlaySong-request (<PlaySong-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlaySong-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlaySong-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name create_fundamentals-srv:<PlaySong-request> is deprecated: use create_fundamentals-srv:PlaySong-request instead.")))

(cl:ensure-generic-function 'number-val :lambda-list '(m))
(cl:defmethod number-val ((m <PlaySong-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader create_fundamentals-srv:number-val is deprecated.  Use create_fundamentals-srv:number instead.")
  (number m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlaySong-request>) ostream)
  "Serializes a message object of type '<PlaySong-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlaySong-request>) istream)
  "Deserializes a message object of type '<PlaySong-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlaySong-request>)))
  "Returns string type for a service object of type '<PlaySong-request>"
  "create_fundamentals/PlaySongRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlaySong-request)))
  "Returns string type for a service object of type 'PlaySong-request"
  "create_fundamentals/PlaySongRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlaySong-request>)))
  "Returns md5sum for a message object of type '<PlaySong-request>"
  "1684a4a334f6ea88c5886966e6b29799")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlaySong-request)))
  "Returns md5sum for a message object of type 'PlaySong-request"
  "1684a4a334f6ea88c5886966e6b29799")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlaySong-request>)))
  "Returns full string definition for message of type '<PlaySong-request>"
  (cl:format cl:nil "uint8 number~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlaySong-request)))
  "Returns full string definition for message of type 'PlaySong-request"
  (cl:format cl:nil "uint8 number~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlaySong-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlaySong-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlaySong-request
    (cl:cons ':number (number msg))
))
;//! \htmlinclude PlaySong-response.msg.html

(cl:defclass <PlaySong-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PlaySong-response (<PlaySong-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlaySong-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlaySong-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name create_fundamentals-srv:<PlaySong-response> is deprecated: use create_fundamentals-srv:PlaySong-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <PlaySong-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader create_fundamentals-srv:success-val is deprecated.  Use create_fundamentals-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlaySong-response>) ostream)
  "Serializes a message object of type '<PlaySong-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlaySong-response>) istream)
  "Deserializes a message object of type '<PlaySong-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlaySong-response>)))
  "Returns string type for a service object of type '<PlaySong-response>"
  "create_fundamentals/PlaySongResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlaySong-response)))
  "Returns string type for a service object of type 'PlaySong-response"
  "create_fundamentals/PlaySongResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlaySong-response>)))
  "Returns md5sum for a message object of type '<PlaySong-response>"
  "1684a4a334f6ea88c5886966e6b29799")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlaySong-response)))
  "Returns md5sum for a message object of type 'PlaySong-response"
  "1684a4a334f6ea88c5886966e6b29799")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlaySong-response>)))
  "Returns full string definition for message of type '<PlaySong-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlaySong-response)))
  "Returns full string definition for message of type 'PlaySong-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlaySong-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlaySong-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlaySong-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlaySong)))
  'PlaySong-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlaySong)))
  'PlaySong-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlaySong)))
  "Returns string type for a service object of type '<PlaySong>"
  "create_fundamentals/PlaySong")