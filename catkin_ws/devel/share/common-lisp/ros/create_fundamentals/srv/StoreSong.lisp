; Auto-generated. Do not edit!


(cl:in-package create_fundamentals-srv)


;//! \htmlinclude StoreSong-request.msg.html

(cl:defclass <StoreSong-request> (roslisp-msg-protocol:ros-message)
  ((number
    :reader number
    :initarg :number
    :type cl:integer
    :initform 0)
   (song
    :reader song
    :initarg :song
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass StoreSong-request (<StoreSong-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StoreSong-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StoreSong-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name create_fundamentals-srv:<StoreSong-request> is deprecated: use create_fundamentals-srv:StoreSong-request instead.")))

(cl:ensure-generic-function 'number-val :lambda-list '(m))
(cl:defmethod number-val ((m <StoreSong-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader create_fundamentals-srv:number-val is deprecated.  Use create_fundamentals-srv:number instead.")
  (number m))

(cl:ensure-generic-function 'song-val :lambda-list '(m))
(cl:defmethod song-val ((m <StoreSong-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader create_fundamentals-srv:song-val is deprecated.  Use create_fundamentals-srv:song instead.")
  (song m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StoreSong-request>) ostream)
  "Serializes a message object of type '<StoreSong-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'number)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'song))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'song))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StoreSong-request>) istream)
  "Deserializes a message object of type '<StoreSong-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'number)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'song) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'song)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StoreSong-request>)))
  "Returns string type for a service object of type '<StoreSong-request>"
  "create_fundamentals/StoreSongRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StoreSong-request)))
  "Returns string type for a service object of type 'StoreSong-request"
  "create_fundamentals/StoreSongRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StoreSong-request>)))
  "Returns md5sum for a message object of type '<StoreSong-request>"
  "1368a608865444cc82f73518170a4f78")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StoreSong-request)))
  "Returns md5sum for a message object of type 'StoreSong-request"
  "1368a608865444cc82f73518170a4f78")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StoreSong-request>)))
  "Returns full string definition for message of type '<StoreSong-request>"
  (cl:format cl:nil "uint32 number~%uint32[] song~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StoreSong-request)))
  "Returns full string definition for message of type 'StoreSong-request"
  (cl:format cl:nil "uint32 number~%uint32[] song~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StoreSong-request>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'song) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StoreSong-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StoreSong-request
    (cl:cons ':number (number msg))
    (cl:cons ':song (song msg))
))
;//! \htmlinclude StoreSong-response.msg.html

(cl:defclass <StoreSong-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass StoreSong-response (<StoreSong-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StoreSong-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StoreSong-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name create_fundamentals-srv:<StoreSong-response> is deprecated: use create_fundamentals-srv:StoreSong-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <StoreSong-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader create_fundamentals-srv:success-val is deprecated.  Use create_fundamentals-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StoreSong-response>) ostream)
  "Serializes a message object of type '<StoreSong-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StoreSong-response>) istream)
  "Deserializes a message object of type '<StoreSong-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StoreSong-response>)))
  "Returns string type for a service object of type '<StoreSong-response>"
  "create_fundamentals/StoreSongResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StoreSong-response)))
  "Returns string type for a service object of type 'StoreSong-response"
  "create_fundamentals/StoreSongResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StoreSong-response>)))
  "Returns md5sum for a message object of type '<StoreSong-response>"
  "1368a608865444cc82f73518170a4f78")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StoreSong-response)))
  "Returns md5sum for a message object of type 'StoreSong-response"
  "1368a608865444cc82f73518170a4f78")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StoreSong-response>)))
  "Returns full string definition for message of type '<StoreSong-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StoreSong-response)))
  "Returns full string definition for message of type 'StoreSong-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StoreSong-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StoreSong-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StoreSong-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StoreSong)))
  'StoreSong-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StoreSong)))
  'StoreSong-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StoreSong)))
  "Returns string type for a service object of type '<StoreSong>"
  "create_fundamentals/StoreSong")