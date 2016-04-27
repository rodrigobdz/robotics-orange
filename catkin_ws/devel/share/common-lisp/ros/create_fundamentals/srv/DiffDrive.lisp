; Auto-generated. Do not edit!


(cl:in-package create_fundamentals-srv)


;//! \htmlinclude DiffDrive-request.msg.html

(cl:defclass <DiffDrive-request> (roslisp-msg-protocol:ros-message)
  ((left
    :reader left
    :initarg :left
    :type cl:float
    :initform 0.0)
   (right
    :reader right
    :initarg :right
    :type cl:float
    :initform 0.0))
)

(cl:defclass DiffDrive-request (<DiffDrive-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DiffDrive-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DiffDrive-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name create_fundamentals-srv:<DiffDrive-request> is deprecated: use create_fundamentals-srv:DiffDrive-request instead.")))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <DiffDrive-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader create_fundamentals-srv:left-val is deprecated.  Use create_fundamentals-srv:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <DiffDrive-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader create_fundamentals-srv:right-val is deprecated.  Use create_fundamentals-srv:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DiffDrive-request>) ostream)
  "Serializes a message object of type '<DiffDrive-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DiffDrive-request>) istream)
  "Deserializes a message object of type '<DiffDrive-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DiffDrive-request>)))
  "Returns string type for a service object of type '<DiffDrive-request>"
  "create_fundamentals/DiffDriveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DiffDrive-request)))
  "Returns string type for a service object of type 'DiffDrive-request"
  "create_fundamentals/DiffDriveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DiffDrive-request>)))
  "Returns md5sum for a message object of type '<DiffDrive-request>"
  "7c56fde7f27c872d8a97d66ee4b33e33")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DiffDrive-request)))
  "Returns md5sum for a message object of type 'DiffDrive-request"
  "7c56fde7f27c872d8a97d66ee4b33e33")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DiffDrive-request>)))
  "Returns full string definition for message of type '<DiffDrive-request>"
  (cl:format cl:nil "float32 left~%float32 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DiffDrive-request)))
  "Returns full string definition for message of type 'DiffDrive-request"
  (cl:format cl:nil "float32 left~%float32 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DiffDrive-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DiffDrive-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DiffDrive-request
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
))
;//! \htmlinclude DiffDrive-response.msg.html

(cl:defclass <DiffDrive-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DiffDrive-response (<DiffDrive-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DiffDrive-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DiffDrive-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name create_fundamentals-srv:<DiffDrive-response> is deprecated: use create_fundamentals-srv:DiffDrive-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DiffDrive-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader create_fundamentals-srv:success-val is deprecated.  Use create_fundamentals-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DiffDrive-response>) ostream)
  "Serializes a message object of type '<DiffDrive-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DiffDrive-response>) istream)
  "Deserializes a message object of type '<DiffDrive-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DiffDrive-response>)))
  "Returns string type for a service object of type '<DiffDrive-response>"
  "create_fundamentals/DiffDriveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DiffDrive-response)))
  "Returns string type for a service object of type 'DiffDrive-response"
  "create_fundamentals/DiffDriveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DiffDrive-response>)))
  "Returns md5sum for a message object of type '<DiffDrive-response>"
  "7c56fde7f27c872d8a97d66ee4b33e33")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DiffDrive-response)))
  "Returns md5sum for a message object of type 'DiffDrive-response"
  "7c56fde7f27c872d8a97d66ee4b33e33")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DiffDrive-response>)))
  "Returns full string definition for message of type '<DiffDrive-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DiffDrive-response)))
  "Returns full string definition for message of type 'DiffDrive-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DiffDrive-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DiffDrive-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DiffDrive-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DiffDrive)))
  'DiffDrive-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DiffDrive)))
  'DiffDrive-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DiffDrive)))
  "Returns string type for a service object of type '<DiffDrive>"
  "create_fundamentals/DiffDrive")