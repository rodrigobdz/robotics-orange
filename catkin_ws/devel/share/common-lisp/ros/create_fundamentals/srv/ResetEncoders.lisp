; Auto-generated. Do not edit!


(cl:in-package create_fundamentals-srv)


;//! \htmlinclude ResetEncoders-request.msg.html

(cl:defclass <ResetEncoders-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ResetEncoders-request (<ResetEncoders-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetEncoders-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetEncoders-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name create_fundamentals-srv:<ResetEncoders-request> is deprecated: use create_fundamentals-srv:ResetEncoders-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetEncoders-request>) ostream)
  "Serializes a message object of type '<ResetEncoders-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetEncoders-request>) istream)
  "Deserializes a message object of type '<ResetEncoders-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetEncoders-request>)))
  "Returns string type for a service object of type '<ResetEncoders-request>"
  "create_fundamentals/ResetEncodersRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetEncoders-request)))
  "Returns string type for a service object of type 'ResetEncoders-request"
  "create_fundamentals/ResetEncodersRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetEncoders-request>)))
  "Returns md5sum for a message object of type '<ResetEncoders-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetEncoders-request)))
  "Returns md5sum for a message object of type 'ResetEncoders-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetEncoders-request>)))
  "Returns full string definition for message of type '<ResetEncoders-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetEncoders-request)))
  "Returns full string definition for message of type 'ResetEncoders-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetEncoders-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetEncoders-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetEncoders-request
))
;//! \htmlinclude ResetEncoders-response.msg.html

(cl:defclass <ResetEncoders-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ResetEncoders-response (<ResetEncoders-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetEncoders-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetEncoders-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name create_fundamentals-srv:<ResetEncoders-response> is deprecated: use create_fundamentals-srv:ResetEncoders-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ResetEncoders-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader create_fundamentals-srv:success-val is deprecated.  Use create_fundamentals-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetEncoders-response>) ostream)
  "Serializes a message object of type '<ResetEncoders-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetEncoders-response>) istream)
  "Deserializes a message object of type '<ResetEncoders-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetEncoders-response>)))
  "Returns string type for a service object of type '<ResetEncoders-response>"
  "create_fundamentals/ResetEncodersResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetEncoders-response)))
  "Returns string type for a service object of type 'ResetEncoders-response"
  "create_fundamentals/ResetEncodersResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetEncoders-response>)))
  "Returns md5sum for a message object of type '<ResetEncoders-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetEncoders-response)))
  "Returns md5sum for a message object of type 'ResetEncoders-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetEncoders-response>)))
  "Returns full string definition for message of type '<ResetEncoders-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetEncoders-response)))
  "Returns full string definition for message of type 'ResetEncoders-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetEncoders-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetEncoders-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetEncoders-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetEncoders)))
  'ResetEncoders-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetEncoders)))
  'ResetEncoders-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetEncoders)))
  "Returns string type for a service object of type '<ResetEncoders>"
  "create_fundamentals/ResetEncoders")