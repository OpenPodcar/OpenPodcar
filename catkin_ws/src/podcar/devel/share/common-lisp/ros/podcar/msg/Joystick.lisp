; Auto-generated. Do not edit!


(cl:in-package podcar-msg)


;//! \htmlinclude Joystick.msg.html

(cl:defclass <Joystick> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (twist
    :reader twist
    :initarg :twist
    :type cl:float
    :initform 0.0)
   (throttle
    :reader throttle
    :initarg :throttle
    :type cl:float
    :initform 0.0)
   (button1
    :reader button1
    :initarg :button1
    :type cl:boolean
    :initform cl:nil)
   (button2
    :reader button2
    :initarg :button2
    :type cl:boolean
    :initform cl:nil)
   (button3
    :reader button3
    :initarg :button3
    :type cl:boolean
    :initform cl:nil)
   (button4
    :reader button4
    :initarg :button4
    :type cl:boolean
    :initform cl:nil)
   (button5
    :reader button5
    :initarg :button5
    :type cl:boolean
    :initform cl:nil)
   (button6
    :reader button6
    :initarg :button6
    :type cl:boolean
    :initform cl:nil)
   (button7
    :reader button7
    :initarg :button7
    :type cl:boolean
    :initform cl:nil)
   (button8
    :reader button8
    :initarg :button8
    :type cl:boolean
    :initform cl:nil)
   (button9
    :reader button9
    :initarg :button9
    :type cl:boolean
    :initform cl:nil)
   (button10
    :reader button10
    :initarg :button10
    :type cl:boolean
    :initform cl:nil)
   (button11
    :reader button11
    :initarg :button11
    :type cl:boolean
    :initform cl:nil)
   (button12
    :reader button12
    :initarg :button12
    :type cl:boolean
    :initform cl:nil)
   (button13
    :reader button13
    :initarg :button13
    :type cl:boolean
    :initform cl:nil)
   (button14
    :reader button14
    :initarg :button14
    :type cl:boolean
    :initform cl:nil)
   (button15
    :reader button15
    :initarg :button15
    :type cl:boolean
    :initform cl:nil)
   (button16
    :reader button16
    :initarg :button16
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Joystick (<Joystick>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Joystick>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Joystick)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name podcar-msg:<Joystick> is deprecated: use podcar-msg:Joystick instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:x-val is deprecated.  Use podcar-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:y-val is deprecated.  Use podcar-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'twist-val :lambda-list '(m))
(cl:defmethod twist-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:twist-val is deprecated.  Use podcar-msg:twist instead.")
  (twist m))

(cl:ensure-generic-function 'throttle-val :lambda-list '(m))
(cl:defmethod throttle-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:throttle-val is deprecated.  Use podcar-msg:throttle instead.")
  (throttle m))

(cl:ensure-generic-function 'button1-val :lambda-list '(m))
(cl:defmethod button1-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button1-val is deprecated.  Use podcar-msg:button1 instead.")
  (button1 m))

(cl:ensure-generic-function 'button2-val :lambda-list '(m))
(cl:defmethod button2-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button2-val is deprecated.  Use podcar-msg:button2 instead.")
  (button2 m))

(cl:ensure-generic-function 'button3-val :lambda-list '(m))
(cl:defmethod button3-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button3-val is deprecated.  Use podcar-msg:button3 instead.")
  (button3 m))

(cl:ensure-generic-function 'button4-val :lambda-list '(m))
(cl:defmethod button4-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button4-val is deprecated.  Use podcar-msg:button4 instead.")
  (button4 m))

(cl:ensure-generic-function 'button5-val :lambda-list '(m))
(cl:defmethod button5-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button5-val is deprecated.  Use podcar-msg:button5 instead.")
  (button5 m))

(cl:ensure-generic-function 'button6-val :lambda-list '(m))
(cl:defmethod button6-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button6-val is deprecated.  Use podcar-msg:button6 instead.")
  (button6 m))

(cl:ensure-generic-function 'button7-val :lambda-list '(m))
(cl:defmethod button7-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button7-val is deprecated.  Use podcar-msg:button7 instead.")
  (button7 m))

(cl:ensure-generic-function 'button8-val :lambda-list '(m))
(cl:defmethod button8-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button8-val is deprecated.  Use podcar-msg:button8 instead.")
  (button8 m))

(cl:ensure-generic-function 'button9-val :lambda-list '(m))
(cl:defmethod button9-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button9-val is deprecated.  Use podcar-msg:button9 instead.")
  (button9 m))

(cl:ensure-generic-function 'button10-val :lambda-list '(m))
(cl:defmethod button10-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button10-val is deprecated.  Use podcar-msg:button10 instead.")
  (button10 m))

(cl:ensure-generic-function 'button11-val :lambda-list '(m))
(cl:defmethod button11-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button11-val is deprecated.  Use podcar-msg:button11 instead.")
  (button11 m))

(cl:ensure-generic-function 'button12-val :lambda-list '(m))
(cl:defmethod button12-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button12-val is deprecated.  Use podcar-msg:button12 instead.")
  (button12 m))

(cl:ensure-generic-function 'button13-val :lambda-list '(m))
(cl:defmethod button13-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button13-val is deprecated.  Use podcar-msg:button13 instead.")
  (button13 m))

(cl:ensure-generic-function 'button14-val :lambda-list '(m))
(cl:defmethod button14-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button14-val is deprecated.  Use podcar-msg:button14 instead.")
  (button14 m))

(cl:ensure-generic-function 'button15-val :lambda-list '(m))
(cl:defmethod button15-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button15-val is deprecated.  Use podcar-msg:button15 instead.")
  (button15 m))

(cl:ensure-generic-function 'button16-val :lambda-list '(m))
(cl:defmethod button16-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader podcar-msg:button16-val is deprecated.  Use podcar-msg:button16 instead.")
  (button16 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Joystick>) ostream)
  "Serializes a message object of type '<Joystick>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'twist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'throttle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button1) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button2) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button3) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button4) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button5) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button6) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button7) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button8) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button9) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button10) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button11) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button12) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button13) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button14) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button15) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button16) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Joystick>) istream)
  "Deserializes a message object of type '<Joystick>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'twist) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'throttle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'button1) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button2) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button3) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button4) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button5) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button6) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button7) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button8) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button9) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button10) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button11) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button12) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button13) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button14) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button15) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button16) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Joystick>)))
  "Returns string type for a message object of type '<Joystick>"
  "podcar/Joystick")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Joystick)))
  "Returns string type for a message object of type 'Joystick"
  "podcar/Joystick")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Joystick>)))
  "Returns md5sum for a message object of type '<Joystick>"
  "8ed7d955f44aee65ff70d63051fd8603")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Joystick)))
  "Returns md5sum for a message object of type 'Joystick"
  "8ed7d955f44aee65ff70d63051fd8603")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Joystick>)))
  "Returns full string definition for message of type '<Joystick>"
  (cl:format cl:nil "float64 x~%float64 y~%float64 twist~%float64 throttle~%bool button1~%bool button2~%bool button3~%bool button4~%bool button5~%bool button6~%bool button7~%bool button8~%bool button9~%bool button10~%bool button11~%bool button12~%bool button13~%bool button14~%bool button15~%bool button16~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Joystick)))
  "Returns full string definition for message of type 'Joystick"
  (cl:format cl:nil "float64 x~%float64 y~%float64 twist~%float64 throttle~%bool button1~%bool button2~%bool button3~%bool button4~%bool button5~%bool button6~%bool button7~%bool button8~%bool button9~%bool button10~%bool button11~%bool button12~%bool button13~%bool button14~%bool button15~%bool button16~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Joystick>))
  (cl:+ 0
     8
     8
     8
     8
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Joystick>))
  "Converts a ROS message object to a list"
  (cl:list 'Joystick
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':twist (twist msg))
    (cl:cons ':throttle (throttle msg))
    (cl:cons ':button1 (button1 msg))
    (cl:cons ':button2 (button2 msg))
    (cl:cons ':button3 (button3 msg))
    (cl:cons ':button4 (button4 msg))
    (cl:cons ':button5 (button5 msg))
    (cl:cons ':button6 (button6 msg))
    (cl:cons ':button7 (button7 msg))
    (cl:cons ':button8 (button8 msg))
    (cl:cons ':button9 (button9 msg))
    (cl:cons ':button10 (button10 msg))
    (cl:cons ':button11 (button11 msg))
    (cl:cons ':button12 (button12 msg))
    (cl:cons ':button13 (button13 msg))
    (cl:cons ':button14 (button14 msg))
    (cl:cons ':button15 (button15 msg))
    (cl:cons ':button16 (button16 msg))
))
