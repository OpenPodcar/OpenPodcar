;; Auto-generated. Do not edit!


(when (boundp 'podcar::Joystick)
  (if (not (find-package "PODCAR"))
    (make-package "PODCAR"))
  (shadow 'Joystick (find-package "PODCAR")))
(unless (find-package "PODCAR::JOYSTICK")
  (make-package "PODCAR::JOYSTICK"))

(in-package "ROS")
;;//! \htmlinclude Joystick.msg.html


(defclass podcar::Joystick
  :super ros::object
  :slots (_x _y _twist _throttle _button1 _button2 _button3 _button4 _button5 _button6 _button7 _button8 _button9 _button10 _button11 _button12 _button13 _button14 _button15 _button16 ))

(defmethod podcar::Joystick
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:twist __twist) 0.0)
    ((:throttle __throttle) 0.0)
    ((:button1 __button1) nil)
    ((:button2 __button2) nil)
    ((:button3 __button3) nil)
    ((:button4 __button4) nil)
    ((:button5 __button5) nil)
    ((:button6 __button6) nil)
    ((:button7 __button7) nil)
    ((:button8 __button8) nil)
    ((:button9 __button9) nil)
    ((:button10 __button10) nil)
    ((:button11 __button11) nil)
    ((:button12 __button12) nil)
    ((:button13 __button13) nil)
    ((:button14 __button14) nil)
    ((:button15 __button15) nil)
    ((:button16 __button16) nil)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _twist (float __twist))
   (setq _throttle (float __throttle))
   (setq _button1 __button1)
   (setq _button2 __button2)
   (setq _button3 __button3)
   (setq _button4 __button4)
   (setq _button5 __button5)
   (setq _button6 __button6)
   (setq _button7 __button7)
   (setq _button8 __button8)
   (setq _button9 __button9)
   (setq _button10 __button10)
   (setq _button11 __button11)
   (setq _button12 __button12)
   (setq _button13 __button13)
   (setq _button14 __button14)
   (setq _button15 __button15)
   (setq _button16 __button16)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:twist
   (&optional __twist)
   (if __twist (setq _twist __twist)) _twist)
  (:throttle
   (&optional __throttle)
   (if __throttle (setq _throttle __throttle)) _throttle)
  (:button1
   (&optional __button1)
   (if __button1 (setq _button1 __button1)) _button1)
  (:button2
   (&optional __button2)
   (if __button2 (setq _button2 __button2)) _button2)
  (:button3
   (&optional __button3)
   (if __button3 (setq _button3 __button3)) _button3)
  (:button4
   (&optional __button4)
   (if __button4 (setq _button4 __button4)) _button4)
  (:button5
   (&optional __button5)
   (if __button5 (setq _button5 __button5)) _button5)
  (:button6
   (&optional __button6)
   (if __button6 (setq _button6 __button6)) _button6)
  (:button7
   (&optional __button7)
   (if __button7 (setq _button7 __button7)) _button7)
  (:button8
   (&optional __button8)
   (if __button8 (setq _button8 __button8)) _button8)
  (:button9
   (&optional __button9)
   (if __button9 (setq _button9 __button9)) _button9)
  (:button10
   (&optional __button10)
   (if __button10 (setq _button10 __button10)) _button10)
  (:button11
   (&optional __button11)
   (if __button11 (setq _button11 __button11)) _button11)
  (:button12
   (&optional __button12)
   (if __button12 (setq _button12 __button12)) _button12)
  (:button13
   (&optional __button13)
   (if __button13 (setq _button13 __button13)) _button13)
  (:button14
   (&optional __button14)
   (if __button14 (setq _button14 __button14)) _button14)
  (:button15
   (&optional __button15)
   (if __button15 (setq _button15 __button15)) _button15)
  (:button16
   (&optional __button16)
   (if __button16 (setq _button16 __button16)) _button16)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _twist
    8
    ;; float64 _throttle
    8
    ;; bool _button1
    1
    ;; bool _button2
    1
    ;; bool _button3
    1
    ;; bool _button4
    1
    ;; bool _button5
    1
    ;; bool _button6
    1
    ;; bool _button7
    1
    ;; bool _button8
    1
    ;; bool _button9
    1
    ;; bool _button10
    1
    ;; bool _button11
    1
    ;; bool _button12
    1
    ;; bool _button13
    1
    ;; bool _button14
    1
    ;; bool _button15
    1
    ;; bool _button16
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _twist
       (sys::poke _twist (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _throttle
       (sys::poke _throttle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _button1
       (if _button1 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button2
       (if _button2 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button3
       (if _button3 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button4
       (if _button4 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button5
       (if _button5 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button6
       (if _button6 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button7
       (if _button7 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button8
       (if _button8 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button9
       (if _button9 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button10
       (if _button10 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button11
       (if _button11 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button12
       (if _button12 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button13
       (if _button13 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button14
       (if _button14 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button15
       (if _button15 (write-byte -1 s) (write-byte 0 s))
     ;; bool _button16
       (if _button16 (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _twist
     (setq _twist (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _throttle
     (setq _throttle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _button1
     (setq _button1 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button2
     (setq _button2 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button3
     (setq _button3 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button4
     (setq _button4 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button5
     (setq _button5 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button6
     (setq _button6 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button7
     (setq _button7 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button8
     (setq _button8 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button9
     (setq _button9 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button10
     (setq _button10 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button11
     (setq _button11 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button12
     (setq _button12 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button13
     (setq _button13 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button14
     (setq _button14 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button15
     (setq _button15 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button16
     (setq _button16 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get podcar::Joystick :md5sum-) "8ed7d955f44aee65ff70d63051fd8603")
(setf (get podcar::Joystick :datatype-) "podcar/Joystick")
(setf (get podcar::Joystick :definition-)
      "float64 x
float64 y
float64 twist
float64 throttle
bool button1
bool button2
bool button3
bool button4
bool button5
bool button6
bool button7
bool button8
bool button9
bool button10
bool button11
bool button12
bool button13
bool button14
bool button15
bool button16

")



(provide :podcar/Joystick "8ed7d955f44aee65ff70d63051fd8603")


