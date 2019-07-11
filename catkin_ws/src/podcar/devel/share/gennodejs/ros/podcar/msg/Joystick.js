// Auto-generated. Do not edit!

// (in-package podcar.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Joystick {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.twist = null;
      this.throttle = null;
      this.button1 = null;
      this.button2 = null;
      this.button3 = null;
      this.button4 = null;
      this.button5 = null;
      this.button6 = null;
      this.button7 = null;
      this.button8 = null;
      this.button9 = null;
      this.button10 = null;
      this.button11 = null;
      this.button12 = null;
      this.button13 = null;
      this.button14 = null;
      this.button15 = null;
      this.button16 = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('twist')) {
        this.twist = initObj.twist
      }
      else {
        this.twist = 0.0;
      }
      if (initObj.hasOwnProperty('throttle')) {
        this.throttle = initObj.throttle
      }
      else {
        this.throttle = 0.0;
      }
      if (initObj.hasOwnProperty('button1')) {
        this.button1 = initObj.button1
      }
      else {
        this.button1 = false;
      }
      if (initObj.hasOwnProperty('button2')) {
        this.button2 = initObj.button2
      }
      else {
        this.button2 = false;
      }
      if (initObj.hasOwnProperty('button3')) {
        this.button3 = initObj.button3
      }
      else {
        this.button3 = false;
      }
      if (initObj.hasOwnProperty('button4')) {
        this.button4 = initObj.button4
      }
      else {
        this.button4 = false;
      }
      if (initObj.hasOwnProperty('button5')) {
        this.button5 = initObj.button5
      }
      else {
        this.button5 = false;
      }
      if (initObj.hasOwnProperty('button6')) {
        this.button6 = initObj.button6
      }
      else {
        this.button6 = false;
      }
      if (initObj.hasOwnProperty('button7')) {
        this.button7 = initObj.button7
      }
      else {
        this.button7 = false;
      }
      if (initObj.hasOwnProperty('button8')) {
        this.button8 = initObj.button8
      }
      else {
        this.button8 = false;
      }
      if (initObj.hasOwnProperty('button9')) {
        this.button9 = initObj.button9
      }
      else {
        this.button9 = false;
      }
      if (initObj.hasOwnProperty('button10')) {
        this.button10 = initObj.button10
      }
      else {
        this.button10 = false;
      }
      if (initObj.hasOwnProperty('button11')) {
        this.button11 = initObj.button11
      }
      else {
        this.button11 = false;
      }
      if (initObj.hasOwnProperty('button12')) {
        this.button12 = initObj.button12
      }
      else {
        this.button12 = false;
      }
      if (initObj.hasOwnProperty('button13')) {
        this.button13 = initObj.button13
      }
      else {
        this.button13 = false;
      }
      if (initObj.hasOwnProperty('button14')) {
        this.button14 = initObj.button14
      }
      else {
        this.button14 = false;
      }
      if (initObj.hasOwnProperty('button15')) {
        this.button15 = initObj.button15
      }
      else {
        this.button15 = false;
      }
      if (initObj.hasOwnProperty('button16')) {
        this.button16 = initObj.button16
      }
      else {
        this.button16 = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Joystick
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [twist]
    bufferOffset = _serializer.float64(obj.twist, buffer, bufferOffset);
    // Serialize message field [throttle]
    bufferOffset = _serializer.float64(obj.throttle, buffer, bufferOffset);
    // Serialize message field [button1]
    bufferOffset = _serializer.bool(obj.button1, buffer, bufferOffset);
    // Serialize message field [button2]
    bufferOffset = _serializer.bool(obj.button2, buffer, bufferOffset);
    // Serialize message field [button3]
    bufferOffset = _serializer.bool(obj.button3, buffer, bufferOffset);
    // Serialize message field [button4]
    bufferOffset = _serializer.bool(obj.button4, buffer, bufferOffset);
    // Serialize message field [button5]
    bufferOffset = _serializer.bool(obj.button5, buffer, bufferOffset);
    // Serialize message field [button6]
    bufferOffset = _serializer.bool(obj.button6, buffer, bufferOffset);
    // Serialize message field [button7]
    bufferOffset = _serializer.bool(obj.button7, buffer, bufferOffset);
    // Serialize message field [button8]
    bufferOffset = _serializer.bool(obj.button8, buffer, bufferOffset);
    // Serialize message field [button9]
    bufferOffset = _serializer.bool(obj.button9, buffer, bufferOffset);
    // Serialize message field [button10]
    bufferOffset = _serializer.bool(obj.button10, buffer, bufferOffset);
    // Serialize message field [button11]
    bufferOffset = _serializer.bool(obj.button11, buffer, bufferOffset);
    // Serialize message field [button12]
    bufferOffset = _serializer.bool(obj.button12, buffer, bufferOffset);
    // Serialize message field [button13]
    bufferOffset = _serializer.bool(obj.button13, buffer, bufferOffset);
    // Serialize message field [button14]
    bufferOffset = _serializer.bool(obj.button14, buffer, bufferOffset);
    // Serialize message field [button15]
    bufferOffset = _serializer.bool(obj.button15, buffer, bufferOffset);
    // Serialize message field [button16]
    bufferOffset = _serializer.bool(obj.button16, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Joystick
    let len;
    let data = new Joystick(null);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [twist]
    data.twist = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [throttle]
    data.throttle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [button1]
    data.button1 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button2]
    data.button2 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button3]
    data.button3 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button4]
    data.button4 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button5]
    data.button5 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button6]
    data.button6 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button7]
    data.button7 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button8]
    data.button8 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button9]
    data.button9 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button10]
    data.button10 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button11]
    data.button11 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button12]
    data.button12 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button13]
    data.button13 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button14]
    data.button14 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button15]
    data.button15 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button16]
    data.button16 = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'podcar/Joystick';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8ed7d955f44aee65ff70d63051fd8603';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 x
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Joystick(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.twist !== undefined) {
      resolved.twist = msg.twist;
    }
    else {
      resolved.twist = 0.0
    }

    if (msg.throttle !== undefined) {
      resolved.throttle = msg.throttle;
    }
    else {
      resolved.throttle = 0.0
    }

    if (msg.button1 !== undefined) {
      resolved.button1 = msg.button1;
    }
    else {
      resolved.button1 = false
    }

    if (msg.button2 !== undefined) {
      resolved.button2 = msg.button2;
    }
    else {
      resolved.button2 = false
    }

    if (msg.button3 !== undefined) {
      resolved.button3 = msg.button3;
    }
    else {
      resolved.button3 = false
    }

    if (msg.button4 !== undefined) {
      resolved.button4 = msg.button4;
    }
    else {
      resolved.button4 = false
    }

    if (msg.button5 !== undefined) {
      resolved.button5 = msg.button5;
    }
    else {
      resolved.button5 = false
    }

    if (msg.button6 !== undefined) {
      resolved.button6 = msg.button6;
    }
    else {
      resolved.button6 = false
    }

    if (msg.button7 !== undefined) {
      resolved.button7 = msg.button7;
    }
    else {
      resolved.button7 = false
    }

    if (msg.button8 !== undefined) {
      resolved.button8 = msg.button8;
    }
    else {
      resolved.button8 = false
    }

    if (msg.button9 !== undefined) {
      resolved.button9 = msg.button9;
    }
    else {
      resolved.button9 = false
    }

    if (msg.button10 !== undefined) {
      resolved.button10 = msg.button10;
    }
    else {
      resolved.button10 = false
    }

    if (msg.button11 !== undefined) {
      resolved.button11 = msg.button11;
    }
    else {
      resolved.button11 = false
    }

    if (msg.button12 !== undefined) {
      resolved.button12 = msg.button12;
    }
    else {
      resolved.button12 = false
    }

    if (msg.button13 !== undefined) {
      resolved.button13 = msg.button13;
    }
    else {
      resolved.button13 = false
    }

    if (msg.button14 !== undefined) {
      resolved.button14 = msg.button14;
    }
    else {
      resolved.button14 = false
    }

    if (msg.button15 !== undefined) {
      resolved.button15 = msg.button15;
    }
    else {
      resolved.button15 = false
    }

    if (msg.button16 !== undefined) {
      resolved.button16 = msg.button16;
    }
    else {
      resolved.button16 = false
    }

    return resolved;
    }
};

module.exports = Joystick;
