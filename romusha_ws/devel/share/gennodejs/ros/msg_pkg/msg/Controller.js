// Auto-generated. Do not edit!

// (in-package msg_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Controller {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.velx = null;
      this.vely = null;
      this.angvel = null;
    }
    else {
      if (initObj.hasOwnProperty('velx')) {
        this.velx = initObj.velx
      }
      else {
        this.velx = 0.0;
      }
      if (initObj.hasOwnProperty('vely')) {
        this.vely = initObj.vely
      }
      else {
        this.vely = 0.0;
      }
      if (initObj.hasOwnProperty('angvel')) {
        this.angvel = initObj.angvel
      }
      else {
        this.angvel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Controller
    // Serialize message field [velx]
    bufferOffset = _serializer.float32(obj.velx, buffer, bufferOffset);
    // Serialize message field [vely]
    bufferOffset = _serializer.float32(obj.vely, buffer, bufferOffset);
    // Serialize message field [angvel]
    bufferOffset = _serializer.float32(obj.angvel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Controller
    let len;
    let data = new Controller(null);
    // Deserialize message field [velx]
    data.velx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vely]
    data.vely = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angvel]
    data.angvel = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'msg_pkg/Controller';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'db828269e36b1326ea6e28d784881f34';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 velx
    float32 vely
    float32 angvel
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Controller(null);
    if (msg.velx !== undefined) {
      resolved.velx = msg.velx;
    }
    else {
      resolved.velx = 0.0
    }

    if (msg.vely !== undefined) {
      resolved.vely = msg.vely;
    }
    else {
      resolved.vely = 0.0
    }

    if (msg.angvel !== undefined) {
      resolved.angvel = msg.angvel;
    }
    else {
      resolved.angvel = 0.0
    }

    return resolved;
    }
};

module.exports = Controller;
