// Auto-generated. Do not edit!

// (in-package traj_utils.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class LocalGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drone_id = null;
      this.global_traj_id = null;
      this.lg_pos_x = null;
      this.lg_pos_y = null;
      this.lg_pos_z = null;
      this.lg_vel_x = null;
      this.lg_vel_y = null;
      this.lg_vel_z = null;
    }
    else {
      if (initObj.hasOwnProperty('drone_id')) {
        this.drone_id = initObj.drone_id
      }
      else {
        this.drone_id = 0;
      }
      if (initObj.hasOwnProperty('global_traj_id')) {
        this.global_traj_id = initObj.global_traj_id
      }
      else {
        this.global_traj_id = 0;
      }
      if (initObj.hasOwnProperty('lg_pos_x')) {
        this.lg_pos_x = initObj.lg_pos_x
      }
      else {
        this.lg_pos_x = 0.0;
      }
      if (initObj.hasOwnProperty('lg_pos_y')) {
        this.lg_pos_y = initObj.lg_pos_y
      }
      else {
        this.lg_pos_y = 0.0;
      }
      if (initObj.hasOwnProperty('lg_pos_z')) {
        this.lg_pos_z = initObj.lg_pos_z
      }
      else {
        this.lg_pos_z = 0.0;
      }
      if (initObj.hasOwnProperty('lg_vel_x')) {
        this.lg_vel_x = initObj.lg_vel_x
      }
      else {
        this.lg_vel_x = 0.0;
      }
      if (initObj.hasOwnProperty('lg_vel_y')) {
        this.lg_vel_y = initObj.lg_vel_y
      }
      else {
        this.lg_vel_y = 0.0;
      }
      if (initObj.hasOwnProperty('lg_vel_z')) {
        this.lg_vel_z = initObj.lg_vel_z
      }
      else {
        this.lg_vel_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LocalGoal
    // Serialize message field [drone_id]
    bufferOffset = _serializer.int16(obj.drone_id, buffer, bufferOffset);
    // Serialize message field [global_traj_id]
    bufferOffset = _serializer.int16(obj.global_traj_id, buffer, bufferOffset);
    // Serialize message field [lg_pos_x]
    bufferOffset = _serializer.float32(obj.lg_pos_x, buffer, bufferOffset);
    // Serialize message field [lg_pos_y]
    bufferOffset = _serializer.float32(obj.lg_pos_y, buffer, bufferOffset);
    // Serialize message field [lg_pos_z]
    bufferOffset = _serializer.float32(obj.lg_pos_z, buffer, bufferOffset);
    // Serialize message field [lg_vel_x]
    bufferOffset = _serializer.float32(obj.lg_vel_x, buffer, bufferOffset);
    // Serialize message field [lg_vel_y]
    bufferOffset = _serializer.float32(obj.lg_vel_y, buffer, bufferOffset);
    // Serialize message field [lg_vel_z]
    bufferOffset = _serializer.float32(obj.lg_vel_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LocalGoal
    let len;
    let data = new LocalGoal(null);
    // Deserialize message field [drone_id]
    data.drone_id = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [global_traj_id]
    data.global_traj_id = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [lg_pos_x]
    data.lg_pos_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lg_pos_y]
    data.lg_pos_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lg_pos_z]
    data.lg_pos_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lg_vel_x]
    data.lg_vel_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lg_vel_y]
    data.lg_vel_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lg_vel_z]
    data.lg_vel_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_utils/LocalGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a8f0ef3ec042e7d8948442e97ca38913';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 drone_id
    int16 global_traj_id
    
    float32 lg_pos_x
    float32 lg_pos_y
    float32 lg_pos_z
    
    float32 lg_vel_x
    float32 lg_vel_y
    float32 lg_vel_z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LocalGoal(null);
    if (msg.drone_id !== undefined) {
      resolved.drone_id = msg.drone_id;
    }
    else {
      resolved.drone_id = 0
    }

    if (msg.global_traj_id !== undefined) {
      resolved.global_traj_id = msg.global_traj_id;
    }
    else {
      resolved.global_traj_id = 0
    }

    if (msg.lg_pos_x !== undefined) {
      resolved.lg_pos_x = msg.lg_pos_x;
    }
    else {
      resolved.lg_pos_x = 0.0
    }

    if (msg.lg_pos_y !== undefined) {
      resolved.lg_pos_y = msg.lg_pos_y;
    }
    else {
      resolved.lg_pos_y = 0.0
    }

    if (msg.lg_pos_z !== undefined) {
      resolved.lg_pos_z = msg.lg_pos_z;
    }
    else {
      resolved.lg_pos_z = 0.0
    }

    if (msg.lg_vel_x !== undefined) {
      resolved.lg_vel_x = msg.lg_vel_x;
    }
    else {
      resolved.lg_vel_x = 0.0
    }

    if (msg.lg_vel_y !== undefined) {
      resolved.lg_vel_y = msg.lg_vel_y;
    }
    else {
      resolved.lg_vel_y = 0.0
    }

    if (msg.lg_vel_z !== undefined) {
      resolved.lg_vel_z = msg.lg_vel_z;
    }
    else {
      resolved.lg_vel_z = 0.0
    }

    return resolved;
    }
};

module.exports = LocalGoal;
