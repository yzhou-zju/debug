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

class RemapLocalGoalList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.guard_drone_id = null;
      this.assignment = null;
      this.remap_lg_pos_x = null;
      this.remap_lg_pos_y = null;
      this.remap_lg_pos_z = null;
      this.remap_lg_vel_x = null;
      this.remap_lg_vel_y = null;
      this.remap_lg_vel_z = null;
    }
    else {
      if (initObj.hasOwnProperty('guard_drone_id')) {
        this.guard_drone_id = initObj.guard_drone_id
      }
      else {
        this.guard_drone_id = 0;
      }
      if (initObj.hasOwnProperty('assignment')) {
        this.assignment = initObj.assignment
      }
      else {
        this.assignment = [];
      }
      if (initObj.hasOwnProperty('remap_lg_pos_x')) {
        this.remap_lg_pos_x = initObj.remap_lg_pos_x
      }
      else {
        this.remap_lg_pos_x = [];
      }
      if (initObj.hasOwnProperty('remap_lg_pos_y')) {
        this.remap_lg_pos_y = initObj.remap_lg_pos_y
      }
      else {
        this.remap_lg_pos_y = [];
      }
      if (initObj.hasOwnProperty('remap_lg_pos_z')) {
        this.remap_lg_pos_z = initObj.remap_lg_pos_z
      }
      else {
        this.remap_lg_pos_z = [];
      }
      if (initObj.hasOwnProperty('remap_lg_vel_x')) {
        this.remap_lg_vel_x = initObj.remap_lg_vel_x
      }
      else {
        this.remap_lg_vel_x = [];
      }
      if (initObj.hasOwnProperty('remap_lg_vel_y')) {
        this.remap_lg_vel_y = initObj.remap_lg_vel_y
      }
      else {
        this.remap_lg_vel_y = [];
      }
      if (initObj.hasOwnProperty('remap_lg_vel_z')) {
        this.remap_lg_vel_z = initObj.remap_lg_vel_z
      }
      else {
        this.remap_lg_vel_z = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RemapLocalGoalList
    // Serialize message field [guard_drone_id]
    bufferOffset = _serializer.int16(obj.guard_drone_id, buffer, bufferOffset);
    // Serialize message field [assignment]
    bufferOffset = _arraySerializer.uint32(obj.assignment, buffer, bufferOffset, null);
    // Serialize message field [remap_lg_pos_x]
    bufferOffset = _arraySerializer.float32(obj.remap_lg_pos_x, buffer, bufferOffset, null);
    // Serialize message field [remap_lg_pos_y]
    bufferOffset = _arraySerializer.float32(obj.remap_lg_pos_y, buffer, bufferOffset, null);
    // Serialize message field [remap_lg_pos_z]
    bufferOffset = _arraySerializer.float32(obj.remap_lg_pos_z, buffer, bufferOffset, null);
    // Serialize message field [remap_lg_vel_x]
    bufferOffset = _arraySerializer.float32(obj.remap_lg_vel_x, buffer, bufferOffset, null);
    // Serialize message field [remap_lg_vel_y]
    bufferOffset = _arraySerializer.float32(obj.remap_lg_vel_y, buffer, bufferOffset, null);
    // Serialize message field [remap_lg_vel_z]
    bufferOffset = _arraySerializer.float32(obj.remap_lg_vel_z, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RemapLocalGoalList
    let len;
    let data = new RemapLocalGoalList(null);
    // Deserialize message field [guard_drone_id]
    data.guard_drone_id = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [assignment]
    data.assignment = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [remap_lg_pos_x]
    data.remap_lg_pos_x = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [remap_lg_pos_y]
    data.remap_lg_pos_y = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [remap_lg_pos_z]
    data.remap_lg_pos_z = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [remap_lg_vel_x]
    data.remap_lg_vel_x = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [remap_lg_vel_y]
    data.remap_lg_vel_y = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [remap_lg_vel_z]
    data.remap_lg_vel_z = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.assignment.length;
    length += 4 * object.remap_lg_pos_x.length;
    length += 4 * object.remap_lg_pos_y.length;
    length += 4 * object.remap_lg_pos_z.length;
    length += 4 * object.remap_lg_vel_x.length;
    length += 4 * object.remap_lg_vel_y.length;
    length += 4 * object.remap_lg_vel_z.length;
    return length + 30;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_utils/RemapLocalGoalList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '94d1962f185de6a5a18e3c7557706328';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 guard_drone_id
    
    uint32[] assignment
    
    float32[] remap_lg_pos_x
    float32[] remap_lg_pos_y
    float32[] remap_lg_pos_z
    
    float32[] remap_lg_vel_x
    float32[] remap_lg_vel_y
    float32[] remap_lg_vel_z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RemapLocalGoalList(null);
    if (msg.guard_drone_id !== undefined) {
      resolved.guard_drone_id = msg.guard_drone_id;
    }
    else {
      resolved.guard_drone_id = 0
    }

    if (msg.assignment !== undefined) {
      resolved.assignment = msg.assignment;
    }
    else {
      resolved.assignment = []
    }

    if (msg.remap_lg_pos_x !== undefined) {
      resolved.remap_lg_pos_x = msg.remap_lg_pos_x;
    }
    else {
      resolved.remap_lg_pos_x = []
    }

    if (msg.remap_lg_pos_y !== undefined) {
      resolved.remap_lg_pos_y = msg.remap_lg_pos_y;
    }
    else {
      resolved.remap_lg_pos_y = []
    }

    if (msg.remap_lg_pos_z !== undefined) {
      resolved.remap_lg_pos_z = msg.remap_lg_pos_z;
    }
    else {
      resolved.remap_lg_pos_z = []
    }

    if (msg.remap_lg_vel_x !== undefined) {
      resolved.remap_lg_vel_x = msg.remap_lg_vel_x;
    }
    else {
      resolved.remap_lg_vel_x = []
    }

    if (msg.remap_lg_vel_y !== undefined) {
      resolved.remap_lg_vel_y = msg.remap_lg_vel_y;
    }
    else {
      resolved.remap_lg_vel_y = []
    }

    if (msg.remap_lg_vel_z !== undefined) {
      resolved.remap_lg_vel_z = msg.remap_lg_vel_z;
    }
    else {
      resolved.remap_lg_vel_z = []
    }

    return resolved;
    }
};

module.exports = RemapLocalGoalList;
