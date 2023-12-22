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

class SwarmGlobalPathList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.guard_drone_id = null;
      this.path_num = null;
      this.swarm_global_path_x = null;
      this.swarm_global_path_y = null;
      this.swarm_global_path_z = null;
      this.swarm_global_path_scale = null;
    }
    else {
      if (initObj.hasOwnProperty('guard_drone_id')) {
        this.guard_drone_id = initObj.guard_drone_id
      }
      else {
        this.guard_drone_id = 0;
      }
      if (initObj.hasOwnProperty('path_num')) {
        this.path_num = initObj.path_num
      }
      else {
        this.path_num = 0;
      }
      if (initObj.hasOwnProperty('swarm_global_path_x')) {
        this.swarm_global_path_x = initObj.swarm_global_path_x
      }
      else {
        this.swarm_global_path_x = [];
      }
      if (initObj.hasOwnProperty('swarm_global_path_y')) {
        this.swarm_global_path_y = initObj.swarm_global_path_y
      }
      else {
        this.swarm_global_path_y = [];
      }
      if (initObj.hasOwnProperty('swarm_global_path_z')) {
        this.swarm_global_path_z = initObj.swarm_global_path_z
      }
      else {
        this.swarm_global_path_z = [];
      }
      if (initObj.hasOwnProperty('swarm_global_path_scale')) {
        this.swarm_global_path_scale = initObj.swarm_global_path_scale
      }
      else {
        this.swarm_global_path_scale = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SwarmGlobalPathList
    // Serialize message field [guard_drone_id]
    bufferOffset = _serializer.int16(obj.guard_drone_id, buffer, bufferOffset);
    // Serialize message field [path_num]
    bufferOffset = _serializer.int16(obj.path_num, buffer, bufferOffset);
    // Serialize message field [swarm_global_path_x]
    bufferOffset = _arraySerializer.float32(obj.swarm_global_path_x, buffer, bufferOffset, null);
    // Serialize message field [swarm_global_path_y]
    bufferOffset = _arraySerializer.float32(obj.swarm_global_path_y, buffer, bufferOffset, null);
    // Serialize message field [swarm_global_path_z]
    bufferOffset = _arraySerializer.float32(obj.swarm_global_path_z, buffer, bufferOffset, null);
    // Serialize message field [swarm_global_path_scale]
    bufferOffset = _arraySerializer.float32(obj.swarm_global_path_scale, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SwarmGlobalPathList
    let len;
    let data = new SwarmGlobalPathList(null);
    // Deserialize message field [guard_drone_id]
    data.guard_drone_id = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [path_num]
    data.path_num = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [swarm_global_path_x]
    data.swarm_global_path_x = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [swarm_global_path_y]
    data.swarm_global_path_y = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [swarm_global_path_z]
    data.swarm_global_path_z = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [swarm_global_path_scale]
    data.swarm_global_path_scale = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.swarm_global_path_x.length;
    length += 4 * object.swarm_global_path_y.length;
    length += 4 * object.swarm_global_path_z.length;
    length += 4 * object.swarm_global_path_scale.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_utils/SwarmGlobalPathList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '015139dac170a0f9242e5a66432e43ee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 guard_drone_id
    int16 path_num
    
    float32[] swarm_global_path_x
    float32[] swarm_global_path_y
    float32[] swarm_global_path_z
    float32[] swarm_global_path_scale
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SwarmGlobalPathList(null);
    if (msg.guard_drone_id !== undefined) {
      resolved.guard_drone_id = msg.guard_drone_id;
    }
    else {
      resolved.guard_drone_id = 0
    }

    if (msg.path_num !== undefined) {
      resolved.path_num = msg.path_num;
    }
    else {
      resolved.path_num = 0
    }

    if (msg.swarm_global_path_x !== undefined) {
      resolved.swarm_global_path_x = msg.swarm_global_path_x;
    }
    else {
      resolved.swarm_global_path_x = []
    }

    if (msg.swarm_global_path_y !== undefined) {
      resolved.swarm_global_path_y = msg.swarm_global_path_y;
    }
    else {
      resolved.swarm_global_path_y = []
    }

    if (msg.swarm_global_path_z !== undefined) {
      resolved.swarm_global_path_z = msg.swarm_global_path_z;
    }
    else {
      resolved.swarm_global_path_z = []
    }

    if (msg.swarm_global_path_scale !== undefined) {
      resolved.swarm_global_path_scale = msg.swarm_global_path_scale;
    }
    else {
      resolved.swarm_global_path_scale = []
    }

    return resolved;
    }
};

module.exports = SwarmGlobalPathList;
