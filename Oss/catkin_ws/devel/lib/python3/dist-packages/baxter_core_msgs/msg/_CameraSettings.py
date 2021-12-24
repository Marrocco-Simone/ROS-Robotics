# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from baxter_core_msgs/CameraSettings.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import baxter_core_msgs.msg

class CameraSettings(genpy.Message):
  _md5sum = "d133bef4a3bd9a6e490a5dc91d20f429"
  _type = "baxter_core_msgs/CameraSettings"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32           width
int32           height
float32         fps
CameraControl[] controls

================================================================================
MSG: baxter_core_msgs/CameraControl
int32   id
int32   value

int32 CAMERA_CONTROL_EXPOSURE=100
int32 CAMERA_CONTROL_GAIN=101
int32 CAMERA_CONTROL_WHITE_BALANCE_R=102
int32 CAMERA_CONTROL_WHITE_BALANCE_G=103
int32 CAMERA_CONTROL_WHITE_BALANCE_B=104
int32 CAMERA_CONTROL_WINDOW_X=105
int32 CAMERA_CONTROL_WINDOW_Y=106
int32 CAMERA_CONTROL_FLIP=107
int32 CAMERA_CONTROL_MIRROR=108
int32 CAMERA_CONTROL_RESOLUTION_HALF=109
"""
  __slots__ = ['width','height','fps','controls']
  _slot_types = ['int32','int32','float32','baxter_core_msgs/CameraControl[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       width,height,fps,controls

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CameraSettings, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.width is None:
        self.width = 0
      if self.height is None:
        self.height = 0
      if self.fps is None:
        self.fps = 0.
      if self.controls is None:
        self.controls = []
    else:
      self.width = 0
      self.height = 0
      self.fps = 0.
      self.controls = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_2if().pack(_x.width, _x.height, _x.fps))
      length = len(self.controls)
      buff.write(_struct_I.pack(length))
      for val1 in self.controls:
        _x = val1
        buff.write(_get_struct_2i().pack(_x.id, _x.value))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.controls is None:
        self.controls = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.width, _x.height, _x.fps,) = _get_struct_2if().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.controls = []
      for i in range(0, length):
        val1 = baxter_core_msgs.msg.CameraControl()
        _x = val1
        start = end
        end += 8
        (_x.id, _x.value,) = _get_struct_2i().unpack(str[start:end])
        self.controls.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2if().pack(_x.width, _x.height, _x.fps))
      length = len(self.controls)
      buff.write(_struct_I.pack(length))
      for val1 in self.controls:
        _x = val1
        buff.write(_get_struct_2i().pack(_x.id, _x.value))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.controls is None:
        self.controls = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.width, _x.height, _x.fps,) = _get_struct_2if().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.controls = []
      for i in range(0, length):
        val1 = baxter_core_msgs.msg.CameraControl()
        _x = val1
        start = end
        end += 8
        (_x.id, _x.value,) = _get_struct_2i().unpack(str[start:end])
        self.controls.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2i = None
def _get_struct_2i():
    global _struct_2i
    if _struct_2i is None:
        _struct_2i = struct.Struct("<2i")
    return _struct_2i
_struct_2if = None
def _get_struct_2if():
    global _struct_2if
    if _struct_2if is None:
        _struct_2if = struct.Struct("<2if")
    return _struct_2if
