import argparse

import rclpy
from rclpy.node import Node
from onvif import ONVIFCamera

import numpy as np

import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped

def euler_to_quaternion(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]

def quaternion_to_euler(x, y, z, w):
  import math
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  X = np.atan2(t0, t1)

  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  Y = np.asin(t2)

  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  Z = np.atan2(t3, t4)

  return X, Y, Z

class CameraNode(Node):
  def __init__(self, name, params, offset={'pan':22, 'tilt':0}):
    super().__init__(name, namespace=name)

    self.name = name
    self.offset = offset

    # setup transform broadcaster
    self.transform_broadcaster = tf2_ros.TransformBroadcaster(self)

    # setup PTZ
    self._camera = ONVIFCamera(params['address'], params['port'], params['user'], params['password'])
    self._media = self._camera.create_media_service()
    self._ptz = self._camera.create_ptz_service()

    media_profile = self._media.GetProfiles()[0]
    self._profile_token = media_profile.token
    self._config_token = media_profile.PTZConfiguration.token
    
    # get PTZ move range and store in parameters
    request = self._ptz.create_type('GetConfigurationOptions')
    request.ConfigurationToken = media_profile.PTZConfiguration.token
    ptz_configuration_options = self._ptz.GetConfigurationOptions(request)

    self.declare_parameter("pan_max")
    self.declare_parameter("pan_min")
    self.declare_parameter("tilt_max")
    self.declare_parameter("tilt_min")

    XMAX = rclpy.parameter.Parameter(
      'pan_max',
      rclpy.Parameter.Type.DOUBLE,
      ptz_configuration_options.Spaces.AbsolutePanTiltPositionSpace[0].XRange.Max
    )

    XMIN = rclpy.parameter.Parameter(
      'pan_min',
      rclpy.Parameter.Type.DOUBLE,
      ptz_configuration_options.Spaces.AbsolutePanTiltPositionSpace[0].XRange.Min
    )

    YMAX = rclpy.parameter.Parameter(
      'tilt_max',
      rclpy.Parameter.Type.DOUBLE,
      ptz_configuration_options.Spaces.AbsolutePanTiltPositionSpace[0].YRange.Max
    )

    YMIN = rclpy.parameter.Parameter(
      'tilt_min',
      rclpy.Parameter.Type.DOUBLE,
      ptz_configuration_options.Spaces.AbsolutePanTiltPositionSpace[0].YRange.Min
    )

    self.set_parameters([XMAX, XMIN, YMAX, YMIN])

    # initialize self.pos
    self.update_pose()

    
    self._timer = self.create_timer(0.5, self.update_pose) # update every 0.5 sec

    # create control subscriber
    self._subscriber = self.create_subscription(
      PoseStamped,
      'cmd_abs_move',
      self.handle_cmd_abs_move,
      10
    )

  def update_pose(self):
    '''
    Get camera position and broadcast transform
    '''
    pos = self._ptz.GetStatus({'ProfileToken': self._profile_token}).Position
    pt = self.normal_to_radian(pos.PanTilt)

    self.pos = {
      'pan': pt['x'],
      'tilt': pt['y'],
      'zoom': pos.Zoom.x
    }

    self.zoom = pos.Zoom.x

    t = TransformStamped()
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'robot'
    t.child_frame_id = self.name
    q = euler_to_quaternion(0.0, pt['y'], pt['x'])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    self.transform_broadcaster.sendTransform(t)
  

  def handle_cmd_abs_move(self, msg:PoseStamped):
    '''
    Handle absolute move commands.
    '''
    q = msg.pose.orientation
    zoom = msg.pose.position.z
    _, tilt, pan = quaternion_to_euler(q.x, q.y, q.z, q.w)
    self.absolute_move(pan=pan, tilt=tilt, zoom=zoom)
    
  def absolute_move(self, pan=None, tilt=None, zoom=None):
    """
    Move camera to absolute position.

    Params:
      pan - Pan position in radians
      tilt - tilt position in radians
      zoom - Zoom position from 0.0 to 1.0
    """

    self._ptz.AbsoluteMove({
      'ProfileToken': self._profile_token,
      'Position': {
        'PanTilt': radian_to_normal({
          'x': pan if pan is not None else self.pos['pan'],
          'y': tilt if tilt is not None else self.pos['tilt'],
        }),
        'Zoom': {'x': zoom if zoom is not None else self.pos['zoom']}
      }
    })

  def continuous_move(self, pan_speed, tilt_speed, zoom_speed):
    # TODO
    raise NotImplementedError

  def normal_to_radian(self, vec):
    """
    Converts from camera normalized form to radians.
    """
    return {
      'x': (vec['x']+1.0)*180+self.offset['pan'],
      'y': -(vec['y']-1.0)*45+self.offset['tilt']
    }

  def radian_to_normal(self, vec):
    """
    Converts from radians to camera normalized form.
    """
    return {
      'x': 2*((vec['x']-self.offset['pan'])%360)/360 - 1.0,
      'y':-2*((vec['y']-self.offset['tilt'])%90)/90 + 1.0
    }

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('address', help='IP address of camera')
  parser.add_argument('-p', '--port', type=int, help='Port of camera', default=80)
  parser.add_argument('-u', '--user', help='Username for authentication', default='')
  parser.add_argument('-a', '--password', help='Password for authentication', default='')
  parser.add_argument('-n', '--name', help='Camera name for identification', default='onvif_camera')

  args, _ = parser.parse_known_args()

  rclpy.init()
  try:
    camera_node = CameraNode(
      name=args.name,
      params={
        'address': args.address,
        'port': args.port,
        'user': args.user,
        'password': args.password
      }
    )
    try:
      rclpy.spin(camera_node)
    finally:
      camera_node.destroy_node()
  finally:
    rclpy.shutdown()


if __name__ == '__main__':
  main()