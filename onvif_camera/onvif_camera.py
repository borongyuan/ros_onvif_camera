import rclpy
from rclpy.node import Node
from onvif import ONVIFCamera
from onvif.exceptions import *
from zeep.transports import Transport

import numpy as np

import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3Stamped

from geometry_helper.geometry_helper import *


class CameraNode(Node):
    def __init__(self):
        super().__init__('onvif_camera', namespace='onvif_camera')

        self.declare_parameter('host', '192.168.1.64')
        self.declare_parameter('port', 80)
        self.declare_parameter('user', 'admin')
        self.declare_parameter('passwd', 'hik12345')
        self.declare_parameter(
            'wsdl_dir', '/usr/local/lib/python3.8/site-packages/wsdl')

        host = self.get_parameter('host')
        port = self.get_parameter('port')
        user = self.get_parameter('user')
        passwd = self.get_parameter('passwd')
        wsdl_dir = self.get_parameter('wsdl_dir')

        # setup PTZ
        self.get_logger().info(
            f'Connecting to camera {host.value}:{port.value}')
        self._camera = ONVIFCamera(
            host=host.value,
            port=port.value,
            user=user.value,
            passwd=passwd.value,
            wsdl_dir=wsdl_dir.value)
        self.get_logger().info('Configuring services')
        self._media = self._camera.create_media_service()
        self._ptz = self._camera.create_ptz_service()
        self.get_logger().info('Configured')

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
        pos = self._ptz.GetStatus(
            {'ProfileToken': self._profile_token}).Position
        pt = self.normal_to_radian(pos.PanTilt)

        self.pos = {
            'pan': pt['x'],
            'tilt': pt['y'],
            'zoom': pos.Zoom.x
        }

        self.get_logger().info('Initial position: %s' % (repr(self.pos)))

        # setup transform broadcaster
        self._tf_br = tf2_ros.TransformBroadcaster(self)
        self._timer = self.create_timer(
            0.5, self.update_pose)  # update every 0.5 sec

        # create control subscriber
        self._cmd_abs_subscriber = self.create_subscription(
            PoseStamped,
            '/cmd_abs_move',
            self.handle_cmd_abs_move,
            10
        )

        self._cmd_continuous_subscriber = self.create_subscription(
            Vector3Stamped,
            'cmd_continuous_move',
            self.handle_cmd_continuous_move,
            10
        )

    def update_pose(self):
        '''
        Get camera position and broadcast transform
        '''
        pos = self._ptz.GetStatus(
            {'ProfileToken': self._profile_token}).Position
        pt = self.normal_to_radian(pos.PanTilt)

        self.pos = {
            'pan': pt['x'],
            'tilt': pt['y'],
            'zoom': pos.Zoom.x
        }

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'ptz'
        t.child_frame_id = 'onvif_camera'
        q = euler_to_quaternion(0.0, pt['y'], pt['x'])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self._tf_br.sendTransform(t)

    def handle_cmd_abs_move(self, msg: PoseStamped):
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

        # self.get_logger().info('Moving to: %f, %f, %f'%(
        #   pan if pan is not None else self.pos['pan'],
        #   tilt if tilt is not None else self.pos['tilt'],
        #   zoom if zoom is not None else self.pos['zoom']))

        self.get_logger().info(
            'Moving to: %s' % (repr(self.radian_to_normal({
                'x': pan if pan is not None else self.pos['pan'],
                'y': tilt if tilt is not None else self.pos['tilt'],
            }))))

        self._ptz.AbsoluteMove({
            'ProfileToken': self._profile_token,
            'Position': {
                'PanTilt': self.radian_to_normal({
                    'x': pan if pan is not None else self.pos['pan'],
                    'y': tilt if tilt is not None else self.pos['tilt'],
                }),
                'Zoom': {'x': zoom if zoom is not None else self.pos['zoom']}
            }
        })

    def handle_cmd_continuous_move(self, msg):
        self.continuous_move(msg.vector.x, msg.vector.y, msg.vector.z)

    def continuous_move(self, pan_speed=0.0, tilt_speed=0.0, zoom_speed=0.0):
        """
        Move camera by specifying speed.
        """

        self._ptz.AbsoluteMove({
            'ProfileToken': self._profile_token,
            'Velocity': {
                'PanTilt': self.radian_to_normal({
                    'x': pan_speed,
                    'y': tilt_speed,
                }),
                'Zoom': {'x': zoom_speed}
            }
        })

    def normal_to_radian(self, vec):
        """
        Converts from camera normalized form to radians.
        """
        return {
            'x': (vec['x']+1.0)*(np.pi),
            'y': -(vec['y']-1.0)*(np.pi/4)
        }

    def radian_to_normal(self, vec):
        """
        Converts from radians to camera normalized form.
        """
        return {
            'x': round(2*(vec['x'] % (2*np.pi))/(2*np.pi) - 1.0, 4),
            'y': round(-2*(vec['y'] % (np.pi/2))/(np.pi/2) + 1.0, 4)
        }


def main():
    rclpy.init()

    try:
        camera_node = CameraNode()
        rclpy.spin(camera_node)
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
