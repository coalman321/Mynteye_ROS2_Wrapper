import launch
from launch.actions import DeclareLaunchArgument
import launch.substitutions
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration as LC
import math

respawn = False

def makeStaticTFArgs(isOptical, childFrame):
    if(isOptical):
        return ['0', '0', '0', str(-math.pi/2), '0', str(-math.pi/2), LC('base_frame'), LC(childFrame)]

    return ['0', '0', '0', str(-math.pi/2), '0', str(math.pi), LC('base_frame'), LC(childFrame)]


def generate_launch_description():
    return launch.LaunchDescription([
        # declare the launch args to read for this file

        # Device Configuration
        DeclareLaunchArgument(
            'dev_index',
            default_value='0',
            description='Device Index'),
        DeclareLaunchArgument(
            'framerate',
            default_value='30',
            description='Camera stream framerate'),
        DeclareLaunchArgument(
            'dev_mode',
            default_value='2',
            description='Mode of the camera device. color=0, depth=1, all=2'),
        DeclareLaunchArgument(
            'color_mode',
            default_value='0',
            description='Color mode for the color stream. raw=0, rectified=1'),
        DeclareLaunchArgument(
            'depth_mode',
            default_value='0',
            description='Mode for depth stream. raw=0, gray=1, colorful=2'),
        DeclareLaunchArgument(
            'stream_mode',
            default_value='3',
            description='Resolution of the main stream. 640X480=0, 1280x480=1, 1280x720=2, 2560x720=3'),
        DeclareLaunchArgument(
            'color_stream_format',
            default_value='1',
            description='Image format to use. mjpeg=0, yuyv=1'),
        DeclareLaunchArgument(
            'depth_stream_format',
            default_value='1',
            description='Image format to use. mjpeg=0, yuyv=1'),
        DeclareLaunchArgument(
            'depth_type',
            default_value='0',
            description='Image format to use. mono16=0, 16uc1=1'),
        DeclareLaunchArgument(
            'state_ae',
            default_value='true',
            description='Enable auto exposure'),
        DeclareLaunchArgument(
            'state_awb',
            default_value='true',
            description='Enable auto white balance'),
        DeclareLaunchArgument(
            'ir_intensity',
            default_value='4',
            description='Infrared Intensity for the IR LEDS'),
        DeclareLaunchArgument(
            'ir_depth_only',
            default_value='false',
            description='USE only IR for calculating depth'),
        DeclareLaunchArgument(
            'points_frequency',
            default_value='10.0',
            description='Generating points frequency, make slower than framerate'),
        DeclareLaunchArgument(
            'points_factor',
            default_value='1000.0',
            description='Points display z distance scale factor'),
        DeclareLaunchArgument(
            'gravity',
            default_value='9.8',
            description='Gravity constant'),

        # Frame name assignment args
        DeclareLaunchArgument(
            'camera_name',
            default_value='mynteye',
            description='Name of the camera'),
        DeclareLaunchArgument(
            'base_frame',
            default_value=[LC('camera_name'), '_link_frame'],
            description='Base link frame name'),
        DeclareLaunchArgument(
            'left_mono_frame',
            default_value=[LC('camera_name'), '_left_mono_frame'],
            description='Left mono frame name'),
        DeclareLaunchArgument(
            'left_color_frame',
            default_value=[LC('camera_name'), '_left_color_frame'],
            description='Left color frame name'),
        DeclareLaunchArgument(
            'right_mono_frame',
            default_value=[LC('camera_name'), '_right_mono_frame'],
            description='Right mono frame name'),
        DeclareLaunchArgument(
            'right_color_frame',
            default_value=[LC('camera_name'), '_right_color_frame'],
            description='Right color frame name'),
        DeclareLaunchArgument(
            'depth_frame',
            default_value=[LC('camera_name'), '_depth_frame'],
            description='Depth frame name'),
        DeclareLaunchArgument(
            'points_frame',
            default_value=[LC('camera_name'), '_points_frame'],
            description='Points frame name'),
        DeclareLaunchArgument(
            'imu_frame',
            default_value=[LC('camera_name'), '_imu_frame'],
            description='IMU Frame name'),
        DeclareLaunchArgument(
            'temp_frame',
            default_value=[LC('camera_name'), '_temp_frame'],
            description='Temperature frame name'),
        DeclareLaunchArgument(
            'imu_frame_processed',
            default_value=[LC('camera_name'), '_imu_frame_processed'],
            description='Processed IMU frame name'),

        # Topic name assignments
        DeclareLaunchArgument(
            'left_mono_topic',
            default_value=[LC('camera_name'), '/left/image_mono'],
            description='Left mono topic name'),
        DeclareLaunchArgument(
            'left_color_topic',
            default_value=[LC('camera_name'), '/left/image_color'],
            description='Left color topic name'),
        DeclareLaunchArgument(
            'right_mono_topic',
            default_value=[LC('camera_name'), '/right/image_mono'],
            description='Right mono topic name'),
        DeclareLaunchArgument(
            'right_color_topic',
            default_value=[LC('camera_name'), '/right/image_color'],
            description='Right color topic name'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value=[LC('camera_name'), '/depth/image_raw'],
            description='Depth topic name'),
        DeclareLaunchArgument(
            'points_topic',
            default_value=[LC('camera_name'), '/points/data_raw'],
            description='Points topic name'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value=[LC('camera_name'), '/imu/data_raw'],
            description='IMU topic name'),
        DeclareLaunchArgument(
            'temp_topic',
            default_value=[LC('camera_name'), '/temp/data_raw'],
            description='Temperature topic name'),
        DeclareLaunchArgument(
            'imu_processed_topic',
            default_value=[LC('camera_name'), '/imu/data_raw_processed'],
            description='Processed IMU topic name'),

        # Mesh files
        DeclareLaunchArgument(
            'mesh_file',
            default_value='D-0315.obj',
            description='Mesh file name to use'),

        # TF publishers for image data
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[LC('camera_name'), 'b2l_mono'],
            respawn=respawn,
            output='screen',
            arguments=makeStaticTFArgs(True, 'left_mono_frame')
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[LC('camera_name'), 'b2l_Color'],
            respawn=respawn,
            output='screen',
            arguments=makeStaticTFArgs(True, 'left_color_frame')
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[LC('camera_name'), 'b2r_mono'],
            respawn=respawn,
            output='screen',
            arguments=makeStaticTFArgs(True, 'right_mono_frame')
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[LC('camera_name'), 'b2r_color'],
            respawn=respawn,
            output='screen',
            arguments=makeStaticTFArgs(True, 'right_color_frame')
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[LC('camera_name'), 'b2d'],
            respawn=respawn,
            output='screen',
            arguments=makeStaticTFArgs(True, 'depth_frame')
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[LC('camera_name'), 'b2p'],
            respawn=respawn,
            output='screen',
            arguments=makeStaticTFArgs(True, 'points_frame')
        ),

        # TF publishers for sensor data
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[LC('camera_name'), 'b2imu'],
            respawn=respawn,
            output='screen',
            arguments=makeStaticTFArgs(False, 'imu_frame')
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[LC('camera_name'), 'b2temp'],
            respawn=respawn,
            output='screen',
            arguments=makeStaticTFArgs(False, 'temp_frame')
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[LC('camera_name'), 'b2imu_processed'],
            respawn=respawn,
            output='screen',
            arguments=makeStaticTFArgs(False, 'imu_frame_processed')
        ),

        # create the nodes
        Node(
            package='mynteye_wrapper_d',
            executable='mynteye_wrapper_d_node',
            name=LC('camera_name'),
            respawn=respawn,
            output='screen',

            # use the parameters on the node
            parameters=[
                {'dev_index': LC('dev_index')},
                {'framerate': LC('framerate')},
                {'dev_mode': LC('dev_mode')},
                {'color_mode': LC('color_mode')},
                {'depth_mode': LC('depth_mode')},
                {'stream_mode': LC('stream_mode')},
                {'color_stream_format': LC('color_stream_format')},
                {'depth_stream_format': LC('depth_stream_format')},
                {'state_ae': LC('state_ae')},
                {'state_awb': LC('state_awb')},
                {'ir_intensity': LC('ir_intensity')},
                {'ir_depth_only': LC('ir_depth_only')},
                {'points_factor': LC('points_factor')},
                {'points_frequency': LC('points_frequency')},
                {'gravity': LC('gravity')},
                {'depth_type': LC('depth_type')},
                {'base_frame': LC('base_frame')},
                {'left_mono_frame': LC('left_mono_frame')},
                {'left_color_frame': LC('left_color_frame')},
                {'right_mono_frame': LC('right_mono_frame')},
                {'right_color_frame': LC('right_color_frame')},
                {'depth_frame': LC('depth_frame')},
                {'points_frame': LC('points_frame')},
                {'imu_frame': LC('imu_frame')},
                {'temp_frame': LC('temp_frame')},
                {'imu_frame_processed': LC('imu_frame_processed')},
                {'left_mono_topic': LC('left_mono_topic')},
                {'left_color_topic': LC('left_color_topic')},
                {'right_mono_topic': LC('right_mono_topic')},
                {'right_color_topic': LC('right_color_topic')},
                {'depth_topic': LC('depth_topic')},
                {'points_topic': LC('points_topic')},
                {'imu_topic': LC('imu_topic')},
                {'temp_topic': LC('temp_topic')},
                {'imu_processed_topic': LC('imu_processed_topic')},
                {'mesh_file': LC('mesh_file')},
            ]
        )
    ])
