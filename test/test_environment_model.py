import pytest
import rclpy
from unittest.mock import MagicMock, patch
from tf2_ros import TransformException
import numpy as np
from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from custom_msgs.msg import DetectedObjectsPositionArray, DetectedObject
from environment_model.environment_model import EnvironmentModel


@pytest.fixture
def node():
    """Fixture to create and destroy the EnvironmentModel node."""
    rclpy.init(args=None)  # Initialize ROS
    node = EnvironmentModel()
    yield node
    node.destroy_node()
    rclpy.shutdown()       # Cleanly shutdown ROS


# TC_EM001
def test_detections_callback_with_detections(node):
    # Test Step 1 ----------------------------------------------------------------------
    msg = Detection2DArray()
    msg.header.stamp.sec = 1
    msg.header.stamp.nanosec = 0
    node.detections_callback(msg)
    assert node.flg_calc_xyz == 0
    assert node.bbox == []
    
    # Test Step 2 ----------------------------------------------------------------------
    msg = Detection2DArray()
    det = Detection2D()
    det.bbox = BoundingBox2D()
    det.bbox.center.position.x = 320.0
    det.bbox.center.position.y = 240.0
    det.bbox.size_x = 50.0
    det.bbox.size_y = 60.0
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = "car"
    hyp.hypothesis.score = 0.9
    det.results.append(hyp)
    msg.detections.append(det)
    msg.header.stamp.sec = 100
    msg.header.stamp.nanosec = 200000000

    node.detections_callback(msg)

    assert node.flg_calc_xyz == 1
    assert len(node.bbox) == 1
    assert node.class_ids_confidence[0][0] == "car"
    


# TC_EM002
def test_odom_callback_sets_yaw(node):
    msg = Odometry()
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.7071
    msg.pose.pose.orientation.w = 0.7071
    node.odom_callback(msg)
    assert pytest.approx(node.yaw, rel=1e-3) == np.pi/2


# TC_EM003
def test_broadcast_frame_publishes_tf(node):
    # Test Step 1 ----------------------------------------------------------------------
    node.br.sendTransform = MagicMock()
    node.yaw = np.pi / 4
    node.broadcast_frame()
    node.br.sendTransform.assert_called_once()

    # Test Step 2 ----------------------------------------------------------------------
    t_sent = node.br.sendTransform.call_args[0][0]
    assert t_sent.child_frame_id == 'v2x_frame'
    assert t_sent.transform.translation.x == pytest.approx(0.65)
    assert t_sent.transform.translation.y == pytest.approx(0.0)
    assert t_sent.transform.translation.z == pytest.approx(-0.07)

    import tf_transformations
    q = [t_sent.transform.rotation.x, t_sent.transform.rotation.y, 
         t_sent.transform.rotation.z, t_sent.transform.rotation.w]
    roll, pitch, yaw = tf_transformations.euler_from_quaternion(q)
    assert roll == pytest.approx(0.0, abs=1e-3)
    assert pitch == pytest.approx(0.0, abs=1e-3)
    assert yaw == pytest.approx(-node.yaw, abs=1e-3)


# TC_EM004
@patch("environment_model.environment_model.do_transform_point")
def test_depth_callback_1(mock_do_transform, node):
    # Test Step 1 ----------------------------------------------------------------------
    depth = Image()
    depth.height = 480
    depth.width = 640
    depth.step = 1280
    depth.data = np.zeros((480, 640), dtype=np.uint16).tobytes()
    node.depth_callback(depth)
    assert node.bbox == []
    assert node.flg_calc_xyz == 0
    
    # Test Step 2 ----------------------------------------------------------------------
    transform = TransformStamped()
    node.tf_buffer = MagicMock() 
    node.tf_buffer.lookup_transform.return_value = transform

    mock_do_transform.return_value = PointStamped()
    mock_do_transform.return_value.point.x = 1.0
    mock_do_transform.return_value.point.y = 2.0
    mock_do_transform.return_value.point.z = 3.0

    # prepare depth image with valid depth
    depth = Image()
    depth.height = 480
    depth.width = 640
    depth.step = 1280
    depth_data = np.zeros((480, 640), dtype=np.uint16)
    
    depth.data = depth_data.tobytes()
    
    # fake bbox and state
    node.detection_time = 123.456  # float value before calling depth_callback
    node.bbox = [[320, 240, 50, 50]]
    node.class_ids_confidence = [["car", 90]]
    node.publish_objects_position.publish = MagicMock()
    node.depth_callback(depth)
    node.publish_objects_position.publish.assert_called_once()
    assert len(node.valid_depths) == 0

    
# TC_EM005
@patch("environment_model.environment_model.do_transform_point")
def test_depth_callback_2(mock_do_transform, node):
    # Test Step 1 ----------------------------------------------------------------------
    transform = TransformStamped()
    node.tf_buffer = MagicMock() 
    node.tf_buffer.lookup_transform.return_value = transform

    mock_do_transform.return_value = PointStamped()
    mock_do_transform.return_value.point.x = 1.0
    mock_do_transform.return_value.point.y = 2.0
    mock_do_transform.return_value.point.z = 3.0

    # prepare depth image with valid depth
    depth = Image()
    depth.height = 480
    depth.width = 640
    depth.step = 1280
    depth_data = np.zeros((480, 640), dtype=np.uint16)
   
    for i in range(240, 260):  # bbox area near center
        for j in range(300, 340):
            if j < 310:
                depth_data[i][j] = 900   # cluster 1
            else:
                depth_data[i][j] = 1300  # cluster 2
            
    depth.data = depth_data.tobytes()

    #fake bbox and state
    node.detection_time = 123.456  # float value before calling depth_callback
    node.bbox = [[320, 240, 50, 50]]
    node.class_ids_confidence = [["car", 90]]
    node.flg_calc_xyz = 1
    node.publish_objects_position.publish = MagicMock()

    node.depth_callback(depth)

    node.publish_objects_position.publish.assert_called_once()
    assert len(node.valid_depths) != 0
    assert len(set(node.labels)) == 2
    assert node.dominant_depth == 1.3
    assert node.X == pytest.approx(-0.00171411727, rel=1e-6)
    assert node.Y == pytest.approx(-0.01673656583, rel=1e-6)
    assert node.Z == 1.3
    
    # Test Step 2 ----------------------------------------------------------------------
    depth_data = np.ones((480, 640), dtype=np.uint16) * 1000
    depth.data = depth_data.tobytes()
    # fake bbox and state
    node.detection_time = 123.456  # float value before calling depth_callback
    node.bbox = [[320, 240, 50, 50]]
    node.class_ids_confidence = [["car", 90]]
    node.flg_calc_xyz = 1
    node.publish_objects_position.publish = MagicMock()

    node.depth_callback(depth)
    
    node.publish_objects_position.publish.assert_called_once()
    assert len(set(node.labels)) == 1
    assert node.dominant_depth == 1.0
    
    # Test Step 3 ----------------------------------------------------------------------
    mock_do_transform.side_effect = TransformException("fake transform error")
    
    # prepare dummy depth image with one valid depth
    depth = Image()
    depth.height = 10
    depth.width = 10
    depth.step = depth.width * 2
    depth_data = np.ones((depth.height, depth.width), dtype=np.uint16) * 1000
    depth.data = depth_data.tobytes()
    
    # set fake bbox and class
    node.bbox = [[5, 5, 4, 4]]  # some bbox inside the 10x10 image
    node.class_ids_confidence = [["car", 90]]
    node.flg_calc_xyz = 1
    node.detection_time = 123.456
    node.publish_objects_position.publish = MagicMock()
    
    # provide a dummy transform so lookup_transform succeeds
    node.tf_buffer = MagicMock()
    node.tf_buffer.lookup_transform.return_value = MagicMock()

    # call depth_callback, this should hit the except block
    node.depth_callback(depth)
    
    # verify that bbox is cleared after processing
    assert node.bbox == []


# ---------------- main() ----------------
@patch("environment_model.environment_model.EnvironmentModel")
@patch("rclpy.shutdown")
@patch("rclpy.spin")
@patch("rclpy.init")
def test_main_lifecycle(mock_init, mock_spin, mock_shutdown, mock_env_node):
    from environment_model.environment_model import main
    main()  # this will now use the mocked EnvironmentModel

    # assert ROS lifecycle functions are called
    mock_init.assert_called_once()
    mock_spin.assert_called_once()
    mock_shutdown.assert_called_once()
    
    # assert the node object was created
    mock_env_node.assert_called_once()


