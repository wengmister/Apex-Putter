import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.msg import PlanningScene as PlanningMsg
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


class PlanningSceneClass:
    def __init__(self, node: Node):
        """
        Initializes the PlanningScene with an existing ROS 2 node.

        Args:
            node: The running ROS node used to interface with ROS.
        """
        self.node = node
        self.node.get_logger().info("Initialize PlanningScene")
        self.apply_planning_scene_client = self.node.create_client(
            ApplyPlanningScene, '/apply_planning_scene',
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Wait for the service to become available
        self.node.get_logger().info('Waiting for /apply_planning_scene service...')
        if not self.apply_planning_scene_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Service /apply_planning_scene not available.')
            raise RuntimeError('Service /apply_planning_scene not available.')

        self.node.get_logger().info('Service /apply_planning_scene is now available.')

        # Create the joint state subscriber
        self.joint_state_subscriber = None

    def done_callback(self, task):
        result = task.result()
        self.future.set_result(result)

    async def add_box_async(self, box_id, size, position, orientation=(0.0, 0.0, 0.0, 1.0), frame_id='base'):
        """
        Adds a box to the planning scene synchronously.
        """
        self.node.get_logger().info("add_box_async()")
        collision_object = CollisionObject()
        collision_object.id = box_id
        collision_object.header.frame_id = frame_id

        # Define the box shape
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)

        # Define the box pose
        box_pose = Pose()
        box_pose.position.x, box_pose.position.y, box_pose.position.z = position
        box_pose.orientation.x, box_pose.orientation.y, box_pose.orientation.z, box_pose.orientation.w = orientation

        collision_object.primitives = [box]
        collision_object.primitive_poses = [box_pose]
        collision_object.operation = CollisionObject.ADD

        # Apply the collision object to the planning scene
        result = await self._apply_collision_object(collision_object)
        return result

    def add_box(self, box_id, size, position, orientation=(0.0, 0.0, 0.0, 1.0), frame_id='base'):
        self.node.get_logger().info("add_box()")
        executor = rclpy.get_global_executor()

        if executor is None:
            raise RuntimeError(
                "No executor is running. Make sure rclpy.init() has been called")

        # Create a new future for this request
        self.future = Future()

        executor.create_task(
            self.add_box_async(
                box_id,
                size,
                position,
                orientation,
                frame_id
            )
        ).add_done_callback(self.done_callback)

        self.node.get_logger().info('Task done')

        return self.future

    async def remove_box_async(self, box_id, frame_id='base'):
        """
        Removes a box from the planning scene synchronously.
        """
        self.node.get_logger().info("remove_box_async()")
        collision_object = CollisionObject()
        collision_object.id = box_id
        collision_object.header.frame_id = frame_id
        collision_object.operation = CollisionObject.REMOVE

        # Apply the collision object to the planning scene
        result = await self._apply_collision_object(collision_object)
        return result

    async def add_sphere_async(self, sphere_id, radius, position, orientation=(0.0, 0.0, 0.0, 1.0), frame_id='base'):
        """
        Adds a sphere to the planning scene asynchronously.
        """
        self.node.get_logger().info("add_sphere_async()")

        collision_object = CollisionObject()
        collision_object.id = sphere_id
        collision_object.header.frame_id = frame_id

        # Define the sphere shape
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        # For a sphere, dimensions[0] is the radius
        sphere.dimensions = [radius]

        # Define the sphere pose
        sphere_pose = Pose()
        sphere_pose.position.x, sphere_pose.position.y, sphere_pose.position.z = position
        sphere_pose.orientation.x, sphere_pose.orientation.y, sphere_pose.orientation.z, sphere_pose.orientation.w = orientation

        collision_object.primitives = [sphere]
        collision_object.primitive_poses = [sphere_pose]
        collision_object.operation = CollisionObject.ADD

        # Apply the collision object to the planning scene
        result = await self._apply_collision_object(collision_object)
        return result

    def add_sphere(self, sphere_id, radius, position, orientation=(0.0, 0.0, 0.0, 1.0), frame_id='base'):
        """
        Adds a sphere to the planning scene synchronously.
        This method creates an async task and returns a Future that you can wait on.
        """
        self.node.get_logger().info("add_sphere()")
        executor = rclpy.get_global_executor()

        if executor is None:
            raise RuntimeError(
                "No executor is running. Make sure rclpy.init() has been called")

        # Create a new future for this request
        self.future = Future()

        executor.create_task(
            self.add_sphere_async(
                sphere_id,
                radius,
                position,
                orientation,
                frame_id
            )
        ).add_done_callback(self.done_callback)

        self.node.get_logger().info('Task done')

        return self.future


    def remove_box(self, box_id, frame_id='base'):
        self.node.get_logger().info("remove_box()")
        executor = rclpy.get_global_executor()

        if executor is None:
            raise RuntimeError(
                "No executor is running. Make sure rclpy.init() has been called")

        # Create a new future for this request
        self.future = Future()

        executor.create_task(
            self.remove_box_async(
                box_id,
                frame_id
            )
        ).add_done_callback(self.done_callback)

        self.node.get_logger().info('Task done')

        return self.future

    async def attach_object_async(self, object_id, link_name):
        """
        Attaches a collision object to the robot's end-effector synchronously.
        """
        self.node.get_logger().info("attach_object_async()")
        attached_object = AttachedCollisionObject()
        attached_object.link_name = link_name
        attached_object.object.id = object_id
        attached_object.object.operation = CollisionObject.ADD

        # Remove the object from the world collision objects
        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE

        planning_scene = PlanningMsg()
        planning_scene.is_diff = True
        planning_scene.robot_state.attached_collision_objects.append(
            attached_object)
        planning_scene.world.collision_objects.append(collision_object)

        result = await self._apply_planning_scene(planning_scene)
        return result

    def attach_object(self, object_id, link_name):
        self.node.get_logger().info("attach_object()")
        executor = rclpy.get_global_executor()

        if executor is None:
            raise RuntimeError(
                "No executor is running. Make sure rclpy.init() has been called")

        # Create a new future for this request
        self.future = Future()

        executor.create_task(
            self.attach_object_async(
                object_id,
                link_name
            )
        ).add_done_callback(self.done_callback)

        self.node.get_logger().info('Task done')

        return self.future

    async def detach_object_async(self, object_id, link_name):
        """
        Detaches a collision object from the robot's end-effector synchronously.
        """
        self.node.get_logger().info("detach_object_async()")
        attached_object = AttachedCollisionObject()
        attached_object.link_name = link_name
        attached_object.object.id = object_id
        attached_object.object.operation = CollisionObject.REMOVE

        planning_scene = PlanningMsg()
        planning_scene.is_diff = True
        planning_scene.robot_state.is_diff = True
        planning_scene.robot_state.attached_collision_objects.append(
            attached_object)

        result = await self._apply_planning_scene(planning_scene)
        return result

    def detach_object(self, object_id, link_name):
        self.node.get_logger().info("detach_object()")
        executor = rclpy.get_global_executor()

        if executor is None:
            raise RuntimeError(
                "No executor is running. Make sure rclpy.init() has been called")

        # Create a new future for this request
        self.future = Future()

        executor.create_task(
            self.detach_object_async(
                object_id,
                link_name
            )
        ).add_done_callback(self.done_callback)

        self.node.get_logger().info('Task done')

        return self.future

    async def load_scene_from_parameters_async(self, parameters):
        """
        Loads a planning scene from parameters synchronously.
        """
        self.node.get_logger().info("load_scene_from_paramters_async()")
        for param in parameters:
            self.node.get_logger().info("Adding param")
            await self.add_box_async(
                box_id=param['id'],
                size=param['size'],
                position=param['position'],
                orientation=param.get('orientation', (0.0, 0.0, 0.0, 1.0)),
                frame_id=param.get('frame_id', 'base')
            )

    def load_scene_from_parameters(self, parameters):
        self.node.get_logger().info("load_scene_from_paramters()")
        executor = rclpy.get_global_executor()

        if executor is None:
            raise RuntimeError(
                "No executor is running. Make sure rclpy.init() has been called")

        # Create a new future for this request
        self.future = Future()

        executor.create_task(
            self.load_scene_from_parameters_async(
                parameters
            )
        ).add_done_callback(self.done_callback)

        self.node.get_logger().info('Task done')

        return self.future

    async def _apply_collision_object(self, collision_object):
        """
        Helper function to apply collision object changes synchronously.
        """
        self.node.get_logger().info("apply_collision_object()")
        planning_scene = PlanningMsg()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)
        result = await self._apply_planning_scene(planning_scene)
        return result

    async def _apply_planning_scene(self, planning_scene):
        """
        Helper function to apply changes to the planning scene synchronously.
        """
        self.node.get_logger().info("apply_planning_scene()")
        request = ApplyPlanningScene.Request()
        request.scene = planning_scene

        self.node.get_logger().info("Async call to planning scene client.")
        result = await self.apply_planning_scene_client.call_async(request)
        return result

    def get_current_joint_state(self):
        """
        Retrieves the current joint state of the robot synchronously.
        """
        future = Future()

        def joint_state_callback(msg):
            if msg.name and msg.position:
                if not future.done():
                    future.set_result(msg)
            else:
                self.node.get_logger().warn('Received empty JointState message.')

        if self.joint_state_subscriber is None:
            # Subscribe to the joint_states topic if not already subscribed
            self.joint_state_subscriber = self.node.create_subscription(
                JointState,
                '/joint_state_broadcaster/joint_states',  # Use the correct topic
                joint_state_callback,
                10
            )

        joint_state = future.result()
        if not joint_state.name or not joint_state.position:
            self.node.get_logger().error('JointState message is empty!')
            raise RuntimeError('JointState message is empty!!!!!!!')

        return joint_state
