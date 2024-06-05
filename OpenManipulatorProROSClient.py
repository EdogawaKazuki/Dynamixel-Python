import roslibpy


class OpenManipulatorProROSClient:
    def __init__(self):
        self.ros = None
        self.set_joint_angle_service = None
        self.set_actuator_state_service = None
        self.get_joint_angle_subscriber = None
        self.joint_state = None

    def update_joint_state(self, msg):
        self.joint_state = msg
        # print(self.joint_state["position"])

    def create_service(self, service_name, service_type):
        return roslibpy.Service(self.ros, service_name, service_type)

    def create_subscriber(self, topic_name, topic_type, callback):
        subscriber = roslibpy.Topic(self.ros, topic_name, topic_type)
        subscriber.subscribe(callback)
        return subscriber

    def connect(self, host, port):
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        self.set_joint_angle_service = self.create_service('/goal_joint_space_path', 'open_manipulator_msgs/SetJointPosition')
        self.set_actuator_state_service = self.create_service('/set_actuator_state', 'open_manipulator_msgs/SetActuatorState')
        self.get_joint_angle_subscriber = self.create_subscriber('/joint_states', 'sensor_msgs/JointState', self.update_joint_state)

    def disconnect(self):
        self.ros.terminate()

    def lock_servo(self, joint_id):
        self.lock_joint_group()

    def release_servo(self, joint_id):
        self.release_joint_group()

    def set_joint_angle(self, joint_id, angle, path_time=5.0):
        angle_list = self.get_joint_angle_group()
        angle_list[joint_id] = angle
        self.set_joint_angle_group([i for i in range(len(angle_list))], angle_list, path_time)

    def get_joint_angle(self, joint_id):
        return self.joint_state["position"][joint_id]

    def set_joint_angle_group(self, joint_id_list, angle_list, path_time=5.0):
        for i in range(len(joint_id_list)):
            joint_id_list[i] = "joint" + str(joint_id_list[i])
        angle_list = [angle / 180 * 3.14159265358979323846 for angle in angle_list]
        request = roslibpy.ServiceRequest({
            'planning_group': 'arm',
            'joint_position': {
                'joint_name': joint_id_list,
                'position': angle_list,
                'max_accelerations_scaling_factor': 0.5,
                'max_velocity_scaling_factor': 0.5
            },
            'path_time': path_time
        })
        total_movement = sum([abs(angle_list[i] - self.get_joint_angle_group()[i]) for i in range(len(angle_list))])
        if total_movement > 0.1:
            self.set_joint_angle_service.call(request)

    def get_joint_angle_group(self):
        angle_rad = self.joint_state["position"]
        angle_deg = [angle * 180 / 3.14159265358979323846 for angle in angle_rad]
        return angle_deg

    def release_joint_group(self):
        request = roslibpy.ServiceRequest({
            'set_actuator_state': False,
        })
        self.set_actuator_state_service.call(request)

    def lock_joint_group(self):
        request = roslibpy.ServiceRequest({
            'set_actuator_state': True,
        })
        self.set_actuator_state_service.call(request)

    def print_error(self, joint_id, dxl_comm_result, dxl_error):
        pass

