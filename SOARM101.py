import numpy as np

class SOARM101:
    """
    Forward Kinematics for SO Arm 101
    units: radians/cm
    use SOARM101() to initialize the class
    use arm.forward_kinematics(angles) to get the end effector transformation matrix
    """
    def __init__(self):
        self.angles = [0, 0, 0, 0, 0, 0]
        self.rotation_axis = [2, 0, 0, 0, 1, 0] # z, x, x, x, y, x
        self.link_offsets = [
            [0, 0, 0],
            [0, 3.115, 11.97],
            [0, 11.235, -2.8],
            [0, 13.49, 0.485],
            [0, 5.48, 0],
            [0, 3.15, 2]
        ]
        self.grab_position_offset = [0, 7.6, -1.15]

    def transformation_matrix(self, angle, axis, offset):
        """
        Generate Transformation matrix for each joint
        Args:
            angle: angle of the joint, in radians
            axis: axis of the joint, x:0, y:1, z:2
            offset: offset of the joint, in cm
        Returns:
            Transformation matrix
        """
        if axis == 0:
            return np.array([[1, 0, 0, offset[0]],
                             [0, np.cos(angle), -np.sin(angle), offset[1]],
                             [0, np.sin(angle), np.cos(angle), offset[2]],
                             [0, 0, 0, 1]])
        if axis == 1:
            return np.array([[np.cos(angle), 0, np.sin(angle), offset[0]],
                             [0, 1, 0, offset[1]],
                             [-np.sin(angle), 0, np.cos(angle), offset[2]],
                             [0, 0, 0, 1]])
        if axis == 2:
            return np.array([[np.cos(angle), -np.sin(angle), 0, offset[0]],
                             [np.sin(angle), np.cos(angle), 0, offset[1]],
                             [0, 0, 1, offset[2]],
                             [0, 0, 0, 1]])
        print(f"Invalid axis: {axis}")
        return None

    def forward_kinematics(self, angles=None):
        """
        Forward Kinematics for SO Arm 101
        Args:
            None
        Returns:
            End effector transformation matrix, in radians/cm
        """
        if angles is None:
            angles = self.angles
        result = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        for index, angle in enumerate(angles):
            result = np.dot(result, 
            self.transformation_matrix(angle, self.rotation_axis[index], self.link_offsets[index]))
        result = np.dot(result, self.transformation_matrix(0, 0, self.grab_position_offset))
        # end_effector_position = np.dot(result, np.array([[0], [0], [0], [1]]))[0:3]
        return result

if __name__ == "__main__":
    arm = SOARM101()
    print(arm.forward_kinematics(angles=[0, 80/180*np.pi, 0, 0, 0, 0]))
