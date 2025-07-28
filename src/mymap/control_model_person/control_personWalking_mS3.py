#!/usr/bin/python3

import rospy
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
import math
import tf.transformations as tft

def move_person_standing():
    rospy.init_node('move_person_standing', anonymous=True)
    print("Node initialized for person_standing.")
    
    # Chờ dịch vụ để thiết lập và lấy trạng thái mô hình
    rospy.wait_for_service('/gazebo/set_model_state')
    rospy.wait_for_service('/gazebo/get_model_state')
    print("Services are ready for person_standing.")
    
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        # Lấy trạng thái hiện tại của model từ Gazebo
        current_state = get_model_state('person_walking', "")
        if not current_state.success:
            rospy.logerr("Failed to get the initial state of the model.")
            return
        
        state = ModelState()
        state.model_name = 'person_walking'
        state.pose = current_state.pose  # Sử dụng vị trí và góc quay hiện tại làm trạng thái ban đầu

        # Xác định góc quay ban đầu từ quaternion
        initial_orientation = current_state.pose.orientation
        initial_yaw = tft.euler_from_quaternion([initial_orientation.x,initial_orientation.y,initial_orientation.z,initial_orientation.w])[2]  # Lấy giá trị yaw (góc quay quanh trục z)

        rate = rospy.Rate(10)
        distance = 4.0  # Khoảng cách di chuyển mỗi lần
        moving_forward = True  # Biến kiểm soát hướng di chuyển
        current_distance = 0.0  # Theo dõi khoảng cách đã di chuyển

        while not rospy.is_shutdown():
            # Di chuyển mô hình
            if moving_forward:
                state.pose.position.x += 0.02  # Di chuyển về phía trước dọc trục x
                current_distance += 0.02  # Cập nhật khoảng cách đã di chuyển
                if current_distance >= distance:  # Kiểm tra nếu đã di chuyển đủ xa
                    moving_forward = False  # Đổi hướng
                    new_yaw = initial_yaw + math.pi  # Quay 180 độ từ góc ban đầu
                    quat = tft.quaternion_from_euler(0, 0, new_yaw)  # Tạo quaternion mới
                    state.pose.orientation.x = quat[0]
                    state.pose.orientation.y = quat[1]
                    state.pose.orientation.z = quat[2]
                    state.pose.orientation.w = quat[3]
            else:
                state.pose.position.x -= 0.02  # Di chuyển ngược lại
                current_distance -= 0.02  # Cập nhật khoảng cách đã di chuyển
                if current_distance <= 0:  # Kiểm tra nếu đã quay lại vị trí ban đầu
                    moving_forward = True  # Đổi hướng lại
                    current_distance = 0.0
                    quat = tft.quaternion_from_euler(0, 0, initial_yaw)  # Quay về góc ban đầu
                    state.pose.orientation.x = quat[0]
                    state.pose.orientation.y = quat[1]
                    state.pose.orientation.z = quat[2]
                    state.pose.orientation.w = quat[3]

            try:
                set_model_state(state)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                
            rate.sleep()
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    move_person_standing()
