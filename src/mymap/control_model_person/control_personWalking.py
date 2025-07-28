#!/usr/bin/python3

import rospy
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState

def move_person_walking():
    rospy.init_node('move_person_walking', anonymous=True)
    print("Node initialized for person_walking.")
    
    # Chờ dịch vụ để thiết lập và lấy trạng thái mô hình
    rospy.wait_for_service('/gazebo/set_model_state')
    rospy.wait_for_service('/gazebo/get_model_state')
    print("Services are ready for person_walking.")
    
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        state = ModelState()
        state.model_name = 'person_walking'
        state.pose.orientation.w = 1.0  # Góc quay ban đầu
        state.pose.orientation.z -= 3.14159/2

        rate = rospy.Rate(10)
        distance = 8.0  # Khoảng cách di chuyển mỗi lần
        moving_forward = True  # Biến kiểm soát hướng di chuyển
        current_distance = 0.0  # Theo dõi khoảng cách đã di chuyển

        # Lấy vị trí hiện tại của mô hình
        current_state = get_model_state('person_walking', "")
        if current_state.success:
            state.pose.position = current_state.pose.position

        while not rospy.is_shutdown():
            # Di chuyển mô hình
            if moving_forward:
                state.pose.position.x -= 0.02  # Di chuyển về phía trước dọc trục x âm
                current_distance += 0.02  # Cập nhật khoảng cách đã di chuyển
                if current_distance >= distance:
                    moving_forward = False  # Đổi hướng
                    state.pose.orientation.z += 3.14159  # Quay 180 độ
            else:
                state.pose.position.x += 0.02  # Di chuyển ngược lại
                current_distance -= 0.02  # Cập nhật khoảng cách đã di chuyển
                if current_distance <= 0:
                    moving_forward = True  # Đổi hướng lại
                    current_distance = 0.0
                    state.pose.orientation.z -= 3.14159  # Quay trở lại góc quay ban đầu

            try:
                set_model_state(state)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                
            rate.sleep()
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    move_person_walking()
