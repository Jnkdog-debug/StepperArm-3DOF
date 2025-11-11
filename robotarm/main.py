from emm_v5_controller import Emm_V5_Controller, SysParams_t

# 初始化
motor = Emm_V5_Controller('/dev/ttyUSB0')

# 读取角度
angle1, error = motor.read_current_angle(1)
angle2,error = motor.read_current_angle(2)
angle3,error = motor.read_current_angle(3)
print(angle1)
print(angle2)
print(angle3)
# 控制移动
motor.move_to_angle(1, 0, velocity=500, acceleration=20)
motor.move_to_angle(2, 0, velocity=500, acceleration=20)
motor.move_to_angle(3, 0, velocity=500, acceleration=20)






# 批量操作
angles = motor.read_current_angles_batch([1])

print(angles)