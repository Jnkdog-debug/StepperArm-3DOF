from emm_v5_controller import Emm_V5_Controller
import time

def demo_multi_motor_sync():
    """多机同步控制完整演示"""
    
    # 初始化控制器
    motor_ctrl = Emm_V5_Controller('/dev/ttyUSB0')
    
    if not motor_ctrl.ser:
        print("无法启动程序，请检查串口")
        return
    
    try:
        addresses = [1, 2, 3]  # 三个电机的地址
        
        print("=== 多机同步控制演示 ===")
        
        # 1. 使能所有电机
        print("\n1. 使能所有电机...")
        motor_ctrl.control_motor_enable(addresses, True)
        time.sleep(0.5)
        
        # 2. 方法1: 使用 sync_position_control 方法（推荐）
        print("\n2. 方法1: 使用 sync_position_control 进行同步控制")
        
        # 定义同步运动命令
        sync_commands = [
            {
                'addr': 1,           # 电机1地址
                'direction': 0,       # 方向 (0=CW, 1=CCW)
                'velocity': 800,      # 速度 (RPM)
                'acceleration': 5,    # 加速度档位
                'pulses': 3200,      # 脉冲数 (对应10圈，3200脉冲/圈)
                'absolute': True      # 绝对运动模式
            },
            {
                'addr': 2,           # 电机2地址  
                'direction': 0,
                'velocity': 600,
                'acceleration': 8,
                'pulses': 1600,      # 脉冲数 (对应5圈)
                'absolute': True
            },
            {
                'addr': 3,           # 电机3地址
                'direction': 1,       # 反向
                'velocity': 1000,
                'acceleration': 10,
                'pulses': 4800,      # 脉冲数 (对应15圈)
                'absolute': True
            }
        ]
        
        # 执行同步控制
        print("开始同步运动...")
        success = motor_ctrl.sync_position_control(sync_commands)
        
        if success:
            print("同步运动命令发送成功，等待运动完成...")
            # 等待运动完成 (根据最长的运动时间)
            time.sleep(5)
        else:
            print("同步运动命令发送失败")
        
        # 3. 方法2: 手动实现同步控制（更细粒度控制）
        print("\n3. 方法2: 手动实现同步控制")
        manual_sync_control(motor_ctrl, addresses)
        
        # 4. 方法3: 基于角度的同步控制
        print("\n4. 方法3: 基于角度的同步控制")
        angle_based_sync_control(motor_ctrl, addresses)
        
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 退出前失能所有电机
        print("\n程序结束，失能所有电机")
        motor_ctrl.control_motor_enable(addresses, False)
        motor_ctrl.close()

def manual_sync_control(motor_ctrl, addresses):
    """手动实现同步控制 - 更细粒度的控制"""
    
    print("手动同步控制步骤:")
    
    # 步骤1: 发送所有位置命令，设置snF=True（等待同步）
    print("步骤1: 发送位置命令 (snF=True)...")
    
    for addr in addresses:
        # 为每个电机设置不同的目标位置
        if addr == 1:
            pulses = 32000  # 10圈
            velocity = 800
        elif addr == 2:
            pulses = 16000  # 5圈  
            velocity = 600
        else:  # addr == 3
            pulses = 48000  # 15圈
            velocity = 1000
            
        success = motor_ctrl.Emm_V5_Pos_Control(
            addr=addr,
            direction=0,
            velocity=velocity,
            acceleration=5,
            pulses=pulses,
            raF=True,      # 绝对运动
            snF=True       # 关键：设置为True，电机等待同步命令
        )
        
        print(f"  电机 {addr}: 命令发送{'成功' if success else '失败'}")
        
        # 检查响应
        response, count = motor_ctrl.receive_data(expected_bytes=0, timeout=0.1)
        if count >= 4:
            print(f"    响应: {response.hex(' ')}")
    
    # 短暂延时确保所有命令处理完成
    time.sleep(0.05)
    
    # 步骤2: 发送同步启动命令（广播地址0x00）
    print("步骤2: 发送同步启动命令...")
    success = motor_ctrl.Emm_V5_Synchronous_motion(addr=0)
    
    if success:
        print("  同步启动命令发送成功")
        # 检查同步命令响应
        response, count = motor_ctrl.receive_data(expected_bytes=0, timeout=0.1)
        if count > 0:
            print(f"  同步响应: {response.hex(' ')}")
    else:
        print("  同步启动命令发送失败")
    
    print("手动同步控制完成，等待运动完成...")
    time.sleep(5)

def angle_based_sync_control(motor_ctrl, addresses):
    """基于角度的同步控制 - 更直观的使用方式"""
    
    print("角度同步控制:")
    
    # 定义目标角度
    target_angles = {
        1: 180.0,   # 电机1: 180度
        2: 90.0,    # 电机2: 90度  
        3: -45.0    # 电机3: -45度
    }
    
    # 配置参数
    PULSES_PER_REV = 3200 * (32/9)  # 3200脉冲/圈 × 32/9传动比
    PULSES_PER_DEGREE = PULSES_PER_REV / 360.0
    
    # 构造同步命令
    sync_commands = []
    for addr in addresses:
        angle = target_angles[addr]
        pulses = int(abs(angle) * PULSES_PER_DEGREE)
        direction = 0 if angle >= 0 else 1
        
        sync_commands.append({
            'addr': addr,
            'direction': direction,
            'velocity': 500,      # RPM
            'acceleration': 5,    # 加速度档位
            'pulses': pulses,
            'absolute': True
        })
        
        print(f"  电机 {addr}: 目标角度 {angle}° -> 脉冲数 {pulses}")
    
    # 执行同步控制
    success = motor_ctrl.sync_position_control(sync_commands)
    
    if success:
        print("角度同步控制执行成功")
        time.sleep(3)
        
        # 验证运动结果
        print("验证运动结果:")
        angles = motor_ctrl.read_current_angles_batch(addresses)
        for addr, angle in angles.items():
            if angle is not None:
                print(f"  电机 {addr}: 当前角度 {angle:.2f}°")
    else:
        print("角度同步控制执行失败")

def sequential_vs_sync_comparison():
    """顺序运动 vs 同步运动对比演示"""
    
    motor_ctrl = Emm_V5_Controller('/dev/ttyUSB0')
    
    if not motor_ctrl.ser:
        return
    
    try:
        addresses = [1, 2, 3]
        motor_ctrl.control_motor_enable(addresses, True)
        time.sleep(0.5)
        
        print("=== 顺序运动 vs 同步运动对比 ===")
        
        # 重置到初始位置
        for addr in addresses:
            motor_ctrl.Emm_V5_Reset_CurPos_To_Zero(addr)
        time.sleep(1)
        
        # 1. 顺序运动（一个接一个）
        print("\n1. 顺序运动演示:")
        start_time = time.time()
        
        for addr in addresses:
            motor_ctrl.move_to_angle(addr, 90.0, velocity=300, acceleration=5)
            time.sleep(2)  # 等待每个电机运动完成
        
        sequential_time = time.time() - start_time
        print(f"顺序运动总时间: {sequential_time:.2f}秒")
        
        # 返回初始位置
        for addr in addresses:
            motor_ctrl.move_to_angle(addr, 0.0, velocity=300, acceleration=5)
        time.sleep(2)
        
        # 2. 同步运动（同时运动）
        print("\n2. 同步运动演示:")
        start_time = time.time()
        
        sync_commands = []
        for addr in addresses:
            sync_commands.append({
                'addr': addr,
                'direction': 0,
                'velocity': 300,
                'acceleration': 5,
                'pulses': 25600,  # 对应90度
                'absolute': True
            })
        
        motor_ctrl.sync_position_control(sync_commands)
        time.sleep(2)  # 等待所有电机运动完成
        
        sync_time = time.time() - start_time
        print(f"同步运动总时间: {sync_time:.2f}秒")
        
        print(f"\n时间节省: {sequential_time - sync_time:.2f}秒")
        
    finally:
        motor_ctrl.control_motor_enable(addresses, False)
        motor_ctrl.close()

def advanced_sync_scenarios():
    """高级同步场景演示"""
    
    motor_ctrl = Emm_V5_Controller('/dev/ttyUSB0')
    
    if not motor_ctrl.ser:
        return
    
    try:
        addresses = [1, 2, 3]
        motor_ctrl.control_motor_enable(addresses, True)
        time.sleep(0.5)
        
        print("=== 高级同步场景演示 ===")
        
        # 场景1: 协调运动（保持相对位置）
        print("\n场景1: 协调运动")
        coordinated_movement(motor_ctrl, addresses)
        
        # 场景2: 交替同步运动
        print("\n场景2: 交替同步运动")  
        alternating_sync_movement(motor_ctrl, addresses)
        
    finally:
        motor_ctrl.control_motor_enable(addresses, False)
        motor_ctrl.close()

def coordinated_movement(motor_ctrl, addresses):
    """协调运动 - 多个电机保持特定的相对位置关系"""
    
    # 定义协调运动序列
    movement_sequence = [
        # 所有电机同时移动到不同位置
        [
            {'addr': 1, 'angle': 30.0, 'velocity': 400},
            {'addr': 2, 'angle': 60.0, 'velocity': 400}, 
            {'addr': 3, 'angle': 90.0, 'velocity': 400}
        ],
        # 保持相对关系移动
        [
            {'addr': 1, 'angle': 60.0, 'velocity': 300},
            {'addr': 2, 'angle': 90.0, 'velocity': 300},
            {'addr': 3, 'angle': 120.0, 'velocity': 300}
        ]
    ]
    
    for i, movement in enumerate(movement_sequence):
        print(f"协调运动步骤 {i+1}:")
        
        sync_commands = []
        for cmd in movement:
            pulses = int(abs(cmd['angle']) * 3200 * (32/9) / 360.0)
            direction = 0 if cmd['angle'] >= 0 else 1
            
            sync_commands.append({
                'addr': cmd['addr'],
                'direction': direction,
                'velocity': cmd['velocity'],
                'acceleration': 5,
                'pulses': pulses,
                'absolute': True
            })
            
            print(f"  电机 {cmd['addr']} -> {cmd['angle']}°")
        
        motor_ctrl.sync_position_control(sync_commands)
        time.sleep(3)

def alternating_sync_movement(motor_ctrl, addresses):
    """交替同步运动 - 分组同步运动"""
    
    # 第一组同步运动：电机1和2
    print("第一组同步: 电机1和2")
    sync_commands = [
        {'addr': 1, 'direction': 0, 'velocity': 500, 'acceleration': 5, 'pulses': 16000, 'absolute': True},
        {'addr': 2, 'direction': 0, 'velocity': 500, 'acceleration': 5, 'pulses': 16000, 'absolute': True}
    ]
    motor_ctrl.sync_position_control(sync_commands)
    time.sleep(2)
    
    # 第二组同步运动：电机2和3  
    print("第二组同步: 电机2和3")
    sync_commands = [
        {'addr': 2, 'direction': 1, 'velocity': 400, 'acceleration': 8, 'pulses': 8000, 'absolute': True},
        {'addr': 3, 'direction': 0, 'velocity': 400, 'acceleration': 8, 'pulses': 24000, 'absolute': True}
    ]
    motor_ctrl.sync_position_control(sync_commands)
    time.sleep(2)

if __name__ == "__main__":
    # 运行主演示
    demo_multi_motor_sync()
    
    # 运行对比演示
    # sequential_vs_sync_comparison()
    
    # 运行高级场景演示  
    # advanced_sync_scenarios()