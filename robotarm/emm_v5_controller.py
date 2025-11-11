"""
Emm_V5.0 步进闭环驱动 Python 控制库
基于 Emm_V5.0步进闭环驱动说明书Rev1.3.pdf 协议实现
"""

import time
import serial
from enum import Enum
from typing import Dict, List, Tuple, Optional, Any

class SysParams_t(Enum):
    """系统参数枚举 - 对应文档6.3.4节"""
    S_VER   = 0   # 固件版本和硬件版本
    S_RL    = 1   # 相电阻和相电感
    S_PID   = 2   # PID参数
    S_VBUS  = 3   # 总线电压
    S_CPHA  = 5   # 相电流
    S_ENCL  = 7   # 线性化校准后的编码器值
    S_TPOS  = 8   # 电机目标位置角度
    S_VEL   = 9   # 电机实时转速
    S_CPOS  = 10  # 电机实时位置角度
    S_PERR  = 11  # 电机位置误差角度
    S_FLAG  = 13  # 状态标志位
    S_Conf  = 14  # 驱动配置参数
    S_State = 15  # 系统状态参数
    S_ORG   = 16  # 回零状态标志位

class HomingMode(Enum):
    """回零模式枚举 - 对应文档7.1节"""
    NEAREST = 0x00  # 单圈就近回零
    DIRECTION = 0x01  # 单圈方向回零
    SENSELESS = 0x02  # 多圈无限位碰撞回零
    ENDSTOP = 0x03    # 多圈有限位开关回零

class ResponseMode(Enum):
    """响应模式枚举 - 对应文档4.2节Response菜单"""
    NONE = 0x00      # 不返回任何确认
    RECEIVE = 0x01   # 只返回确认收到命令
    REACHED = 0x02   # 只返回到位命令
    BOTH = 0x03      # 返回确认和到位命令
    OTHER = 0x04     # 位置模式返回到位，其他返回确认

class Emm_V5_Controller:
    """
    Emm_V5.0 步进闭环驱动控制器
    完整实现文档中的所有控制功能
    """
    
    # 协议配置
    PROTOCOL_ANGLE_RESOLUTION = 65536.0
    DEGREES_PER_REVOLUTION = 360.0
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        初始化控制器
        
        Args:
            port: 串口设备路径
            baudrate: 波特率，默认115200
            timeout: 串口超时时间
        """
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._initialize_serial()
    
    def _initialize_serial(self):
        """初始化串口连接"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                inter_byte_timeout=0.1
            )
            print(f"串口 {self.port} @ {self.baudrate} 打开成功")
        except serial.SerialException as e:
            print(f"打开串口失败: {e}")
            self.ser = None
    
    # ==========================================================
    # 基础通信方法
    # ==========================================================
    
    def send_command(self, cmd_bytes: bytes) -> bool:
        """
        发送命令到串口
        
        Args:
            cmd_bytes: 命令字节
            
        Returns:
            bool: 发送是否成功
        """
        if not self.ser or not self.ser.is_open:
            print("串口未打开，无法发送命令")
            return False
        
        try:
            self.ser.write(cmd_bytes)
            time.sleep(0.005)  # 命令间延时
            return True
        except Exception as e:
            print(f"发送命令失败: {e}")
            return False
    
    def receive_data(self, expected_bytes: int = 0, timeout: float = 0.5) -> Tuple[bytes, int]:
        """
        接收串口数据
        
        Args:
            expected_bytes: 期望接收的字节数，0表示接收所有可用数据
            timeout: 接收超时时间
            
        Returns:
            Tuple[bytes, int]: (接收到的数据, 数据长度)
        """
        if not self.ser or not self.ser.is_open:
            return b'', 0
        
        original_timeout = self.ser.timeout
        self.ser.timeout = timeout
        
        try:
            if expected_bytes > 0:
                rx_data = self.ser.read(expected_bytes)
            else:
                rx_data = self.ser.read_all()
            return rx_data, len(rx_data)
        except Exception as e:
            print(f"接收数据失败: {e}")
            return b'', 0
        finally:
            self.ser.timeout = original_timeout
    
    def send_and_receive(self, cmd_bytes: bytes, expected_response_len: int = 0, 
                        timeout: float = 0.5) -> Tuple[bytes, int]:
        """
        发送命令并接收响应
        
        Args:
            cmd_bytes: 命令字节
            expected_response_len: 期望响应长度
            timeout: 接收超时时间
            
        Returns:
            Tuple[bytes, int]: (响应数据, 数据长度)
        """
        if self.send_command(cmd_bytes):
            return self.receive_data(expected_response_len, timeout)
        return b'', 0
    
    # ==========================================================
    # 控制动作命令 - 文档6.3.1节
    # ==========================================================
    
    def Emm_V5_En_Control(self, addr: int, state: bool, snF: bool = False) -> bool:
        """
        电机使能控制 - 命令格式: 地址 + 0xF3 + 0xAB + 使能状态 + 多机同步标志 + 0x6B
        
        Args:
            addr: 电机地址 (1-255, 0为广播地址)
            state: True为使能，False为失能
            snF: 多机同步标志，True为启用同步
        
        Returns:
            bool: 命令执行是否成功
        """
        cmd = bytearray([
            addr, 0xF3, 0xAB,
            int(state), int(snF), 0x6B
        ])
        
        response, count = self.send_and_receive(bytes(cmd), 4)
        return count >= 4 and response[1] == 0xF3 and response[2] == 0x02
    
    def Emm_V5_Vel_Control(self, addr: int, direction: int, velocity: int, 
                          acceleration: int, snF: bool = False) -> bool:
        """
        速度模式控制 - 命令格式: 地址 + 0xF6 + 方向 + 速度 + 加速度 + 多机同步标志 + 0x6B
        
        Args:
            addr: 电机地址
            direction: 方向 (0=CW, 1=CCW)
            velocity: 速度 (RPM)
            acceleration: 加速度档位 (0-255)
            snF: 多机同步标志
            
        Returns:
            bool: 命令执行是否成功
        """
        cmd = bytearray([
            addr, 0xF6, direction,
            (velocity >> 8) & 0xFF, velocity & 0xFF,
            acceleration, int(snF), 0x6B
        ])
        
        response, count = self.send_and_receive(bytes(cmd), 4)
        return count >= 4 and response[1] == 0xF6 and response[2] == 0x02
    
    def Emm_V5_Pos_Control(self, addr: int, direction: int, velocity: int, 
                          acceleration: int, pulses: int, raF: bool, 
                          snF: bool = False) -> bool:
        """
        位置模式控制 - 命令格式: 地址 + 0xFD + 方向 + 速度 + 加速度 + 脉冲数 + 相对/绝对模式 + 多机同步标志 + 0x6B
        
        Args:
            addr: 电机地址
            direction: 方向 (0=CW, 1=CCW)
            velocity: 速度 (RPM)
            acceleration: 加速度档位 (0-255)
            pulses: 脉冲数 (32位无符号)
            raF: 相对/绝对模式 (False=相对, True=绝对)
            snF: 多机同步标志
            
        Returns:
            bool: 命令执行是否成功
        """
        # 确保脉冲数为正数
        if pulses < 0:
            pulses = abs(pulses)
            
        cmd = bytearray([
            addr, 0xFD, direction,
            (velocity >> 8) & 0xFF, velocity & 0xFF,  # 速度 (16位)
            acceleration,                              # 加速度 (8位)
            (pulses >> 24) & 0xFF,                    # 脉冲数 bit24-31
            (pulses >> 16) & 0xFF,                    # 脉冲数 bit16-23
            (pulses >> 8) & 0xFF,                     # 脉冲数 bit8-15
            pulses & 0xFF,                            # 脉冲数 bit0-7
            int(raF), int(snF), 0x6B
        ])
        
        response, count = self.send_and_receive(bytes(cmd), 4)
        return count >= 4 and response[1] == 0xFD and response[2] == 0x02
    
    def Emm_V5_Stop_Now(self, addr: int, snF: bool = False) -> bool:
        """
        立即停止 - 命令格式: 地址 + 0xFE + 0x98 + 多机同步标志 + 0x6B
        
        Args:
            addr: 电机地址
            snF: 多机同步标志
            
        Returns:
            bool: 命令执行是否成功
        """
        cmd = bytearray([addr, 0xFE, 0x98, int(snF), 0x6B])
        
        response, count = self.send_and_receive(bytes(cmd), 4)
        return count >= 4 and response[1] == 0xFE and response[2] == 0x02
    
    def Emm_V5_Synchronous_motion(self, addr: int = 0) -> bool:
        """
        多机同步运动 - 命令格式: 地址 + 0xFF + 0x66 + 0x6B
        
        Args:
            addr: 电机地址 (通常使用广播地址0)
            
        Returns:
            bool: 命令执行是否成功
        """
        cmd = bytearray([addr, 0xFF, 0x66, 0x6B])
        
        response, count = self.send_and_receive(bytes(cmd), 4)
        return count >= 4 and response[1] == 0xFF and response[2] == 0x02
    
    # ==========================================================
    # 原点回零命令 - 文档6.3.2节
    # ==========================================================
    
    def Emm_V5_Set_Homing_Zero(self, addr: int, store: bool = True) -> bool:
        """
        设置单圈回零的零点位置 - 命令格式: 地址 + 0x93 + 0x88 + 存储标志 + 0x6B
        
        Args:
            addr: 电机地址
            store: 是否存储到芯片
            
        Returns:
            bool: 命令执行是否成功
        """
        cmd = bytearray([addr, 0x93, 0x88, int(store), 0x6B])
        
        response, count = self.send_and_receive(bytes(cmd), 4)
        return count >= 4 and response[1] == 0x93 and response[2] == 0x02
    
    def Emm_V5_Trigger_Homing(self, addr: int, homing_mode: HomingMode, 
                             snF: bool = False) -> bool:
        """
        触发回零 - 命令格式: 地址 + 0x9A + 回零模式 + 多机同步标志 + 0x6B
        
        Args:
            addr: 电机地址
            homing_mode: 回零模式
            snF: 多机同步标志
            
        Returns:
            bool: 命令执行是否成功
        """
        cmd = bytearray([
            addr, 0x9A, homing_mode.value, int(snF), 0x6B
        ])
        
        response, count = self.send_and_receive(bytes(cmd), 4)
        return count >= 4 and response[1] == 0x9A and response[2] == 0x02
    
    def Emm_V5_Interrupt_Homing(self, addr: int) -> bool:
        """
        强制中断并退出回零操作 - 命令格式: 地址 + 0x9C + 0x48 + 0x6B
        
        Args:
            addr: 电机地址
            
        Returns:
            bool: 命令执行是否成功
        """
        cmd = bytearray([addr, 0x9C, 0x48, 0x6B])
        
        response, count = self.send_and_receive(bytes(cmd), 4)
        return count >= 4 and response[1] == 0x9C and response[2] == 0x02
    
    # ==========================================================
    # 触发动作命令 - 文档6.3.3节
    # ==========================================================
    
    def Emm_V5_Trigger_Encoder_Calibration(self, addr: int) -> bool:
        """
        触发编码器校准 - 命令格式: 地址 + 0x06 + 0x45 + 0x6B
        
        Args:
            addr: 电机地址
            
        Returns:
            bool: 命令执行是否成功
        """
        cmd = bytearray([addr, 0x06, 0x45, 0x6B])
        
        response, count = self.send_and_receive(bytes(cmd), 4)
        return count >= 4 and response[1] == 0x06 and response[2] == 0x02
    
    def Emm_V5_Reset_CurPos_To_Zero(self, addr: int) -> bool:
        """
        将当前位置角度清零 - 命令格式: 地址 + 0x0A + 0x6D + 0x6B
        
        Args:
            addr: 电机地址
            
        Returns:
            bool: 命令执行是否成功
        """
        cmd = bytearray([addr, 0x0A, 0x6D, 0x6B])
        
        response, count = self.send_and_receive(bytes(cmd), 4)
        return count >= 4 and response[1] == 0x0A and response[2] == 0x02
    
    def Emm_V5_Reset_Clog_Pro(self, addr: int) -> bool:
        """
        解除堵转保护 - 命令格式: 地址 + 0x0E + 0x52 + 0x6B
        
        Args:
            addr: 电机地址
            
        Returns:
            bool: 命令执行是否成功
        """
        cmd = bytearray([addr, 0x0E, 0x52, 0x6B])
        
        response, count = self.send_and_receive(bytes(cmd), 4)
        return count >= 4 and response[1] == 0x0E and response[2] == 0x02
    
    def Emm_V5_Restore_Default(self, addr: int) -> bool:
        """
        恢复出厂设置 - 命令格式: 地址 + 0x0F + 0x5F + 0x6B
        
        Args:
            addr: 电机地址
            
        Returns:
            bool: 命令执行是否成功
        """
        cmd = bytearray([addr, 0x0F, 0x5F, 0x6B])
        
        response, count = self.send_and_receive(bytes(cmd), 4)
        return count >= 4 and response[1] == 0x0F and response[2] == 0x02
    
    # ==========================================================
    # 读取参数命令 - 文档6.3.4节
    # ==========================================================
    
    def Emm_V5_Read_Sys_Params(self, addr: int, param: SysParams_t) -> Tuple[bytes, int]:
        """
        读取系统参数 - 通用读取命令
        
        Args:
            addr: 电机地址
            param: 系统参数类型
            
        Returns:
            Tuple[bytes, int]: (响应数据, 数据长度)
        """
        cmd = bytearray([addr])
        
        # 根据参数类型设置功能码
        if param == SysParams_t.S_VER:
            cmd.extend([0x1F])
        elif param == SysParams_t.S_RL:
            cmd.extend([0x20])
        elif param == SysParams_t.S_PID:
            cmd.extend([0x21])
        elif param == SysParams_t.S_VBUS:
            cmd.extend([0x24])
        elif param == SysParams_t.S_CPHA:
            cmd.extend([0x27])
        elif param == SysParams_t.S_ENCL:
            cmd.extend([0x31])
        elif param == SysParams_t.S_TPOS:
            cmd.extend([0x33])
        elif param == SysParams_t.S_VEL:
            cmd.extend([0x35])
        elif param == SysParams_t.S_CPOS:
            cmd.extend([0x36])
        elif param == SysParams_t.S_PERR:
            cmd.extend([0x37])
        elif param == SysParams_t.S_FLAG:
            cmd.extend([0x3A])
        elif param == SysParams_t.S_ORG:
            cmd.extend([0x3B])
        elif param == SysParams_t.S_Conf:
            cmd.extend([0x42, 0x6C])
        elif param == SysParams_t.S_State:
            cmd.extend([0x43, 0x7A])
        else:
            return b'', 0
            
        cmd.append(0x6B)
        return self.send_and_receive(bytes(cmd))
    
    def parse_s_cpos_protocol_angle(self, rxCmd: bytes, rxCount: int) -> Tuple[Optional[float], Optional[str]]:
        """
        解析 S_CPOS 响应并计算角度 - 基于文档6.3.4节的协议
        
        Args:
            rxCmd: 响应数据
            rxCount: 数据长度
            
        Returns:
            Tuple[角度, 错误信息]: 成功返回角度，失败返回None和错误信息
        """
        EXPECTED_LEN = 8
        
        if rxCount != EXPECTED_LEN or rxCmd[1] != 0x36 or rxCmd[-1] != 0x6B:
            return None, f"响应格式错误: 期望{EXPECTED_LEN}字节, 实际{rxCount}字节"
        
        sign_byte = rxCmd[2]
        data_bytes = rxCmd[3:7]
        
        magnitude_32bit = int.from_bytes(data_bytes, byteorder='big', signed=False)
        
        # 根据符号位确定最终值
        if sign_byte == 0x01:
            final_signed_value = -magnitude_32bit
        elif sign_byte == 0x00:
            final_signed_value = magnitude_32bit
        else:
            return None, f"符号位异常: 0x{sign_byte:02x}"
        
        # 核心角度计算: 角度 = (符号值 * 360) / 65536
        angle = (final_signed_value * self.DEGREES_PER_REVOLUTION) / self.PROTOCOL_ANGLE_RESOLUTION
        return angle, None
    
    def read_current_angle(self, addr: int) -> Tuple[Optional[float], Optional[str]]:
        """
        读取电机实时角度
        
        Args:
            addr: 电机地址
            
        Returns:
            Tuple[角度, 错误信息]: 成功返回角度，失败返回None和错误信息
        """
        response, count = self.Emm_V5_Read_Sys_Params(addr, SysParams_t.S_CPOS)
        
        if count >= 8:
            return self.parse_s_cpos_protocol_angle(response, count)
        else:
            return None, f"读取角度失败: 响应长度不足 {count} 字节"
    
    def read_motor_status(self, addr: int) -> Dict[str, Any]:
        """
        读取电机状态标志位
        
        Args:
            addr: 电机地址
            
        Returns:
            Dict: 状态信息字典
        """
        response, count = self.Emm_V5_Read_Sys_Params(addr, SysParams_t.S_FLAG)
        
        status = {
            'enabled': False,
            'in_position': False,
            'stalled': False,
            'stall_protected': False,
            'raw_data': response.hex(' ') if count > 0 else '无响应'
        }
        
        if count >= 5 and response[1] == 0x3A and response[2] == 0x01:
            flag_byte = response[3]
            status.update({
                'enabled': bool(flag_byte & 0x01),
                'in_position': bool(flag_byte & 0x02),
                'stalled': bool(flag_byte & 0x04),
                'stall_protected': bool(flag_byte & 0x08)
            })
        
        return status
    
    # ==========================================================
    # 高级功能方法
    # ==========================================================
    
    def read_current_angles_batch(self, addresses: List[int]) -> Dict[int, Optional[float]]:
        """
        批量读取多个电机的实时角度
        
        Args:
            addresses: 电机地址列表
            
        Returns:
            Dict: {地址: 角度}
        """
        # 批量发送读取命令
        for addr in addresses:
            time.sleep(0.05)
            self.Emm_V5_Read_Sys_Params(addr, SysParams_t.S_CPOS)
        
        # 等待并接收所有响应
        time.sleep(0.05)
        rx_buffer, rx_count = self.receive_data(expected_bytes=0, timeout=0.5)
        
        current_angles = {}
        i = 0
        
        while i < len(rx_buffer):
            if len(rx_buffer) - i < 4:
                break
                
            addr = rx_buffer[i]
            func_code = rx_buffer[i+1]
            
            # 解析 S_CPOS 响应 (8字节)
            if func_code == 0x36 and addr in addresses and len(rx_buffer) - i >= 8:
                packet = rx_buffer[i:i+8]
                angle, error = self.parse_s_cpos_protocol_angle(packet, 8)
                
                if error is None:
                    current_angles[addr] = angle
                else:
                    print(f"电机 {addr} 角度解析错误: {error}")
                    current_angles[addr] = None
                    
                i += 8
            else:
                i += 1
        
        # 确保所有地址都有返回值
        for addr in addresses:
            if addr not in current_angles:
                current_angles[addr] = None
                
        return current_angles

        
    
    
    def control_motor_enable(self, addresses: List[int], enable: bool):
        """
        批量使能/失能电机
        
        Args:
            addresses: 电机地址列表
            enable: True为使能，False为失能
        """
        action = "使能" if enable else "失能"
        print(f"批量{action}电机: {addresses}")
        
        for addr in addresses:
            self.Emm_V5_En_Control(addr, enable, False)
            # 清除响应缓冲区
            self.receive_data(expected_bytes=0, timeout=0.1)
        
        print(f"批量{action}完成")
    
    def sync_position_control(self, motor_commands: List[Dict]) -> bool:
        """
        多机同步位置控制 - 基于文档6.5节的协议
        
        Args:
            motor_commands: 电机命令列表，每个命令包含:
                - addr: 地址
                - direction: 方向
                - velocity: 速度
                - acceleration: 加速度
                - pulses: 脉冲数
                - absolute: 绝对/相对模式
                
        Returns:
            bool: 同步控制是否成功
        """
        print("开始多机同步控制...")
        
        # 第一阶段: 发送所有位置命令 (启用同步标志)
        for cmd in motor_commands:
            success = self.Emm_V5_Pos_Control(
                addr=cmd['addr'],
                direction=cmd['direction'],
                velocity=cmd['velocity'],
                acceleration=cmd['acceleration'],
                pulses=cmd['pulses'],
                raF=cmd['absolute'],
                snF=True  # 启用同步
            )
            
            if not success:
                print(f"电机 {cmd['addr']} 同步命令发送失败")
                return False
        
        # 短暂延时确保所有命令处理完成
        time.sleep(0.05)
        
        # 第二阶段: 发送同步启动命令 (广播地址)
        success = self.Emm_V5_Synchronous_motion(addr=0)
        
        if success:
            print("多机同步控制成功")
        else:
            print("同步启动命令失败")
            
        return success
    
    def move_to_angle(self, addr: int, angle: float, velocity: int = 500, 
                     acceleration: int = 10, absolute: bool = True) -> bool:
        """
        移动到指定角度 (高级封装)
        
        Args:
            addr: 电机地址
            angle: 目标角度 (度)
            velocity: 速度 (RPM)
            acceleration: 加速度档位
            absolute: 是否为绝对运动
            
        Returns:
            bool: 命令执行是否成功
        """
        # 角度转脉冲 (基于3200脉冲/圈，32/9传动比)
        pulses_per_rev = 3200 * (32/9)
        pulses = int(angle * pulses_per_rev / 360.0)
        direction = 0 if angle >= 0 else 1
        
        return self.Emm_V5_Pos_Control(
            addr=addr,
            direction=direction,
            velocity=velocity,
            acceleration=acceleration,
            pulses=pulses,
            raF=absolute,
            snF=False
        )
    
    def close(self):
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")

# ==========================================================
# 使用示例和测试代码
# ==========================================================

def main():
    """主函数 - 演示库的使用方法"""
    # 初始化控制器
    motor_ctrl = Emm_V5_Controller('/dev/ttyUSB0')
    
    if not motor_ctrl.ser:
        print("无法启动程序，请检查串口")
        return
    
    try:
        addresses = [1, 2, 3]
        
        print("=== Emm_V5.0 控制库演示 ===")
        
        # 1. 批量读取角度
        print("\n1. 批量读取角度:")
        angles = motor_ctrl.read_current_angles_batch(addresses)
        for addr, angle in angles.items():
            print(f"  电机 {addr}: {angle:.2f}°" if angle is not None else f"  电机 {addr}: 读取失败")
        
        
        
        # 3. 单电机角度控制
        print("\n3. 单电机角度控制:")
        success = motor_ctrl.move_to_angle(1, 45.0, velocity=300, acceleration=5)
        print(f"  电机 1 移动到 45°: {'成功' if success else '失败'}")
        
        # 4. 读取状态
        print("\n4. 读取电机状态:")
        status = motor_ctrl.read_motor_status(1)
        print(f"  电机 1 状态: {status}")
        
        # 5. 多机同步控制示例
        print("\n5. 多机同步控制示例:")
        sync_commands = [
            {
                'addr': 1,
                'direction': 0,
                'velocity': 500,
                'acceleration': 5,
                'pulses': 32000,
                'absolute': True
            },
            {
                'addr': 2,
                'direction': 0, 
                'velocity': 400,
                'acceleration': 5,
                'pulses': 24000,
                'absolute': True
            }
        ]
        # motor_ctrl.sync_position_control(sync_commands)
        
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 退出前失能所有电机
        motor_ctrl.control_motor_enable(addresses, False)
        motor_ctrl.close()

if __name__ == "__main__":
    main()