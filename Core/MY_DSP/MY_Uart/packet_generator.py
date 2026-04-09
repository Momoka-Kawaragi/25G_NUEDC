#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口屏频率控制数据包生成和校验工具
用于生成标准格式的频率设置数据包，并验证校验码

使用方式：
    python3 packet_generator.py <frequency_hz>
    
示例：
    python3 packet_generator.py 1000      # 生成1000Hz的数据包
    python3 packet_generator.py 50000     # 生成50000Hz的数据包
"""

import sys
import argparse


def generate_packet(frequency):
    """
    生成频率设置数据包
    
    Args:
        frequency (int): 目标频率，单位Hz
        
    Returns:
        tuple: (packet_bytes, packet_hex_string)
    """
    
    # 检查频率范围
    if frequency < 0 or frequency > 0xFFFFFFFF:
        raise ValueError(f"频率超出范围[0, {0xFFFFFFFF}]")
    
    # 帧头和命令
    frame_header = 0x55
    cmd = 0x01
    
    # 频率数据（4字节，大端序）
    freq_b3 = (frequency >> 24) & 0xFF
    freq_b2 = (frequency >> 16) & 0xFF
    freq_b1 = (frequency >> 8) & 0xFF
    freq_b0 = frequency & 0xFF
    
    # 计算校验码（命令 + 4字节频率数据的累加和）
    checksum = (cmd + freq_b3 + freq_b2 + freq_b1 + freq_b0) & 0xFF
    
    # 帧尾
    frame_tail = 0xFF
    
    # 构建数据包
    packet = bytes([frame_header, cmd, freq_b3, freq_b2, freq_b1, freq_b0, checksum, frame_tail])
    
    # 转换为十六进制字符串
    hex_string = ' '.join(f'{b:02X}' for b in packet)
    
    return packet, hex_string


def verify_packet(packet_hex_string):
    """
    验证数据包的格式和校验码
    
    Args:
        packet_hex_string (str): 十六进制数据包字符串，如 "55 01 00 00 03 E8 EC FF"
        
    Returns:
        dict: 验证结果
    """
    
    try:
        # 解析十六进制字符串
        packet = bytes.fromhex(packet_hex_string.replace(' ', ''))
    except ValueError as e:
        return {'valid': False, 'error': f'十六进制格式错误: {e}'}
    
    # 检查长度
    if len(packet) != 8:
        return {'valid': False, 'error': f'数据包长度错误: 期望8字节，实际{len(packet)}字节'}
    
    # 检查帧头和帧尾
    if packet[0] != 0x55:
        return {'valid': False, 'error': f'帧头错误: 期望0x55，实际0x{packet[0]:02X}'}
    
    if packet[7] != 0xFF:
        return {'valid': False, 'error': f'帧尾错误: 期望0xFF，实际0x{packet[7]:02X}'}
    
    # 检查命令符
    cmd = packet[1]
    if cmd != 0x01:
        return {'valid': False, 'error': f'命令符错误: 期望0x01，实际0x{cmd:02X}'}
    
    # 提取频率和校验码
    freq = (packet[2] << 24) | (packet[3] << 16) | (packet[4] << 8) | packet[5]
    checksum_received = packet[6]
    
    # 计算校验码
    checksum_calc = (packet[1] + packet[2] + packet[3] + packet[4] + packet[5]) & 0xFF
    
    # 验证校验码
    if checksum_received != checksum_calc:
        return {
            'valid': False,
            'error': f'校验码错误: 期望0x{checksum_calc:02X}，实际0x{checksum_received:02X}',
            'frequency': freq,
            'checksum_expected': checksum_calc,
            'checksum_received': checksum_received
        }
    
    return {
        'valid': True,
        'frequency': freq,
        'checksum': checksum_received,
        'packet': packet_hex_string
    }


def print_detailed_info(frequency, packet, hex_string):
    """打印数据包的详细信息"""
    print(f"\n{'='*60}")
    print(f"频率设置数据包生成器")
    print(f"{'='*60}")
    print(f"\n目标频率: {frequency:,} Hz")
    print(f"\n数据包格式:")
    print(f"  帧头   | 命令 | 频率(大端序)    | 校验 | 帧尾")
    print(f"  -------|------|-----------------|------|------")
    print(f"  0x{packet[0]:02X}  | 0x{packet[1]:02X}  | 0x{packet[2]:02X} 0x{packet[3]:02X} 0x{packet[4]:02X} 0x{packet[5]:02X} | 0x{packet[6]:02X}  | 0x{packet[7]:02X}")
    print(f"\n十六进制数据包: {hex_string}")
    print(f"\n字段说明:")
    print(f"  帧头(0x55)      : 固定值，标识数据包开始")
    print(f"  命令(0x01)      : 0x01=设置频率")
    print(f"  频率数据        : {frequency:,} = 0x{frequency:08X}")
    print(f"    - 字节0(高位): 0x{packet[2]:02X} (频率 >> 24)")
    print(f"    - 字节1      : 0x{packet[3]:02X} (频率 >> 16)")
    print(f"    - 字节2      : 0x{packet[4]:02X} (频率 >> 8)")
    print(f"    - 字节3(低位): 0x{packet[5]:02X} (频率 & 0xFF)")
    print(f"  校验码(0x{packet[6]:02X})      : 命令+4字节频率 的累加和")
    print(f"    计算: 0x{packet[1]:02X} + 0x{packet[2]:02X} + 0x{packet[3]:02X} + 0x{packet[4]:02X} + 0x{packet[5]:02X}")
    print(f"        = 0x{(packet[1] + packet[2] + packet[3] + packet[4] + packet[5]):04X}")
    print(f"        = 0x{packet[6]:02X} (仅保留低8位)")
    print(f"  帧尾(0xFF)      : 固定值，标识数据包结束")
    print(f"\n{'='*60}\n")


def main():
    parser = argparse.ArgumentParser(
        description='串口屏频率控制数据包生成工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例：
  python3 packet_generator.py 1000       # 生成1000Hz的数据包
  python3 packet_generator.py 50000      # 生成50000Hz的数据包
  python3 packet_generator.py 1000000    # 生成1MHz的数据包
  
  python3 packet_generator.py --verify "55 01 00 00 03 E8 EC FF"  # 验证数据包
        """)
    
    parser.add_argument('frequency', type=int, nargs='?', 
                        help='目标频率(单位Hz)')
    parser.add_argument('--verify', type=str,
                        help='验证数据包格式 (十六进制字符串，用空格分隔)')
    parser.add_argument('--table', action='store_true',
                        help='显示常用频率的数据包表')
    
    args = parser.parse_args()
    
    # 模式1：验证数据包
    if args.verify:
        result = verify_packet(args.verify)
        
        print(f"\n{'='*60}")
        print(f"数据包验证结果")
        print(f"{'='*60}\n")
        
        print(f"输入数据包: {args.verify}")
        
        if result['valid']:
            print(f"\n✓ 数据包有效")
            print(f"  频率: {result['frequency']:,} Hz")
            print(f"  校验码: 0x{result['checksum']:02X}")
        else:
            print(f"\n✗ 数据包无效")
            print(f"  错误: {result['error']}")
            if 'frequency' in result:
                print(f"  频率: {result['frequency']:,} Hz")
        
        print(f"\n{'='*60}\n")
        return
    
    # 模式2：显示常用频率表
    if args.table:
        frequencies = [100, 1000, 10000, 50000, 100000, 
                      1000000, 10000000, 50000000, 100000000, 1000000000]
        
        print(f"\n{'='*80}")
        print(f"常用频率数据包参考表")
        print(f"{'='*80}\n")
        print(f"{'频率':>15} | {'16进制':>10} | {'数据包 (十六进制)':^50}")
        print(f"{'-'*15}-+-{'-'*10}-+-{'-'*50}")
        
        for freq in frequencies:
            packet, hex_string = generate_packet(freq)
            if freq >= 1000000:
                freq_str = f"{freq/1000000:.1f} MHz"
            elif freq >= 1000:
                freq_str = f"{freq/1000:.1f} kHz"
            else:
                freq_str = f"{freq} Hz"
            
            print(f"{freq_str:>15} | 0x{freq:08X} | {hex_string}")
        
        print(f"\n{'='*80}\n")
        return
    
    # 模式3：生成指定频率的数据包
    if args.frequency is not None:
        try:
            packet, hex_string = generate_packet(args.frequency)
            print_detailed_info(args.frequency, packet, hex_string)
        except ValueError as e:
            print(f"错误: {e}")
            return
    else:
        # 默认显示帮助和示例
        parser.print_help()
        print("\n默认示例(生成50000Hz数据包):\n")
        packet, hex_string = generate_packet(50000)
        print_detailed_info(50000, packet, hex_string)


if __name__ == '__main__':
    main()
