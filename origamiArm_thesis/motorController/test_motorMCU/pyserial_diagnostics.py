#!/usr/bin/env python3
"""
PySerial Diagnostic Tool
Run this to diagnose common PySerial issues
"""

import serial
import serial.tools.list_ports
import sys
import platform
import time

def check_pyserial():
    """Check PySerial installation"""
    print("🔍 PySerial Installation Check")
    print("=" * 40)
    
    try:
        import serial
        print(f"✅ PySerial version: {serial.__version__}")
    except ImportError:
        print("❌ PySerial not installed!")
        print("   Fix: pip install pyserial")
        return False
    except AttributeError:
        print("⚠️  PySerial installed but version unknown")
    
    return True

def check_ports():
    """Check available serial ports"""
    print("\n🔌 Available Serial Ports")
    print("=" * 40)
    
    ports = list(serial.tools.list_ports.comports())
    
    if not ports:
        print("❌ No serial ports found!")
        print("\n💡 Troubleshooting:")
        print("   - Check device connection")
        print("   - Install USB drivers")
        print("   - Try different USB cable")
        return []
    
    print(f"✅ Found {len(ports)} port(s):")
    for i, port in enumerate(ports):
        print(f"\n  {i+1}. {port.device}")
        print(f"     Description: {port.description}")
        print(f"     Hardware ID: {port.hwid}")
        if hasattr(port, 'vid') and port.vid:
            print(f"     VID:PID = {port.vid:04X}:{port.pid:04X}")
    
    return ports

def test_port(port_name):
    """Test opening and closing a specific port"""
    print(f"\n🧪 Testing {port_name}")
    print("-" * 30)
    
    try:
        # Test basic open/close
        ser = serial.Serial(port_name, 9600, timeout=0.1)
        print(f"✅ Opened successfully")
        print(f"   Port: {ser.name}")
        print(f"   Baud rate: {ser.baudrate}")
        print(f"   Timeout: {ser.timeout}")
        
        # Test buffer status
        waiting = ser.in_waiting
        print(f"   Buffer: {waiting} bytes waiting")
        
        ser.close()
        print(f"✅ Closed successfully")
        
        return True
        
    except PermissionError:
        print("❌ Permission denied")
        system = platform.system()
        if system == "Linux":
            print("   Fix: sudo usermod -a -G dialout $USER")
            print("   Then logout and login")
        elif system == "Windows":
            print("   Fix: Run as administrator")
        elif system == "Darwin":  # macOS
            print("   Fix: Check Security & Privacy settings")
            
    except FileNotFoundError:
        print("❌ Port not found")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        
    return False

def main():
    """Run complete diagnostic"""
    print("🔧 PySerial Diagnostic Tool")
    print("=" * 50)
    print(f"Platform: {platform.system()} {platform.release()}")
    print(f"Python: {sys.version}")
    
    # Check installation
    if not check_pyserial():
        return
    
    # Check ports
    ports = check_ports()
    
    # Test each port
    if ports:
        print(f"\n🔬 Port Testing")
        print("=" * 40)
        success_count = 0
        
        for port in ports:
            if test_port(port.device):
                success_count += 1
        
        print(f"\n📊 Results: {success_count}/{len(ports)} ports working")
        
        if success_count == 0:
            print("\n💡 All ports failed - likely permission issue")
        elif success_count < len(ports):
            print("\n💡 Some ports failed - check drivers")
        else:
            print("\n🎉 All ports working correctly!")
    
    print(f"\n✅ Diagnostic complete")

if __name__ == "__main__":
    main()
