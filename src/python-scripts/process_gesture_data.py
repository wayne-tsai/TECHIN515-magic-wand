"""
Process Gesture Data - Serial to CSV Converter

This script reads accelerometer data from the ESP32 serial port and 
automatically saves it to CSV files. It detects the start and end of 
gesture captures and creates a new file for each gesture.

Usage:
    python process_gesture_data.py [options]

Options:
    --port PORT       Serial port to use (default: auto-detect)
    --baud BAUD       Baud rate (default: 115200)
    --gesture NAME    Gesture name (default: "gesture")
    --person NAME     Person name (default: "user")
    --output DIR      Output directory (default: "data")
"""

import serial
import serial.tools.list_ports
import os
import time
import argparse
import csv
from datetime import datetime
import sys

def find_arduino_port():
    """
    Auto-detect the port with the ESP32 connected.
    """
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # Look for common ESP32 USB descriptors
        if "CP210" in p.description or "CH340" in p.description or "FTDI" in p.description or "USB Serial" in p.description:
            return p.device
    return None

def list_available_ports():
    """
    List all available serial ports.
    """
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return "No serial ports found."
    
    result = "Available ports:\n"
    for i, p in enumerate(ports):
        result += f"{i+1}. {p.device} - {p.description}\n"
    return result

def ensure_directory(directory):
    """
    Create directory if it doesn't exist.
    """
    if not os.path.exists(directory):
        os.makedirs(directory)
        print(f"Created directory: {directory}")

def save_data_to_csv(filepath, data):
    """
    Save accelerometer data to CSV file with timestamps.
    """
    # Calculate timestamps (100Hz = 10ms intervals)
    timestamps = []
    for i in range(len(data)):
        # Start at 0ms and increment by 10ms for each sample
        timestamps.append(i * 10)
    
    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'x', 'y', 'z'])  # Header with timestamp
        for i, (x, y, z) in enumerate(data):
            writer.writerow([timestamps[i], x, y, z])
    
    print(f"Saved {len(data)} samples to {filepath}")
    return len(data)

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Process gesture data from ESP32")
    parser.add_argument("--port", help="Serial port to use (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--gesture", default="gesture", help="Gesture name (default: 'gesture')")
    parser.add_argument("--person", default="user", help="Person name (default: 'user')")
    parser.add_argument("--output", default="data", help="Output directory (default: 'data')")
    parser.add_argument("--list-ports", action="store_true", help="List available serial ports and exit")
    args = parser.parse_args()
    
    # Just list ports if requested
    if args.list_ports:
        print(list_available_ports())
        return

    # Auto-detect port if not specified
    port = args.port
    if not port:
        port = find_arduino_port()
        if not port:
            print("Error: Could not auto-detect ESP32 port.")
            print(list_available_ports())
            print("Please specify with --port or try --list-ports to see available options.")
            return
    
    # Ensure output directory exists
    output_dir = os.path.join(args.output, args.gesture)
    ensure_directory(output_dir)
    
    print(f"Connecting to ESP32 on {port} at {args.baud} baud...")
    
    try:
        # Connect to the serial port
        ser = serial.Serial(port, args.baud, timeout=1)
        time.sleep(2)  # Wait for connection to establish
        
        print("Connected! Waiting for gesture data...")
        print("Press Ctrl+C to exit")
        print(f"Send 'o' to start capture (will automatically stop after 1 second)")
        
        # Variables for data collection
        collecting = False
        current_data = []
        capture_count = 0
        
        # Main loop
        while True:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    
                    # Check for start marker
                    if "-,-,-" in line:
                        collecting = True
                        current_data = []
                        print("Capture started...")
                        continue
                    
                    # Check for capture complete message
                    if "Capture complete" in line:
                        if current_data:
                            # Generate filename with timestamp
                            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                            capture_count += 1
                            filename = f"output_{args.gesture}_{args.person}_{capture_count}_{timestamp}.csv"
                            filepath = os.path.join(output_dir, filename)
                            
                            # Save data to CSV with timestamps
                            save_data_to_csv(filepath, current_data)
                            current_data = []
                        else:
                            print("Warning: No data was collected during this capture.")
                        collecting = False
                        continue
                    
                    # Process data if we're collecting
                    if collecting:
                        try:
                            # Parse x,y,z values
                            if "," in line:
                                x, y, z = map(float, line.split(','))
                                current_data.append([x, y, z])
                        except ValueError:
                            # Skip lines that don't contain valid data
                            pass
                except UnicodeDecodeError:
                    # Skip binary data that can't be decoded
                    pass
            
            # Send commands to ESP32
            if os.name == 'nt':  # Windows
                import msvcrt
                if msvcrt.kbhit():
                    key = msvcrt.getch().decode('utf-8')
                    if key == 'o':
                        ser.write(b'o')
                        print("Sent start command...")
                    elif key == 'p':
                        ser.write(b'p')
                        print("Sent stop command...")
            else:  # Unix/Mac
                import sys, select
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    if key == 'o':
                        ser.write(b'o')
                        print("Sent start command...")
                    elif key == 'p':
                        ser.write(b'p')
                        print("Sent stop command...")
            
            time.sleep(0.01)  # Small delay to prevent CPU hogging
            
    except KeyboardInterrupt:
        print("\nExiting...")
    except serial.SerialException as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    # Check if pyserial is installed
    try:
        import serial
    except ImportError:
        print("Error: The 'pyserial' module is not installed.")
        print("Please install it with: pip install pyserial")
        sys.exit(1)
        
    main() 