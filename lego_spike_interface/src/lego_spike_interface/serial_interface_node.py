#!/usr/bin/env python3
"""
Starts the serial interface
"""

import argparse
import sys
import rclpy
from lego_spike_interface.interfaces import SerialInterface

class SerialInterfaceNode(Node):
    def __init__(self):
        super().__init__('lego_spike_serial_interface')
    
    def run(self):
        parser = argparse.ArgumentParser('Serial interface for Lego Mindstorms and Lego Spike Prime hub')
        parser.add_argument("-p", "--port", metavar="TTY", type=str, default="/dev/lego", dest='port', help="Serial port to open (default /dev/lego)")
        parser.add_argument("-b", "--baud", metavar="INT", type=int, default=115200, dest='baud', help="Serial port to open (default /dev/lego)")
        parser.add_argument("-v", "--verbose", action='store_true', dest='verbose', help="Show all serial I/O from the hub")
        args, unk_args = parser.parse_known_args()

        interface = SerialInterface(port=args.port, baud=args.baud, verbose=args.verbose)
        interface.open()
        interface.send_main()
        interface.run()
        interface.close()

def main(args=None):
    rclpy.init(args=args)
    node = SerialInterfaceNode()
    node.run()
    rclpy.shutdown()
    sys.exit(0)

if __name__=='__main':
    main()