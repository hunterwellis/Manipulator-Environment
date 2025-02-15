#!/usr/bin/env python3

import serial
import time


class SerialComm:
    def __init__(self, port='/dev/ttyACM1', baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    def open_connection(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(2)
            print(f"Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")

    def close_connection(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")

    def send_message(self, message):
        if self.ser and self.ser.is_open:
            print(message)
            self.ser.write(message.encode('utf-8'))
        else:
            print("Serial connection not open. Cannot send message.")

    # def receive_message(self):
    #     if self.ser and self.ser.is_open:
    #         message = self.ser.readline().decode('utf-8').strip()
    #         if message:
    #             print(f"Received: {message}")
    #         return message
    #     else:
    #         print("Serial connection not open. Cannot receive message.")
    #         return None
    #

def main():
    serial = SerialComm()
    serial.open_connection()
    serial.send_message("j11000\n")
    time.sleep(1)
    time.sleep(1)
    serial.send_message("j10\n")
    time.sleep(0.01)
    serial.close_connection()


if __name__ == "__main__":
    main()

