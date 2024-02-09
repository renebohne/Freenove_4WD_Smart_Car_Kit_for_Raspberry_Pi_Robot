# Copyright 2024 René Gern
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

import smbus

class ADCPublisher(Node):

    def __init__(self):
        super().__init__('freenove_adc')
        
        # Get I2C bus
        self.bus = smbus.SMBus(1)
        
        self.ADDRESS = 0x48
        self.ADS7830_CMD = 0x84 # Single-Ended Inputs
        
        self.publisher_ = self.create_publisher(Float32MultiArray, 'adc/array', 10)
        self.publisher1_ = self.create_publisher(Float32, 'adc/leftIDR', 10)
        self.publisher2_ = self.create_publisher(Float32, 'adc/rightIDR', 10)
        self.publisher3_ = self.create_publisher(Float32, 'adc/battery', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def recvADC(self,channel):
        COMMAND_SET = self.ADS7830_CMD | ((((channel<<2)|(channel>>1))&0x07)<<4)
        self.bus.write_byte(self.ADDRESS,COMMAND_SET)
        while(1):
            value1 = self.bus.read_byte(self.ADDRESS)
            value2 = self.bus.read_byte(self.ADDRESS)
            if value1==value2:
                break;
        voltage = value1 / 255.0 * 3.3  #calculate the voltage value
        voltage = round(voltage,2)
        return voltage
        
    def i2cClose(self):
        self.bus.close()

    def timer_callback(self):
        msg = Float32MultiArray()
        array =[0.0,0.0,0.0] 

        #READ ADC
        array[0]=self.recvADC(0)
        array[1]=self.recvADC(1)
        array[2]=self.recvADC(2)*3.0

        msg.data = array
        self.publisher_.publish(msg)

        msg1 = Float32();
        msg1.data = array[0]
        self.publisher1_.publish(msg1)
        msg1.data = array[1]
        self.publisher2_.publish(msg1)
        msg1.data = array[2]
        self.publisher3_.publish(msg1)

def main(args=None):
    rclpy.init(args=args)
    adc_publisher = ADCPublisher()

    rclpy.spin(adc_publisher)
    adc_publisher.i2cClose()
    adc_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
