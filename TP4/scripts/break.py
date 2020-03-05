import rospy
from kobuki_msgs.msg  import WheelDropEvent
from kobuki_msgs.msg  import BumperEvent
from kobuki_msgs.msg  import MotorPower

class Breaker:
    def __init__(self):
        rospy.init_node("breaker")
        rospy.Subscriber("/mobile_base/events/wheel_drop", WheelDropEvent, self.block_wheels)
        rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.block_bumpers)
        self.publisher = rospy.Publisher('/mobile_base/commands/motor_power', MotorPower, queue_size=10)
        self.power = MotorPower()
        self.wheels = {0:0, 1:0}
        self.bumpers = {0:0, 1:0, 2: 0}
        rospy.spin()
        
    def check_blocks(self):
        if any(self.wheels.values()) or any(self.bumpers.values()):
            print('Turning off')
            self.power.state = 0
        else:
            print('Turning on')
            self.power.state = 1
        self.publisher.publish(self.power)            
       
    def block_wheels(self, msg):
        print('Wheels {}'.format(msg))
        self.wheels[msg.wheel] = msg.state
        self.check_blocks()
        
    def block_bumpers(self, msg):
        print('Bumpers {}'.format(msg))
        self.bumpers[msg.bumper] = msg.state
        self.check_blocks()
  
if __name__ == '__main__':
    Breaker()

