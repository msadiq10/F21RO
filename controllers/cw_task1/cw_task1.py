"""cw_task1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

class Controller:
    def __init__(self, robot):        
        # Robot Parameters
        self.robot = robot
        self.time_step = int(robot.getBasicTimeStep())
        self.max_speed = 6.28
        
        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
    
        # Enable Proximity Sensors
        self.proximity_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            self.proximity_sensors.append(self.robot.getDevice(sensor_name))
            self.proximity_sensors[i].enable(self.time_step)
       
        # Enable Ground Sensors
        self.left_ir = self.robot.getDevice('gs0')
        self.left_ir.enable(self.time_step)
        self.center_ir = self.robot.getDevice('gs1')
        self.center_ir.enable(self.time_step)
        self.right_ir = self.robot.getDevice('gs2')
        self.right_ir.enable(self.time_step)
        

    def run_robot(self):        

        while self.robot.step(self.time_step) != -1:
            
            for i in range(8):
                print("Distance Sensors - Index: {}  Value: {}".format(i,self.proximity_sensors[i].getValue()))
                
            right_wall = self.proximity_sensors[2].getValue() > 80
            front_wall = self.proximity_sensors[0].getValue() > 80 or self.proximity_sensors[7].getValue() > 80
            
            left_speed = self.max_speed
            right_speed = self.max_speed
            
            if front_wall:
                print("Turning left")
                left_speed = -self.max_speed
                right_speed = self.max_speed
            
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)
if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run_robot()
