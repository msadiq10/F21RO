"""cw_task1 controller."""

# Importing Robot class of the controller module.
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
        
        # Set initial velocity of the motors.
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
        
        # Boolean variable used to indicate 
        # whether to turn right or left at the junction.
        # default set to false, meaning turn left.
        self.turn_right = False
        
    # Main function to start the movement of the bot.
    def run_robot(self):    
        # Main loop    
        while self.robot.step(self.time_step) != -1:
        
            # set velocities of the motors of the bot
            # to max speed.
            left_speed = self.max_speed
            right_speed = self.max_speed
            
            # store ground sensors values
            left_ir_value = self.left_ir.getValue()
            center_ir_value = self.center_ir.getValue()
            right_ir_value = self.right_ir.getValue()
            
            # if ground sensors detect black mark,
            # prepare to turn right at the junction
            if left_ir_value < 300 and center_ir_value < 300 and right_ir_value < 300:
                self.turn_right = True
                 
            
            # Check if the bot has reached the junction
            front_wall = self.proximity_sensors[0].getValue() > 120 or self.proximity_sensors[7].getValue() > 120  
            
            # If bot has reached the junction, decide whether to turn left or right accordingly.
            if front_wall and not (left_ir_value < 800 and center_ir_value < 800 and right_ir_value < 800):
                if not self.turn_right:

                    left_speed = -self.max_speed
                    right_speed = self.max_speed
                elif self.turn_right:

                    left_speed = self.max_speed
                    right_speed = -self.max_speed
            
            # Update motor velocities.
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)
# Main 
if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run_robot()
