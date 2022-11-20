from controller import Robot

# Get reference to the robot.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the e-puck motors and distance sensors.
maxMotorVelocity = 4.2
distanceSensorCalibrationConstant = 360

# Get left and right wheel motors.
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

# Get frontal distance sensors.
prox0 = robot.getDevice("ps0")
prox1 = robot.getDevice("ps1")
prox2 = robot.getDevice("ps2")
prox3 = robot.getDevice("ps3")
prox4 = robot.getDevice("ps4")
prox5 = robot.getDevice("ps5")
prox6 = robot.getDevice("ps6")
prox7 = robot.getDevice("ps7")

# Enable distance sensors.
prox0.enable(timeStep)
prox1.enable(timeStep)
prox2.enable(timeStep)
prox3.enable(timeStep)
prox4.enable(timeStep)
prox5.enable(timeStep)
prox6.enable(timeStep)
prox7.enable(timeStep)
# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity .
initialVelocity = 0.7 * maxMotorVelocity

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

while robot.step(timeStep) != -1:
    print("Prox0: "+str(prox0.getValue()))
    print("Prox7: "+str(prox7.getValue()))
    left_obstacle = prox0.getValue() > 200 or prox7.getValue() > 200
    #Sprint(left_obstacle)
    right_obstacle = prox4.getValue() > 75 or prox5.getValue() > 75 or prox6.getValue() > 75 or prox7.getValue() > 75
    if (left_obstacle):
        #print("Left Obstacle Encountered")
        #print(left_obstacle)
        leftMotor.setVelocity(initialVelocity-(0.5*initialVelocity))
        rightMotor.setVelocity(initialVelocity+(0.5*initialVelocity))
    elif (right_obstacle):
        #print("right Obstacle Encountered")
        leftMotor.setVelocity(initialVelocity-(0.5*initialVelocity))
        rightMotor.setVelocity(initialVelocity-(0.5*initialVelocity))