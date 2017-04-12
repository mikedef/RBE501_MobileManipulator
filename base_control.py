
#!/usr/bin/env python  
import roslib
import math
#roslib.load_manifest('base_control')
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker
import tf

def pointToPoint():
    global dist
    global dTheta
    #timeout = (dist / 0.9) + (dTheta * 12 / math.pi) * 1.5
    #startTime = rospy.get_time()
    #endTime = startTime + timeout

    print 'Turning to target'
    print 'Angle to target: ' + str(dTheta)
    
    turnRad(dTheta)
    
    print 'Target angle reached'
    print 'Angular error: ' + str(dTheta)
    rospy.sleep(1.0)

    print 'Driving Forward'
    print 'Distance to target: ' + str(dist)
    
    driveMeters(dist)
    
    print 'Target angle reached'
    print 'Distance error: ' + str(dist)
    
    #print '#pointToPoint was sucessful in getting near target'
    return True #pointToPoint was sucessful in getting near target

        
#Turns the base of the robot by (turnAngle) radians at (turnSpeed) rad/s
def turnRad(turnAngle, turnSpeed = math.pi/12):
    global wR
    initAngle = wR  #starting angle of the robot
    turnSpeed = math.fabs(turnSpeed) #negative speed makes things messy
    passZero = False
    if ((wR + turnAngle) >= 360) or ((wR + turnAngle) < 0):
        passZero = True #True if the robot's rotation will bring it past zero
    
    #limit turn angle to (-180,180)
    if turnAngle >= math.pi or turnAngle <= math.pi:
        turnAngle = modulusOp(turnAngle,math.pi) * (math.pi)
    turnMult = 1.0 #Denotes positive or negative turn
    if turnAngle < 0:
        turnMult = - 1.0
    
    #targetAngle is the desired angle of the robot in world coordiantes after turning by (turnAngle)
    targetAngle = wR + turnAngle
        
    #limit target angle to (-360,360)
    if targetAngle >= (2*math.pi) or targetAngle <= (2*math.pi):
        targetAngle = modulusOp(targetAngle,(2*math.pi)) * (2*math.pi)
        
    #limit targetAngle to [0,360)
    if targetAngle < 0:
        targetAngle = 360 + targetAngle
    
    cmd.linear.x = 0
    cmd.angular.z = turnSpeed * turnMult
    pub.publish(cmd)
    
    if not passZero:
        if turnMult > 0:
            while wR < targetAngle:
                pub.publish(cmd)
        else:
            while wR > targetAngle:
                pub.publish(cmd)
    else:
        if turnMult > 0:
            while wR > initAngle:
                pub.publish(cmd)
            while wR < targetAngle:
                pub.publish(cmd)
        else:
            while wR < initAngle:
                pub.publish(cmd)
            while wR > targetAngle:
                pub.publish(cmd)
            
    cmd.linear.x = 0
    cmd.angular.z = 0
    pub.publish(cmd)

    
#Drives the base of the robot forward by (driveDist) meters at (driveSpeed) m/s 
def driveMeters(driveDist, driveSpeed = 0.25):
    global wX
    global wY

    #Starting x and y position of the robot
    initx = wX
    initY = wY
    
    distDriven = math.sqrt((float(initX - wX)**2) + (float(initY - wY)**2)) #distance driven forward
    
    cmd.linear.x = driveSpeed
    cmd.angular.z = 0
    
    while distDriven < driveDist:
        pub.publish(cmd)
        distDriven = math.sqrt((float(initX - wX)**2) + (float(initY - wY)**2))
        
    cmd.linear.x = 0
    cmd.angular.z = 0
    pub.publish(cmd)

    
def modulusOp(a, b): #because python's mod operation is confusing when negative
    b = math.fabs(b)
    c = 1
    if a < 0:
        c = -1
    rem = math.fabs(a) - b
    while rem >= 1.0:
        rem = rem - b   
    return (b * c)

    
def odometryCb(msg):
    global wX
    global wY
    global wR
    global dist
    global theta
    global dTheta
    global odomStarted
    
    odomStarted = True
    wX = float(msg.pose.pose.position.x)
    wY = float(msg.pose.pose.position.y)
    wR = float(msg.pose.pose.orientation.z)

    dist = math.sqrt((float(tX - wX)**2) + (float(tY - wY)**2)) #distance to target
    theta = math.atan(float(tY - wY)/float(tX - wX)) #angle from robot to target wrt world
    dTheta = theta - wR
    
    if ((rospy.get_time() % 10) > 0.95) and printOut:
        print 'Dist:    ' + str(dist)
        #print 'Theta:   ' + str((theta*180/math.pi))
        #print 'wR:      ' + str((wR*180/math.pi))
        print 'DTheta:  ' + str(dTheta) #(dTheta*180/math.pi))

        
if __name__ == '__main__':

    wX = 0 #x position of robot wrt world
    wY = 0 #y position of robot wrt world
    wR = 0 #angular position of robot wrt world
    tX = 0 #x position of target wrt world
    tY = 0 #y position of target wrt world
    tR = 0 #angular position of robot wrt world
    rVel = 0 #forward velocity of robot wrt robot
    rOmega = 0 #angular velocity of robot wrt robot
    dist = 0
    theta = 0
    dTheta = 0
    printOut = False
    odomStarted = False
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10) 
    rospy.init_node('base_control')

    # rospy.init_node('oodometry', anonymous=True) #make node 
    rospy.Subscriber('odom',Odometry,odometryCb)

    
    tX = 15
    tY = 15
    
    rospy.sleep(1.0)

    cmd = Twist()
    cmd.linear.x = 0
    cmd.angular.z = 0
    
    while (not odomStarted):
        rospy.sleep(0.1)

    pointToPoint()
    rospy.spin()

