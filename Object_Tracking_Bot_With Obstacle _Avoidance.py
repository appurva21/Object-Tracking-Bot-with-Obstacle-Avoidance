from picamera.array import PiRGBArray     #As there is a resolution problem in raspberry pi, will not be able to capture frames by VideoCapture
from picamera import PiCamera
import RPi.GPIO as GPIO
import argparse
import time
import cv2
import cv2.cv as cv
import numpy as np
import pigpio

GPIO.setmode(GPIO.BOARD)
pi = pigpio.pi()

#constant values
threshold_dist = 25
rangle_turn_time = 0.35
rangle_turn_time_l = 0.38
stop_sleep_time = 0.5
forward_sleep_time_b = 0.2
forward_sleep_time_o = 0.4
servo_sleep_time = 0.5
ussensor_sleep_time = 1

#switch
button=7
GPIO.setup(button,GPIO.IN,pull_up_down=GPIO.PUD_UP)

#Ultrasonic sensors
TRIG = 26
ECHO = 18

#left motor
MOTOR_L_R = 16
MOTOR_L_B = 15

#right motor
MOTOR_R_B = 21
MOTOR_R_R = 24

#setting pins for input and output

GPIO.setup(MOTOR_L_B,GPIO.OUT)			#mototrs
GPIO.setup(MOTOR_L_R,GPIO.OUT)
GPIO.setup(MOTOR_R_B,GPIO.OUT)
GPIO.setup(MOTOR_R_R,GPIO.OUT)

pwmLR = GPIO.PWM(MOTOR_L_R,100)
pwmLB = GPIO.PWM(MOTOR_L_B,100)
pwmRR = GPIO.PWM(MOTOR_R_R,100)
pwmRB = GPIO.PWM(MOTOR_R_B,100)

pwmLR.start(0)
pwmLB.start(0)
pwmRR.start(0)
pwmRB.start(0)

GPIO.setup(TRIG,GPIO.OUT)				#ultrasonic_sensors
GPIO.setup(ECHO,GPIO.IN)

#Functions to control robot's movement

def forward():					#forward
	#GPIO.output(MOTOR_R_B,GPIO.HIGH)
	#GPIO.output(MOTOR_R_R,GPIO.LOW)
	#GPIO.output(MOTOR_L_B,GPIO.LOW)
	#GPIO.output(MOTOR_L_R,GPIO.HIGH)
	pwmRB.ChangeDutyCycle(53)
	pwmRR.ChangeDutyCycle(0)
	pwmLB.ChangeDutyCycle(0)
	pwmLR.ChangeDutyCycle(49)
	

def right():					#rightturn
	#GPIO.output(MOTOR_R_B,GPIO.LOW)
	#GPIO.output(MOTOR_R_R,GPIO.HIGH)
	#GPIO.output(MOTOR_L_B,GPIO.LOW)
	#GPIO.output(MOTOR_L_R,GPIO.HIGH)
	pwmRB.ChangeDutyCycle(0)
	pwmRR.ChangeDutyCycle(95)
	pwmLB.ChangeDutyCycle(0)
	pwmLR.ChangeDutyCycle(95)
	
def left():					#leftturn
	#GPIO.output(MOTOR_R_B,GPIO.HIGH)
	#GPIO.output(MOTOR_R_R,GPIO.LOW)
	#GPIO.output(MOTOR_L_B,GPIO.HIGH)
	#GPIO.output(MOTOR_L_R,GPIO.LOW)
	pwmRB.ChangeDutyCycle(90)
	pwmRR.ChangeDutyCycle(0)
	pwmLB.ChangeDutyCycle(90)
	pwmLR.ChangeDutyCycle(0)

def stop():					#stop
	#GPIO.output(MOTOR_R_B,GPIO.LOW)
	#GPIO.output(MOTOR_R_R,GPIO.LOW)
	#GPIO.output(MOTOR_L_B,GPIO.LOW)
	#GPIO.output(MOTOR_L_R,GPIO.LOW)

	pwmRB.ChangeDutyCycle(0)
	pwmRR.ChangeDutyCycle(0)
	pwmLB.ChangeDutyCycle(0)
	pwmLR.ChangeDutyCycle(0)
	
def stop1():					#stop
	#GPIO.output(MOTOR_R_B,GPIO.LOW)
	#GPIO.output(MOTOR_R_R,GPIO.LOW)
	#GPIO.output(MOTOR_L_B,GPIO.LOW)
	#GPIO.output(MOTOR_L_R,GPIO.LOW)


	pwmRB.ChangeDutyCycle(20)
	pwmRR.ChangeDutyCycle(0)
	pwmLB.ChangeDutyCycle(0)
	pwmLR.ChangeDutyCycle(20)

	time.sleep(0.1)

	pwmRB.ChangeDutyCycle(0)
	pwmRR.ChangeDutyCycle(0)
	pwmLB.ChangeDutyCycle(0)
	pwmLR.ChangeDutyCycle(0)
	
	
def sonar(TRIG,ECHO):					#distance calculator
	GPIO.output(TRIG, False)
	time.sleep(0.25)
	GPIO.output(TRIG, True)
	time.sleep(0.00001)
	GPIO.output(TRIG, False)
	pulse_start = 0
	pulse_end = 0
	while GPIO.input(ECHO)==0:
  		pulse_start = time.time()

		
	while GPIO.input(ECHO)==1:
  		pulse_end = time.time()

	pulse_duration = pulse_end - pulse_start

	distance = pulse_duration * 17150

	distance = round(distance, 2)
	return distance

	
#IMAGE PROCESSING

def detect_circle(c):
	# initialize the shape name and approximate the contour
	shape = "unidentified"
	peri = cv2.arcLength(c, True)
	approx = cv2.approxPolyDP(c, 0.02 * peri, True)
	if len(approx) !=3 and len(approx) != 4 and len(approx) != 5:
		shape = "circle"
	# return the name of the shape
	return shape
	
def find_contours(frame):		#returns the largest contour of round shape 
	largest_contour=0
	cont_index=0
	cX=1
	cY=1
	contours, hierarchy = cv2.findContours(frame, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
	for index, contour in enumerate(contours):
		area=cv2.contourArea(contour)
		shape = detect_circle(contour)
		if (shape=="circle"):
			if (area >largest_contour) :
				M = cv2.moments(contour)
				cX = int((M["m10"] / M["m00"]))
				cY = int((M["m01"] / M["m00"]))
				largest_contour=area       
				cont_index=index
                              
	r=(0,0,2,2)
	centre = (cX,cY)
	if len(contours) > 0:
		r = cv2.boundingRect(contours[cont_index])
		
	return centre,r,largest_contour
		
def mask_colourRed(frame):    #returns only the red colors in the frame
	lower_red = np.array([120,79,69])	
	upper_red = np.array([164,255,255])
	hsv_frame =  cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv_frame, lower_red, upper_red)
	
	kernel_dilate = np.ones((8,8),np.uint8)
	kernel_erode  = np.ones((3,3),np.uint8)
	mask = cv2.erode(mask,kernel_erode)			#Eroding
	mask = cv2.dilate(mask,kernel_dilate)		#Dilating
	return mask	
	
def obastacleAvoidance():
#check obstacle in right
	distanceFront = sonar(TRIG,ECHO)
	print ("front Distance:",distanceFront,"cm")
	time.sleep(0.05)
	
	if distanceFront<threshold_dist : #obstacle
		stop1()
		print "stop"
		time.sleep(stop_sleep_time)
		
		#rotate servo to right
		print "Servo-> RIGHT"
		pi.set_servo_pulsewidth(11, 500)
		time.sleep(servo_sleep_time)
		
		
		#check obstacle in right
		distanceRight = sonar(TRIG,ECHO)
		print ("RIGHT Distance:",distanceRight,"cm")
		time.sleep(0.1)
		
		#when no obstacle in right
		if distanceRight > threshold_dist + 5 :
			print "No Obstacle in right"
			
			#turn right
			print "Right Turn"
			right()
			time.sleep(rangle_turn_time)
			
			print "stop"
			stop()
			time.sleep(stop_sleep_time)
			
			#rotate servo left
			print "Servo-> LEFT"
			pi.set_servo_pulsewidth(11, 2500)
			time.sleep(servo_sleep_time)
			
			#till there is obstacle in left
			temp=0
			while (True):
				tempdist = sonar(TRIG,ECHO)
				print "*****:",tempdist
				time.sleep(0.1)
				
				if tempdist > threshold_dist + 30 :
					stop1()
					time.sleep(0.2)
					break
				
				
				print"Forward to cover obstacle"
				forward()
				temp+=0.3
				time.sleep(0.3)
			print "Temp: ", temp
			
			
			print "Forward after obstacle finishes"	
			forward()
			time.sleep(0.4)
			print "stop"
			stop1()
			time.sleep(stop_sleep_time)
			
			print "Servo-> FRONT"
			pi.set_servo_pulsewidth(11, 1500)	
			time.sleep(servo_sleep_time)
			
			print"Left"
			left()
			time.sleep(rangle_turn_time_l)
			print "stop"
			stop()
			time.sleep(stop_sleep_time)
			
			print "Forward to cover width"	
			forward()
			time.sleep(1.7)
			print "stop"
			stop1()
			time.sleep(stop_sleep_time)
			
			print"Left towards obsctacle"
			left()
			time.sleep(rangle_turn_time_l)
			print "stop"
			stop()
			time.sleep(stop_sleep_time)
			
			print "Forward towards trajectory"	
			forward()
			time.sleep(temp+0.4+0.3)
			print "stop"
			stop1()
			time.sleep(stop_sleep_time)
			
			print "Right to align to trajectory"
			right()
			time.sleep(rangle_turn_time)
			print "stop"
			stop()
			time.sleep(stop_sleep_time)
			#print "Forward after left"
			#forward()
		
		#when obstacle in right
		else:
			print "Obstacle in right"
			
			#rotate servo left
			print "Servo ->LEFT"
			pi.set_servo_pulsewidth(11, 2500)
			time.sleep(servo_sleep_time)
			
			#check obstacle in left
			distanceLeft = sonar(TRIG,ECHO)
			print ("LEFT Distance:",distanceLeft,"cm")
			time.sleep(0.1)
			
			#if no obstacle in left
			if distanceLeft > threshold_dist + 5 :
			  
				#turn left
				print "left"
				left()
				time.sleep(rangle_turn_time_l)
				
				print "stop"
				stop()
				time.sleep(stop_sleep_time)
				
				#rotate servo right
				print "Servo->RIGHT"
				pi.set_servo_pulsewidth(11, 500)
				time.sleep(servo_sleep_time)

				temp=0
				#till there is obstacle in right
				while (True):
					tempdist = sonar(TRIG,ECHO)
					if tempdist > threshold_dist + 30 :
						stop1()
						time.sleep(0.2)
						break
					print"Forward to cover obstacle"
					forward()
					temp+=0.3
					time.sleep(0.3)
				print "Temp: ", temp
				
				print "Forward after obstacle finishes"	
				forward()
				time.sleep(0.4)
				print "stop"
				stop1()
				time.sleep(stop_sleep_time)
				
				print "Servo-> FRONT"
				pi.set_servo_pulsewidth(11, 1500)
				time.sleep(servo_sleep_time)
				
				print "Right"
				right()
				time.sleep(rangle_turn_time)
				print "stop"
				stop()
				time.sleep(stop_sleep_time)
				
				print "Forward after obstacle finishes"	
				forward()
				time.sleep(1.7)
				print "stop"
				stop1()
				time.sleep(stop_sleep_time)
				
				print "Right"
				right()
				time.sleep(rangle_turn_time)
				print "stop"
				stop()
				time.sleep(stop_sleep_time)
				
				print "Forward after obstacle finishes"	
				forward()
				time.sleep(temp+0.4+0.3)
				print "stop"
				stop1()
				time.sleep(stop_sleep_time)
				
				print "left"
				left()
				time.sleep(rangle_turn_time_l)
				print "stop"
				stop()
				time.sleep(stop_sleep_time)
				#print "Forward after right"
				#forward()
				
			else:
				#when no place to go
				print "STOP:no place to go"
				stop1()
				time.sleep(1)
		
	else:
		#when no obstacle in front-> move forward
		print "FORWARD:no obstacle in front"
		forward()
		time.sleep(forward_sleep_time_b)
#CAMERA CAPTURE
#initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (160, 128)
camera.framerate = 16
rawCapture = PiRGBArray(camera, size=(160, 128))
f = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)
time.sleep(0.1)
f = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)
time.sleep(0.1)
f = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)
time.sleep(0.1)
# allow the camera to warmup
	
flag=0

print ('Starting....')
stop()

time.sleep(1)

try:
	
	for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		
		# to check status of the switch
		if GPIO.input(button)!=1:
			rawCapture.truncate(0)
			print "Switch is Off"
			time.sleep(1)
			continue
			
		print "Switch is On"
		
		time.sleep(0.01)
		
		#grab the raw NumPy array representing the image, then initialize the timestamp and occupied/unoccupied text
		frame = image.array
		frame = cv2.flip(frame,1)
		hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		mask_red=mask_colourRed(frame) 
		centre,loct,area = find_contours(mask_red)
		x,y,w,h=loct
		cX,cY=centre
		
		distanceFront = sonar(TRIG,ECHO)
		print ("front Distance:",distanceFront,"cm")
		time.sleep(0.05)
		
		#check if contour if of substantial size or not
		if (w*h) < 50:
			found=0
		else:
			found=1
			cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0),2)
			cv2.circle(frame,(int(cX),int(cY)),3,(0,110,255),-1)
			print cX,cY,area
			
		intialArea= 200 #approx 100cm to 150 cm
		intialArea2=600 #approx 20cm
		
	
		if found==0 : #ball not found -> move in direction where ball was last seen
			print "ball not found"
			if flag==0:
				right()
				print"right"
				time.sleep(0.05)
			else:
				left()
				print"left"
				time.sleep(0.05)
			stop()
			print"stop"
			time.sleep(stop_sleep_time)
		 
		elif found==1: #ball found-> check if ball is still far(>110cm) or near(>20cm & 110cm) or it is in <20cm distance
			print "ball found"
			if area<intialArea :
				print "Ball 150 cm away"
				stop()
				obastacleAvoidance()
				
			elif(area>=intialArea):
				if(area<intialArea2):
					print "ball is b/w 20cm and 150cm"
					#bring coordinates of ball to center of camera's imaginary axis
					if(distanceFront>threshold_dist):
						print"No obastacle in front"
						#bring coordinates of ball to center of camera's imaginary axis
						if cX<=35 or cX>=125 :
							if cX<=35:
								flag=0
								print "right"
								right()
								time.sleep(0.05)
							elif cX>=125 :
								flag=1
								print "left"
								left()
								time.sleep(0.05)
						print"forward"
						forward()
						time.sleep(forward_sleep_time_b)
						
						print"stop"
						stop
						time.sleep(stop_sleep_time)
					else:
						#object in sight and near->but obstacle
						obastacleAvoidance()

				else:
					#object to close -> task accomplished
					print "Stop:object to close -> task accomplished"
					stop1()
					time.sleep(stop_sleep_time)
				
		# Display the resulting frame
		cv2.imshow('mask_red_circle',frame)
		
		rawCapture.truncate(0)		# clear the stream in preparation for the next frame
		
		if cv2.waitKey(1) & 0xFF == ord('q'):
			#cleanup GPIO pins
			GPIO.cleanup()
			break

		
			
except:	
	
	print "close"
	# switch servo off
	pi.set_servo_pulsewidth(11, 1500)
	time.sleep(1)
	pi.set_servo_pulsewidth(11, 0);
	pi.stop()
	
	#cleanup GPIO pins
	GPIO.cleanup()