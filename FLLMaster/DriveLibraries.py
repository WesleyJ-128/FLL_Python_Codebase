# The program doesn't work without this.  Not sure why.
from sys import stderr
from time import sleep
from ev3dev2.motor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor import *

# Global Variables for color sensor calibration
reflHighVal = 100
reflLowVal = 0
reflRate = 1

class Robot:
    # Import stuff
    from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, LargeMotor
    from ev3dev2.sensor.lego import ColorSensor, GyroSensor, InfraredSensor, TouchSensor, UltrasonicSensor
    from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
    from ev3dev2.sound import Sound
    from ev3dev2.button import Button
    from ev3dev2.led import Leds, LED_COLORS, LED_DEFAULT_COLOR, LED_GROUPS, LEDS
    from ev3dev2.display import Display
    import ev3dev2.fonts as fonts
    import math

    def __init__(self, filename):
        """
        Stores all the properties of the robot, such as wheel circumference, motor ports, ect.  Also provides methods for higher-level
        interaction with the robot, such as driving in a straight line at a specific heading, following a line, or driving until hitting 
        an obstacle.  Also instantiates motor and sensor objects for direct use later, if necessary.
        Reads ``filename`` for robot properties; ``filename`` should be a .ini or .cfg file in INI format.  See the current robot.cfg 
        for format example.
        """
        # Load and read the config file
        from configparser import SafeConfigParser
        conf = SafeConfigParser()
        conf.read(filename)

        # Set the "ForFLL" flag based on it's status in the config file (used in some input validation)
        self.ForFLL = bool(conf.get('Other', 'ForFLL') == "TRUE")

        # Instantiate objects for controlling things on the brick itself (Speaker, Buttons, Lights, and the LCD)
        self.spkr = self.Sound()
        self.btn = self.Button()
        self.led = self.Leds()
        self.disp = self.Display()

        # Read the drive motor ports from the config file, and store.  "eval()" is used because the port names "OUTPUT_#",
        # where # is a capital letter, A - D, are variables used as constants, and reading as a string does not work.
        self.LeftMotor = eval(conf.get('Drivebase', 'LeftMotorPort'))
        self.RightMotor = eval(conf.get('Drivebase', 'RightMotorPort'))
        # Read and store the wheel circumference and width between the wheels
        self.WheelCircumference = float(conf.get('Drivebase', 'WheelCircumference'))
        self.WidthBetweenWheels = float(conf.get('Drivebase', 'WidthBetweenWheels'))
        # Read and store MotorInverted and GyroInverted.  Both are relative to the robot, and drive functions will not work correctly if these 
        # values are incorrect, and they are the first things to check if the drive functions do not work correctly.
        self.MotorInverted = bool(conf.get('Drivebase', 'MotorInverted') == "TRUE")
        self.GyroInverted = bool(conf.get('Drivebase', 'GyroInverted') == "TRUE")
        # Reads and stores the gear ratio value.
        self.GearRatio = float(conf.get('Drivebase', 'GearRatio'))
        # Reads and stores the PID gains for driving in a straight line.
        self.kp = float(conf.get('Drivebase', 'kp'))
        self.ki = float(conf.get('Drivebase', 'ki'))
        self.kd = float(conf.get('Drivebase', 'kd'))

        # Read the sensor ports from the config file, and store.  "eval()" is used because the port names "INPUT_#",
        # where # is a number, 1 - 4, are variables used as constants, and reading as a string does not work.
        self.ColorPort = eval(conf.get('Sensors', 'ColorPort'))
        self.GyroPort = eval(conf.get('Sensors', 'GyroPort'))
        self.InfraredPort = eval(conf.get('Sensors', 'InfraredPort'))
        self.TouchPort = eval(conf.get('Sensors', 'TouchPort'))
        self.UltrasonicPort = eval(conf.get('Sensors', 'UltrasonicPort'))
        # Reads and stores the PID gains for following a line.
        self.kpLine = float(conf.get('Sensors', 'kpLine'))
        self.kiLine = float(conf.get('Sensors', 'kiLine'))
        self.kdLine = float(conf.get('Sensors', 'kdLine'))

        # Read the auxillary motor ports, if any, from the config file, and store.  "eval()" is used because the port names "OUTPUT_#",
        # where # is a capital letter, A - D, are variables used as constants, and reading as a string does not work.
        self.AuxMotor1 = eval(conf.get('AuxMotors', 'AuxMotor1'))
        self.AuxMotor2 = eval(conf.get('AuxMotors', 'AuxMotor2'))

        # Instantiate the auxillary motor objects, checking if they are connected first.
        if self.AuxMotor1 is not None:    
            try:
                self.m1 = MediumMotor(self.AuxMotor1)
            except:
                print("Aux Motor 1 Not Connected!")
                self.spkr.beep('-f 220')
        if self.AuxMotor2 is not None:    
            try:
                self.m2 = MediumMotor(self.AuxMotor2)
            except:
                print("Aux Motor 2 Not Connected!")
                self.spkr.beep('-f 220')
                sleep(0.3)
                self.spkr.beep('-f 220')

        # Instantiate the sensor objects
        self.cs = ColorSensor(self.ColorPort)
        self.gs = GyroSensor(self.GyroPort)
        # Only instantiate auxillary sensors if the config file shows they exist
        if self.InfraredPort is not None:
            try:
                self.ir = InfraredSensor(self.InfraredPort)
            except:
                pass
        if self.UltrasonicPort is not None:
            try:
                self.us = UltrasonicSensor(self.UltrasonicPort)
            except:
                pass
        if self.TouchPort is not None:
            try:
                self.ts = TouchSensor(self.TouchPort)
            except:
                pass

        # Instantiate the drive motor objects, as well as the motorset objects for controlling both simultainiously
        self.tank = MoveTank(self.LeftMotor, self.RightMotor)
        self.steer = MoveSteering(self.LeftMotor, self.RightMotor)
        self.lm = LargeMotor(self.LeftMotor)
        self.rm = LargeMotor(self.RightMotor)

        # Reset the gyro angle to zero by switching modes
        self.gs._ensure_mode(self.gs.MODE_GYRO_G_A)
        self.cs._ensure_mode(self.cs.MODE_COL_REFLECT)

        # If the motors are inverted xor the gear ratio is negative, set the motor poloraity to be inverted, 
        # so normal motor commands will run the motors in the opposite direction.
        if self.MotorInverted ^ (self.GearRatio / abs(self.GearRatio) == -1):
            self.lm.polarity = "inversed"
            self.rm.polarity = "inversed"
            self.tank.set_polarity = "inversed"
        else:
            self.lm.polarity = "normal"
            self.rm.polarity = "normal"
            self.tank.set_polarity = "normal"
        
        # Set the integer GyroInvertedNum to reflect the boolean GyroInverted, with -1 = True, and 1 = False, 
        # for use in calculations later
        if self.GyroInverted:
            self.GyroInvertedNum = -1
        else:
            self.GyroInvertedNum = 1
        
        # Beep to signify the robot isdone initialization (it takes a while)
        self.spkr.beep()

    def reflectCal(self):
        """
        "Calibrate" the color sensor using high and low setpoints, and a simple linear mapping.
        The adjusted value can then be accessed using robot.correctedRLI, and the raw value can be acessed using
        robot.cs.reflected_light_intensity.

        Use:  When called, the robot will emit a high-pitched beep.  This is a signal to place the robot on a white surface
        and press the center button.  The robot then emits a low-pitched beep.  This is a signal to repeat the procedure with a
        black surface.
        """
        # These variables need to be accessed by the correctedRLI function, and thus need to be global.
        global reflHighVal
        global reflLowVal
        global reflRate
        # Signal for white
        self.spkr.beep('-f 880')
        # Wait for user conformation of a white surface
        self.btn.wait_for_bump('enter')
        # Set High fixpoint
        reflHighVal = (self.cs.reflected_light_intensity)
        # Conformation of completion
        self.spkr.beep()
        # Signal for black
        self.spkr.beep('-f 220')
        # Wait for user conformation of a black surface
        self.btn.wait_for_bump('enter')
        # Set Low fixpoint
        reflLowVal = self.cs.reflected_light_intensity
        # Conformation of completion
        self.spkr.beep()
        # Calculate the slope of the linear function that maps the fixpoints to 0 - 100
        reflRate = 100 / (reflHighVal - reflLowVal)

    @property
    def correctedRLI(self):
        """
        Returns the reflected light intensity from the color sensor, scaled based on the high and low values created by reflectCal

        This means LineFollow can use 50 as the target, even though the actual reading for black might be 20, and white 75, as it
        will be scaled to 0 - 100.
        """
        # Calculates adjusted value with a linear mapping.  To see how this works go here: https://www.desmos.com/calculator/d4mudhrdng
        value = min([100, max([0, (self.cs.reflected_light_intensity * reflRate) + (-reflLowVal * reflRate)])])
        return(value)

    @property
    def correctedAngle(self):
        """
        Retuns the gyro value corrected for the orientation of the gyro in the robot; turning right will always increase the value,
        and turning left will always decrease the value.  The raw value can be accessed with robot.gs.angle_and_rate[0]

        Angle and rate is used to prevent switching modes and resetting the angle.
        """

        # Multiply the gyro angle by -1 if the gyro is mounted upside-down relative to the motors in the robot.
        # GyroInvertedNum is set up in __init__()
        return(self.gs.angle_and_rate[0] * self.GyroInvertedNum)

    def zeroGyro(self):
        """
        Reset the gyro angle to zero by switching modes.  gyro.reset would have been used instead of this function, 
        but it does not work
        """
        self.gs._ensure_mode(self.gs.MODE_GYRO_RATE)
        self.gs._ensure_mode(self.gs.MODE_GYRO_G_A)

    def DriveAtHeading(self, Heading, Distance, Speed, Stop):
        """
        Moves the robot in a specified direction at a specified speed for a certian number of centimeters, while using the gyro sensor to keep the robot moving in a straight line.

        ``Heading``: The angle at which to drive, with the direction the gyro was last calibrated in being zero.
        ``Distance``: The distance to drive, in centimeters (positive only).
        ``Speed``: The speed at which to drive, in motor percentage (same speed units as EV3-G).  A negative value will make the robot drive backwards.
        ``Stop``: Stop motors after completion.  If ``FALSE``, motors will continue running after ``Distance`` has been traveled.  Otherwise, motors will stop after ``Distance`` cm.
        """
        # Ensure values are within reasonable limits, and change them if necessary (Idiotproofing).
        if Distance <= 0:
            print("Distance must be greater than zero.  Use negative speed to drive backwards.")
            return
        elif Distance > 265:
            # The longest distance on an FLL table (diagonal) is about 265cm.
            if self.ForFLL:
                print("Please don't use silly distances (max = 265cm)")
                return
        if Speed > 75:
            Speed = 75
            print("Speed must be between -75 and 75 (inclusive).")
        elif Speed < -75:
            Speed = -75
            print("Speed must be between -75 and 75 (inclusive).")

        # "Reset" motor encoders by subtracting start values
        left_motor_start = self.lm.degrees
        right_motor_start = self.rm.degrees
        left_motor_now = self.lm.degrees
        right_motor_now = self.rm.degrees
        left_motor_change = left_motor_now - left_motor_start
        right_motor_change = right_motor_now - right_motor_start

        # Determine the sign of the speed, for PID correction
        sign = Speed / abs(Speed)

        # Find number of degrees that motors need to rotate to reach the desired number of cm.
        target = (Distance * 360) / self.WheelCircumference * abs(self.GearRatio)

        # Find the average of the left and right encoders, as they could be different from PID correction
        avg = abs((left_motor_change + right_motor_change) / 2)

        # Initialize variables for PID control
        integral = 0.0
        last_error = 0.0
        derivative = 0.0

        # Check if the motors have gone far enough
        while avg < target:
            # Read the gyro
            current_angle = self.correctedAngle

            # Calculate the PID components
            error = current_angle - Heading
            integral = integral + error
            derivative = error - last_error
            last_error = error

            # Calculate Steering value based on PID components and kp, ki, and kd
            turn_native_units = sign * max([min([(self.kp * error) + (self.ki * integral) + (self.kd * derivative), 100]), -100])

            # Check if the motors will stop at the end.  If not, the speed will be adjusted to come to a smooth stop.
            if Stop == False:
                speedMult = 1
            else:
                # Check if the robot has gone 70% or more of the distance.  If so, start slowing down
                if (target - avg) <= (0.3 * target):
                    # Calculate the pecrentage of the distance left to travel
                    targDist = 1 - (avg / target)
                    # Calculate speedMult based on pecentage; linear function was designed to bring the robot to a
                    # smooth stop while still reaching the target, resulting in 20% of the intial speed at end.
                    speedMult = ((8 / 3) * targDist) + 0.2
                else:
                    speedMult = 1

            # Start the motors, using the steering value calculated by the PID control, and the input speed multiplied by speedMult (0 - 1, see above).
            self.steer.on(-turn_native_units, (Speed * speedMult))

            # Update corrected encoder values
            left_motor_now = self.lm.degrees
            right_motor_now = self.rm.degrees
            left_motor_change = left_motor_now - left_motor_start
            right_motor_change = right_motor_now - right_motor_start
            avg = abs((left_motor_change + right_motor_change) / 2)
        
        # If the robot is to stop, stop the motors.  Otherwise, leave the motors on and return.
        if not Stop == False:
            self.steer.stop()
    
    def degrees2power(self, currentDifference):
        """
        Returns a power value based on the degrees left to turn, using a linear function that allows the robot
        to reach the desired angle, while slowing down as to not overshoot.  Used only in GyroTurn.
        """
        if currentDifference == 0:
            # Minimum power value is 5
            return(5)
        # Using and returning absolute values, for simplicity.  GyroTurn fixes the sign.
        currentDifference = abs(currentDifference)
        # Return calculated value
        return(min([50, max([5, ((0.125 * currentDifference) + 4.875)])]))

    def GyroTurn(self, Heading):
        """
        Turns the robot to ``Heading``, slowing down as it approaches.  Will unwind any full circles
        (if the gyro reads 540, and the value given is 90, the robot will spin until reaching 90, not 450).
        """

        # Initalize the sign variable (used to correct overshoot)
        sign = 1

        # If the current heading is equal to the desired heading, no turning is needed.
        if Heading - self.correctedAngle == 0:
            return
        
        # Read the gyro, and store in currentHeading
        currentHeading = self.correctedAngle
        # Continue turning until the robot is within 1 degree of the target (can be reduced if necessary)
        while (currentHeading > 0.5 + Heading) or (currentHeading < Heading - 0.5):
            # Calculate the difference between where the robot should be and where it is
            currentDifference = Heading - currentHeading
            # The sign variable defines the direction in which to turn. It should have the same sign as the currentDifference variable.
            # If not, multiply by -1 to fix it.
            if ((sign > 0) and (currentDifference < 0)) or ((sign < 0) and (currentDifference > 0)):
                sign *= -1
            # Set the power variable to the absolute power (calculated by degrees2power) multiplied by the sign variable
            power = sign * self.degrees2power(currentDifference)
            # Start turning
            self.tank.on(power, -1 * power)
            # Update currentHeading
            currentHeading = self.correctedAngle
        # When the loop finishes, stop the motors.
        self.tank.stop()

    def ArcTurn(self, Degrees, Radius, Speed):
        """
        Drive the robot in an arc with a specified radius, for a certain number of degrees.

        ``Degrees``:  The number or degrees to drive around the arc.  Positive will turn the front of the robot right, negative left (car turn).  
        ``Radius``:  The radius of the arc to drive, in cm.  Must be positive.
        ``Speed``:  The speed at which to drive around the arc, in percentage of full power (same units as EV3-G).  Negative will drive
        backwards.
        """

        # Ensure the parameters are within reasonable limits, and adjust/reject as necessary.
        if Speed > 75:
            Speed = 75
            print("Speed must be between -75 and 75 (inclusive).")
        elif Speed < -75:
            Speed = -75
            print("Speed must be between -75 and 75 (inclusive).")
        if Radius <= 0:
            print("Radius must be greater than zero.  Use negative degrees to turn the opposite direction.")
            return
        # No point in driving in circles
        Degrees = math.fmod(Degrees, 360)

        # Read the angle of the robot, and store in startHeading
        startHeading = self.correctedAngle

        # If both Degrees and Speed have the same sign, the right wheel should be slowed down.
        # Otherwise, slow down the left wheel.
        if ((Degrees > 0) and (Speed > 0)) or ((Degrees < 0) and (Speed < 0)):
            self.tank.on(Speed, (Radius - self.WidthBetweenWheels) * Speed / Radius)
        else:
            self.tank.on((Radius - self.WidthBetweenWheels) * Speed / Radius, Speed)
        
        # If Degrees is positive, wait for the degrees turned to become >= Degrees (to turn).
        # Otherwise, wait for the degrees turned to become <= Degrees
        if Degrees > 0:
            while (self.correctedAngle - startHeading) < Degrees:
                pass
        else:
            while (self.correctedAngle - startHeading) > Degrees:
                pass
        
        # Stop the motors
        self.tank.stop()
    
    def DriveBump(self, Heading, Speed):
        """
        Moves the robot in a specified direction at a specified speed until it hits something, while using the gyro sensor to keep the robot moving in a straight line.

        ``Heading``: The angle at which to drive, with the direction the gyro was last calibrated in being zero.
        ``Speed``: The speed at which to drive, in motor percentage (same speed units as EV3-G).  A negative value will make the robot drive backwards.
        """           
        
        # Ensure values are within reasonable limits, and change them if necessary (Idiotproofing).
        if Speed > 75:
            Speed = 75
            print("Speed must be between -75 and 75 (inclusive).")
        elif Speed < -75:
            Speed = -75
            print("Speed must be between -75 and 75 (inclusive).")

        # Check and store the sign of the input speed (for PID correction), and convert the target speed to encoder ticks per second
        sign = Speed * -1 / abs(Speed)
        target = abs((self.lm.max_speed * Speed) / 100)

        # Initialize variables for PID control
        integral = 0.0
        last_error = 0.0
        derivative = 0.0

        # Read the gyro
        current_angle = self.correctedAngle

        # Calculate the PID components
        error = current_angle - Heading
        integral = integral + error
        derivative = error - last_error
        last_error = error

        # Calculate Steering value based on PID components and kp, ki, and kd
        turn_native_units = sign * max([min([(self.kp * error) + (self.ki * integral) + (self.kd * derivative), 100]), -100])

        # Start the motors without speed regulation, using the Steering value and Speed
        lsrs = self.steer.get_speed_steering(turn_native_units, Speed) # lsrs = left-speed right-speed
        lsNative = lsrs[0]
        rsNative = lsrs[1]
        self.lm.on(SpeedNativeUnits(lsNative))
        self.rm.on(SpeedNativeUnits(rsNative))

        # Wait for motors to get up to speed, then check and store the average speed (between the two motors)
        time.sleep(1)
        avgSpd = abs((self.lm.speed + self.rm.speed) / 2)

        # Check if the motors have slowed down (because the robot hit something)
        while avgSpd > 0.90 * target:
            # Read the gyro
            current_angle = self.correctedAngle

            # Calculate the PID components
            error = current_angle - Heading
            integral = integral + error
            derivative = error - last_error
            last_error = error

            # Calculate Steering value based on PID components and kp, ki, and kd
            turn_native_units = sign * max([min([(self.kp * error) + (self.ki * integral) + (self.kd * derivative), 100]), -100])

            # Start the motors without speed regulation, using the Steering value and Speed
            lsrs = self.steer.get_speed_steering(turn_native_units, Speed)
            lsNative = lsrs[0]
            rsNative = lsrs[1]
            self.lm.on(SpeedNativeUnits(lsNative))
            self.rm.on(SpeedNativeUnits(rsNative))

            # Check and store the average speed again
            avgSpd = abs((self.lm.speed + self.rm.speed) / 2)
        
        # Stop the motors
        self.lm.stop()
        self.rm.stop()

    def AuxMotorBumpStop(self, Speed, Threshold, Port):
        """
        Similar to DriveBump, will run an auxillary motor until the speed drops below ``Threshold``% of ``Speed``.

        ``Speed``: Speed at which to run the motor, in motor percentage (Same units as EV3-G).
        ``Threshold``: Percentage of ``Speed`` that the motor speed must drop below before shutting off.
        ``Port``: Motor port.
        """
        # Most if statments are only because there are two possible ports; one case for each port

        # Ensure values are within reasonable, and adjust/reject as necessary
        if Speed > 75:
            Speed = 75
            print("Speed must be between -75 and 75 (inclusive).")
        elif Speed < -75:
            Speed = -75
            print("Speed must be between -75 and 75 (inclusive).")
        if Threshold <= 0:
            print("Threshold must be greater than zero and less than one")
            return
        elif Threshold > 100:
            print("Threshold must be greater than zero and less than or equal to 100")
            return
        
        # Different commands for different motors, check which motor to use
        if Port == self.AuxMotor1:
            # Find the target speed in encoder ticks per second
            target = abs((self.m1.max_speed * Speed) / 100)
            # Start the motor, with the calculated speed
            self.m1.on(SpeedNativeUnits(target))
            # Wait for the motor to get up to speed
            time.sleep(0.5)
            # Set motrspeed to the current motor speed
            motrspeed = abs(self.m1.speed)
        else:
            # Everything here is the same as the first case statement, but using m2 instead of m1 (motor 2, not motor 1)
            target = abs((self.m2.max_speed * Speed) / 100)
            self.m2.on(SpeedNativeUnits(target))
            time.sleep(0.5)
            motrspeed = abs(self.m2.speed)
            # Keep the motor on until the motor speed drops below Threshold% of Speed
        while motrspeed > (target * Threshold) / 100:
            if Port == self.AuxMotor1:
                # Update motrspeed until the motor slows down enough to stop
                motrspeed = abs(self.m1.speed)
            else:
                # Same as before, just for the other motor
                motrspeed = abs(self.m2.speed)
        # Shut off the motor
        if Port == self.AuxMotor1:
            self.m1.off()
        else:
            self.m2.off()
    
    def LineStop(self, Heading, Speed, Stop):
        """
        Moves the robot in a specified direction at a specified speed until a line (White-Black) is seen, while using the gyro sensor to keep the robot moving in a straight line.

        ``Heading``: The angle at which to drive, with the direction the gyro was last calibrated in being zero.
        ``Speed``: The speed at which to drive, in motor percentage (same speed units as EV3-G).  A negative value will make the robot drive backwards.
        ``Stop``: Stop motors after completion.  If ``FALSE``, motors will continue running after ``Distance`` has been traveled.  Otherwise, motors will stop after ``Distance`` cm.
        """
        # Ensure values are within reasonable limits, and change them if necessary (Idiotproofing).
        if Speed > 75:
            Speed = 75
            print("Speed must be between -75 and 75 (inclusive).")
        elif Speed < -75:
            Speed = -75
            print("Speed must be between -75 and 75 (inclusive).")

        # Check and store the sign of the input speed for PID correction
        sign = Speed / abs(Speed)

        # Set the color sensor to "Color" mode
        self.cs._ensure_mode(self.cs.MODE_COL_COLOR)

        # Set the brick light to solid amber
        #EV3.SetLEDColor("ORANGE", "NORMAL")

        # Initialize variables for PID control and end checking
        integral = 0.0
        last_error = 0.0
        derivative = 0.0
        end = False
        seenWhite = False

        # Check if the motors have gone far enough
        while not end:
            # Read the gyro
            current_angle = self.correctedAngle

            # Calculate the PID components
            error = current_angle - Heading
            integral = integral + error
            derivative = error - last_error
            last_error = error

            # Calculate Steering value based on PID components and kp, ki, and kd
            turn_native_units = sign * max([min([(self.kp * error) + (self.ki * integral) + (self.kd * derivative), 100]), -100])

            # Start the motors, using the steering value calculated by the PID control, and the input speed multiplied by speedMult (0 - 1, see above).
            self.steer.on(-turn_native_units, (Speed))

            # Check if the sensor is seeing white
            color = self.cs.color
            if color == self.cs.COLOR_WHITE:
                seenWhite = True
            elif (color == self.cs.COLOR_BLACK) and seenWhite:
                end = True
            elif color == self.cs.COLOR_BLUE:
                seenWhite = seenWhite
            else:
                seenWhite = False
        
        # If the robot is to stop, stop the motors.  Otherwise, leave the motors on.
        if not Stop == False:
            self.steer.stop()
        
        # Set the brick light back to green flashing
        #EVS.SetLEDColor("GREEN", "PULSE")
    
    def LineFollow(self, Distance, Speed, Stop):
        """
        Follows the edge of a line for a specific distance at a specific speed.

        ``Distance``: The distance to drive, in centimeters (positive only).
        ``Speed``: The speed at which to drive, in motor percentage (same speed units as EV3-G).  (Positive only; there's no going back)
        ``Stop``: Stop motors after completion.  If ``FALSE``, motors will continue running after ``Distance`` has been traveled.  Otherwise, motors will stop after ``Distance`` cm.
        """
        # Ensure values are within reasonable limits, and change them if necessary (Idiotproofing).
        if Speed > 50:
            Speed = 50
            print("Speed must be between -50 and 50 (inclusive).")
        elif Speed < -50:
            Speed = -50
            print("Speed must be between -50 and 50 (inclusive).")

        # "Reset" motor encoders by subtracting start values
        left_motor_start = self.lm.degrees
        right_motor_start = self.rm.degrees
        left_motor_now = self.lm.degrees
        right_motor_now = self.rm.degrees
        left_motor_change = left_motor_now - left_motor_start
        right_motor_change = right_motor_now - right_motor_start

        # Determine the sign of the speed, for PID correction
        sign = Speed / abs(Speed)

        # Find number of degrees that motors need to rotate to reach the desired number of cm.
        target = (Distance * 360) / self.WheelCircumference * abs(self.GearRatio)

        # Find the average of the left and right encoders, as they could be different from PID correction
        avg = abs((left_motor_change + right_motor_change) / 2)

        # Initialize variables for PID control
        integral = 0.0
        last_error = 0.0
        derivative = 0.0

        # Check if the motors have gone far enough
        while avg < target:
            # Read the gyro
            current_RLI = self.correctedRLI

            # Calculate the PID components
            error = 50 - current_RLI
            integral = integral + error
            derivative = error - last_error
            last_error = error

            # Calculate Steering value based on PID components and kp, ki, and kd
            turn_native_units = sign * max([min([(self.kpLine * error) + (self.kiLine * integral) + (self.kdLine * derivative), 100]), -100])

            # Check if the motors will stop at the end.  If not, the speed will be adjusted to come to a smooth stop.
            if Stop == False:
                speedMult = 1
            else:
                # Check if the robot has gone 70% or more of the distance.  If so, start slowing down
                if (target - avg) <= (0.3 * target):
                    # Calculate the pecrentage of the distance left to travel
                    targDist = 1 - (avg / target)
                    # Calculate speedMult based on pecentage; linear function was designed to bring the robot to a
                    # smooth stop while still reaching the target.
                    speedMult = ((8 / 3) * targDist) + 0.2
                else:
                    speedMult = 1

            # Start the motors, using the steering value calculated by the PID control, and the input speed multiplied by speedMult (0 - 1, see above).
            self.steer.on(-turn_native_units, (Speed * speedMult))

            # Update corrected encoder values
            left_motor_now = self.lm.degrees
            right_motor_now = self.rm.degrees
            left_motor_change = left_motor_now - left_motor_start
            right_motor_change = right_motor_now - right_motor_start
            avg = abs((left_motor_change + right_motor_change) / 2)
        
        # If the robot is to stop, stop the motors.  Otherwise, leave the motors on and return.
        if not Stop == False:
            self.steer.stop()

    def TriangleAvoid(self, Heading, Distance, Speed):
        """
        Moves the robot in a specified direction at a specified speed for a certian number of centimeters, while using the gyro sensor to keep the robot moving in a straight line.

        ``Heading``: The angle at which to drive, with the direction the gyro was last calibrated in being zero.
        ``Distance``: The distance to drive, in centimeters (positive only).
        ``Speed``: The speed at which to drive, in motor percentage (same speed units as EV3-G).  A negative value will make the robot drive backwards.
        ``Stop``: Stop motors after completion.  If ``FALSE``, motors will continue running after ``Distance`` has been traveled.  Otherwise, motors will stop after ``Distance`` cm.
        """
        # Ensure values are within reasonable limits, and change them if necessary (Idiotproofing).
        if Distance <= 0:
            print("Distance must be greater than zero.  Use negative speed to drive backwards.")
            return
        elif Distance > 265:
            # The longest distance on an FLL table (diagonal) is about 265cm.
            if self.ForFLL:
                print("Please don't use silly distances (max = 265cm)")
                return
        if Speed > 75:
            Speed = 75
            print("Speed must be between -75 and 75 (inclusive).")
        elif Speed < -75:
            Speed = -75
            print("Speed must be between -75 and 75 (inclusive).")
        if not self.us:
            print("Avoidance Functions need the US sensor to work.")
            return

        # "Reset" motor encoders by subtracting start values
        left_motor_start = self.lm.degrees
        right_motor_start = self.rm.degrees
        left_motor_now = self.lm.degrees
        right_motor_now = self.rm.degrees
        left_motor_change = left_motor_now - left_motor_start
        right_motor_change = right_motor_now - right_motor_start

        # Determine the sign of the speed, for PID correction
        sign = Speed / abs(Speed)

        # Find number of degrees that motors need to rotate to reach the desired number of cm.
        target = (Distance * 360) / self.WheelCircumference * abs(self.GearRatio)

        # Find the average of the left and right encoders, as they could be different from PID correction
        avg = abs((left_motor_change + right_motor_change) / 2)

        # Find the number of centimeters driven and left to go
        driven_so_far = (avg * self.WheelCircumference) / 360
        left_to_go = Distance - driven_so_far

        # Initialize variables for PID control
        integral = 0.0
        last_error = 0.0
        derivative = 0.0

        # Check if the motors have gone far enough
        while avg < target:
            # Read the gyro and ultrasonic sensors
            current_angle = self.correctedAngle
            dist_to_obstacle = self.us.distance_centimeters

            # Calculate the PID components
            error = current_angle - Heading
            integral = integral + error
            derivative = error - last_error
            last_error = error

            # Calculate Steering value based on PID components and kp, ki, and kd
            turn_native_units = sign * max([min([(self.kp * error) + (self.ki * integral) + (self.kd * derivative), 100]), -100])

            # Check if the motors will stop at the end.  If not, the speed will be adjusted to come to a smooth stop.
            # Check if the robot has gone 70% or more of the distance.  If so, start slowing down
            if (target - avg) <= (0.3 * target):
                # Calculate the pecrentage of the distance left to travel
                targDist = 1 - (avg / target)
                # Calculate speedMult based on pecentage; linear function was designed to bring the robot to a
                # smooth stop while still reaching the target, resulting in 20% of the intial speed at end.
                speedMult = ((8 / 3) * targDist) + 0.2
            else:
                speedMult = 1

            if (dist_to_obstacle <= 30) and (left_to_go > 30):
                self.steer.off(brake=False)
                left_motor_now = self.lm.degrees
                right_motor_now = self.rm.degrees
                left_motor_change = left_motor_now - left_motor_start
                right_motor_change = right_motor_now - right_motor_start
                avg = abs((left_motor_change + right_motor_change) / 2)
                driven_so_far = (avg * self.WheelCircumference) / 360
                dist_to_obstacle = self.us.distance_centimeters
                start_angle = self.correctedAngle
                while self.us.distance_centimeters < 60:
                    self.tank.on(-10, 10)
                self.tank.off()
                self.GyroTurn(self.correctedAngle - 5)
                end_angle = self.correctedAngle
                degrees_turned = abs(end_angle - start_angle)
                first_hypotenuse = dist_to_obstacle / math.cos(math.radians(degrees_turned))
                self.DriveAtHeading(end_angle, first_hypotenuse, 20, True)
                second_triangle_short_leg = math.sqrt((first_hypotenuse ** 2) - (dist_to_obstacle ** 2))
                second_triangle_long_leg = Distance - (driven_so_far + dist_to_obstacle + 2.21)
                second_triangle_second_angle = math.degrees(math.atan(second_triangle_long_leg / second_triangle_short_leg))
                degrees_to_turn = 90 - second_triangle_second_angle
                self.GyroTurn(Heading)
                self.GyroTurn(Heading + degrees_to_turn)
                second_hypotenuse = math.sqrt((second_triangle_long_leg ** 2) + (second_triangle_short_leg ** 2))
                self.TriangleAvoid(self.correctedAngle, second_hypotenuse, 20)
                self.GyroTurn(Heading)
                return


            # Start the motors, using the steering value calculated by the PID control, and the input speed multiplied by speedMult (0 - 1, see above).
            self.steer.on(-turn_native_units, (Speed * speedMult))

            # Update corrected encoder values
            left_motor_now = self.lm.degrees
            right_motor_now = self.rm.degrees
            left_motor_change = left_motor_now - left_motor_start
            right_motor_change = right_motor_now - right_motor_start
            avg = abs((left_motor_change + right_motor_change) / 2)
            driven_so_far = (avg * self.WheelCircumference) / 360
        
        # Stop the motors.
        self.steer.stop()