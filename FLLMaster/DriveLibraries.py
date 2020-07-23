from ev3dev2.motor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor import *

class Robot:
    from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, LargeMotor
    from ev3dev2.sensor.lego import ColorSensor, GyroSensor, InfraredSensor, TouchSensor, UltrasonicSensor
    from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
    from ev3dev2.sound import Sound
    from ev3dev2.button import Button
    from ev3dev2.led import Leds, LED_COLORS, LED_DEFAULT_COLOR, LED_GROUPS, LEDS
    import math
    
    def __init__(self, filename):
        from configparser import SafeConfigParser
        conf = SafeConfigParser()
        conf.read(filename)

        self.ForFLL = bool(conf.get('Other', 'ForFLL') == "TRUE")

        self.spkr = self.Sound()
        self.btn = self.Button()
        self.led = self.Leds()

        self.LeftMotor = eval(conf.get('Drivebase', 'LeftMotorPort'))
        self.RightMotor = eval(conf.get('Drivebase', 'RightMotorPort'))
        self.WheelCircumference = float(conf.get('Drivebase', 'WheelCircumference'))
        self.WidthBetweenWheels = float(conf.get('Drivebase', 'WidthBetweenWheels'))
        self.MotorInverted = bool(conf.get('Drivebase', 'MotorInverted') == "TRUE")
        self.GearRatio = float(conf.get('Drivebase', 'GearRatio'))
        self.GyroInverted = bool(conf.get('Drivebase', 'GyroInverted') == "TRUE")
        self.kp = float(conf.get('Drivebase', 'kp'))
        self.ki = float(conf.get('Drivebase', 'ki'))
        self.kd = float(conf.get('Drivebase', 'kd'))

        self.ColorPort = eval(conf.get('Sensors', 'ColorPort'))
        self.GyroPort = eval(conf.get('Sensors', 'GyroPort'))
        self.InfraredPort = eval(conf.get('Sensors', 'InfraredPort'))
        self.TouchPort = eval(conf.get('Sensors', 'TouchPort'))
        self.UltrasonicPort = eval(conf.get('Sensors', 'UltrasonicPort'))

        self.AuxMotor1 = eval(conf.get('AuxMotors', 'AuxMotor1'))
        self.AuxMotor2 = eval(conf.get('AuxMotors', 'AuxMotor2'))

        #self.m1 = MediumMotor(self.AuxMotor1)
        #self.m2 = MediumMotor(self.AuxMotor2)


        self.cs = ColorSensor(self.ColorPort)
        self.gs = GyroSensor(self.GyroPort)
        self.ir = InfraredSensor(self.InfraredPort)
        self.us = UltrasonicSensor(self.UltrasonicPort)

        self.tank = MoveTank(self.LeftMotor, self.RightMotor)
        self.tank.gyro = GyroSensor(self.GyroPort)
        self.steer = MoveSteering(self.LeftMotor, self.RightMotor)
        self.lm = LargeMotor(self.LeftMotor)
        self.rm = LargeMotor(self.RightMotor)

        self.gs._ensure_mode(self.gs.MODE_GYRO_G_A)
        self.cs._ensure_mode(self.cs.MODE_COL_REFLECT)

        if self.MotorInverted ^ (self.GearRatio / abs(self.GearRatio) == -1):
            self.lm.polarity = "inversed"
            self.rm.polarity = "inversed"
            self.tank.set_polarity = "inversed"
        else:
            self.lm.polarity = "normal"
            self.rm.polarity = "normal"
            self.tank.set_polarity = "normal"
        
        if self.GyroInverted:
            self.GyroInvertedNum = -1
        else:
            self.GyroInvertedNum = 1
        
        self.spkr.beep()

    def reflectCal(self):
        global reflHighVal
        global reflLowVal
        global reflRate
        self.spkr.speak("Show me white")
        self.btn.wait_for_bump('enter')
        reflHighVal = (self.cs.reflected_light_intensity)
        self.spkr.beep()
        self.spkr.speak("Show me black")
        self.btn.wait_for_bump('enter')
        reflLowVal = self.cs.reflected_light_intensity
        self.spkr.beep()
        reflRate = 100 / (reflHighVal - reflLowVal)

    @property
    def correctedRLI(self):
        value = min([100, max([0, (self.cs.reflected_light_intensity * reflRate) + (-reflLowVal * reflRate)])])
        return(value)

    @property
    def correctedAngle(self):
        # Multiply the gyro angle by -1 if the gyro is mounted upside-down relative to the motors in the robot.
        # GyroInvertedNum is set up in __init__()
        return(self.gs.angle_and_rate[0] * self.GyroInvertedNum)

    def zeroGyro(self):
        # Reset the gyro angle to zero by switching modes. gyro.reset would have been used instead of this function, but it does not work
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
        target = (Distance * 360) / self.WheelCircumference

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
    
    def degrees2power(self, currentDifference):
        if currentDifference == 0:
            return(5)
        currentDifference = abs(currentDifference)
        return(min([50, max([5, ((0.125 * currentDifference) + 4.875)])]))

    def GyroTurn(self, Heading):
        sign = 1

        if Heading - self.correctedAngle == 0:
            return
        
        currentHeading = self.correctedAngle
        while (currentHeading > 0.5 + Heading) or (currentHeading < Heading - 0.5):
            currentDifference = Heading - currentHeading
            if ((sign > 0) and (currentDifference < 0)) or ((sign < 0) and (currentDifference > 0)):
                sign *= -1
            power = sign * self.degrees2power(currentDifference)
            self.tank.on(power, -1 * power)
            currentHeading = self.correctedAngle
        self.tank.stop()

    def ArcTurn(self, Degrees, Radius, Speed):
        if Speed > 75:
            Speed = 75
            print("Speed must be between -75 and 75 (inclusive).")
        elif Speed < -75:
            Speed = -75
            print("Speed must be between -75 and 75 (inclusive).")
        if Radius <= 0:
            print("Radius must be greater than zero.  Use negative degrees to turn the opposite direction.")
            return
        math.fmod(Degrees, 360)

        startHeading = self.correctedAngle

        if ((Degrees > 0) and (Speed > 0)) or ((Degrees < 0) and (Speed < 0)):
            self.tank.on(Speed, (Radius - self.WidthBetweenWheels) * Speed / Radius)
        else:
            self.tank.on((Radius - self.WidthBetweenWheels) * Speed / Radius, Speed)
        
        if Degrees > 0:
            while (self.correctedAngle - startHeading) < Degrees:
                dummy = 1
        else:
            while (self.correctedAngle - startHeading) > Degrees:
                dummy = 1
        
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
        lsrs = self.steer.get_speed_steering(turn_native_units, Speed)
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
        
        target = abs((self.lm.max_speed * Speed) / 100)
        
        if Port == self.AuxMotor1:
            msNative = (Speed * self.m1.max_speed) / 100
            self.m1.on(SpeedNativeUnits(msNative))
            time.sleep(0.5)
            motrspeed = abs(self.m1.speed)
        else:
            msNative = (Speed * self.m2.max_speed) / 100
            self.m2.on(SpeedNativeUnits(msNative))
            time.sleep(0.5)
            motrspeed = abs(self.m2.speed)
        while motrspeed > (target * Threshold) / 100:
            if Port == self.AuxMotor1:
                motrspeed = abs(self.m1.speed)
            else:
                motrspeed = abs(self.m2.speed)
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
            if self.correctedRLI >= 95:
                self.spkr.beep(play_type=1)
                seenWhite = True
            elif (self.correctedRLI <= 10) and seenWhite:
                end = True
            else:
                seenWhite = False
        
        # If the robot is to stop, stop the motors.  Otherwise, leave the motors on.
        if not Stop == False:
            self.steer.stop()
        
        # Set the brick light back to green flashing
        #EVS.SetLEDColor("GREEN", "PULSE")