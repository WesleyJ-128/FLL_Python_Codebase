# Copyright (c) 2009-2021 FIRST and other WPILib contributors
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of FIRST, WPILib, nor the names of other WPILib
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY FIRST AND OTHER WPILIB CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY NONINFRINGEMENT AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL FIRST OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Everything in this file is directly ported from the Java WPILib provided for FRC.


import math
import time

class Rotation2d:
    """
    A rotation in a 2d coordinate frame represented a point on the unit circle (cosine and sine).
    """
    def __init__(self, value = 0.0):
        """
        Constructs a Rotation2d with the given radian value. The x and y don't have to be normalized.
        
        ``value``: The value of the angle in radians.
        """

        self.value = value
        self.cos = math.cos(value)
        self.sin = math.sin(value)

    @classmethod
    def fromDegrees(cls, degrees):
        """
        Constructs and returns a Rotation2d with the given degree value.

        ``degrees``: The value of the angle in degrees.
        """
        return cls(math.radians(degrees))

    @classmethod
    def fromCoords(cls, x, y):
        """
        Constructs and returns a Rotation2d with the given x and y (cosine and sine) components.

        ``x``: The x component or cosine of the rotation.

        ``y``: The y component or sine of the rotation.
        """
        magnitude = math.hypot(x, y)
        if magnitude > 1e-6:
            sin = y / magnitude
            cos = x / magnitude
        else:
            sin = 0.0
            cos = 1.0
        return cls(math.atan2(sin, cos))
    
    def plus(self, other: "Rotation2d"):
        """
        Adds two rotations together, with the result being bounded between -pi and pi.
        
        For example, Rotation2d.fromDegrees(30) + Rotation2d.fromDegrees(60) = Rotation2d{-pi/2}
        
        ``other``: The rotation to add.

        Returns: The sum of the two rotations.
        """
        return self.rotateBy(other)
    
    def minus(self, other: "Rotation2d"):
        """
        Subtracts the new rotation from the current rotation and returns the new rotation.
        
        For example, Rotation2d.fromDegrees(10) - Rotation2d.fromDegrees(100) = Rotation2d{-pi/2}
        
        ``other``: The rotation to subtract.
        
        Returns: The difference between the two rotations.
        """
        return self.rotateBy(other.unaryMinus())
    
    def unaryMinus(self):
        """
        Takes the inverse of the current rotation. This is simply the negative of the current angular value.
        
        Returns: The inverse of the current rotation.
        """
        return Rotation2d(-self.value)
    
    def times(self, scalar):
        """
         Multiplies the current rotation by a scalar.

         ``scalar``: The scalar.

         Returns:  The new scaled Rotation2d.
        """
        return Rotation2d(self.value * scalar)

    def rotateBy(self, other: "Rotation2d"):
        """
        Adds the new rotation to the current rotation using a rotation matrix.
        
        The matrix multiplication is as follows:

        [cos_new]   [other.cos, -other.sin][cos]
        [sin_new] = [other.sin,  other.cos][sin]
        value_new = atan2(sin_new, cos_new)

        ``other``: The rotation to rotate by.
        Returns: The new rotated Rotation2d.
        """
        return Rotation2d.fromCoords(self.cos * other.cos - self.sin * other.sin, self.cos * other.cos + self.sin * other.sin)

    def getRadians(self) -> float:
        """
        Returns the radian value of the rotation
        """
        return self.value
    
    def getDegrees(self):
        """
        Returns the degree value of the rotation
        """
        return math.degrees(self.value)
    
    def getCos(self):
        """
        Returns the cosine of the rotation
        """
        return self.cos

    def getSin(self):
        """
        Returns the sine of the rotation
        """
        return self.sin

    def getTan(self):
        """
        Returns the tangent of the rotation
        """
        return self.sin / self.cos
    
    def __repr__(self):
        return "Rotation2d(Rads: {}, Deg: {})".format(self.value, math.degrees(self.value))
    
    def __eq__(self, obj):
        """
        Checks equality between this Rotation2s and another object.

        ``obj``: The other object.
        """
        if isinstance(obj, Rotation2d):
            return math.hypot(self.cos - obj.cos, self.sin - obj.sin) < 1e-9
        else:
            return False

class Translation2d:
    """
    Represents a translation in 2d space. This object can be used to represent a point or a vector.

    This assumes that you are using conventional mathematical axes. When the robot is placed on 
    the origin, facing toward the X direction, moving forward increases the X, whereas moving to the
    left increases the Y.
    """
    def __init__(self, x = 0.0, y = 0.0):
        """
        Constructs a Translation2d with the X and Y components equal to the provided values.

        ``x``: The x component of the translation.
        
        ``y``: The y component of the translation.
        """
        self.x = x
        self.y = y

    @classmethod
    def polar(cls, distance, angle: Rotation2d):
        """
        Constructs and returns a Translation2d with the provided distance and angle. This is essentially converting
        from polar coordinates to Cartesian coordinates.

        ``distance`` The distance from the origin to the end of the translation.
        
        ``angle`` The angle between the x-axis and the translation vector.
        """
        return cls(distance * angle.getCos(), distance * angle.getSin())
    
    def getDistance(self, other: "Translation2d"):
        """
        Calculates the distance between two translations in 2d space.

        This function uses the pythagorean theorem to calculate the distance. distance = sqrt((x2 -x1)^2 + (y2 - y1)^2)

        ``other``: The translation to compute the distance to.

        Returns: The distance between the two translations.
        """
        return math.hypot(other.x - self.x, other.y - self.y)
    
    def getX(self):
        """
        Returns the X component of the translation.
        """
        return self.x

    def getY(self):
        """
        Returns the Y component of the translation.
        """
        return self.y
    
    def getNorm(self):
        """
        Returns the norm, or distance from the origin to the translation.
        """
        return math.hypot(self.x, self.y)
    
    def rotateBy(self, other: Rotation2d):
        """
        Applies a rotation to the translation in 2d space.
        
        This multiplies the translation vector by a counterclockwise rotation matrix of the given
        angle. [x_new] [other.cos, -other.sin][x] [y_new] = [other.sin, other.cos][y]
        
        For example, rotating a Translation2d of {2, 0} by 90 degrees will return a Translation2d of
        {0, 2}.

        ``other``: The rotation to rotate the translation by.

        Returns: The new rotated translation.
        """
        return Translation2d(self.x * other.getCos() - self.y * other.getSin(), self.x * other.getSin() + self.y * other.getCos())
    
    def plus(self, other: "Translation2d"):
        """
        Adds two translations in 2d space and returns the sum. This is similar to vector addition.
        
        For example, Translation2d{1.0, 2.5} + Translation2d{2.0, 5.5} = Translation2d{3.0, 8.0}

        ``other``:  The translation to add.

        Returns: The sum of the translations.
        """
        return Translation2d(self.x + other.x, self.y + other.y)
    
    def minus(self, other: "Translation2d"):
        """
        Subtracts the other translation from the other translation and returns the difference.
        
        For example, Translation2d{5.0, 4.0} - Translation2d{1.0, 2.0} = Translation2d{4.0, 2.0}

        ``other``: The translation to subtract.

        Returns: The difference between the two translations.
        """
        return Translation2d(self.x - other.x, self.y-other.y)
    
    def unaryMinus(self):
        """
        Returns the inverse of the current translation. This is equivalent to rotating by 180 degrees,
        flipping the point over both axes, or simply negating both components of the translation.
        """
        return Translation2d(-self.x, -self.y)
    
    def times(self, scalar: float):
        """
        Multiplies the translation by a scalar and returns the new translation.
        
        For example, Translation2d{2.0, 2.5} * 2 = Translation2d{4.0, 5.0}

        ``scalar``: The scalar to multiply by.

        Returns: The scaled translation.
        """
        return Translation2d(self.x * scalar, self.y * scalar)
    
    def div(self, scalar: float):
        """
        Divides the translation by a scalar and returns the new translation.

        For example, Translation2d{2.0, 2.5} / 2 = Translation2d{1.0, 1.25}

        ``scalar``: The scalar to multiply by.

        Returns: The reference to the new mutated object.
        """
        return Translation2d(self.x / scalar, self.y / scalar)
    
    def __repr__(self):
        return "Translation2d(X: {}, Y: {})".format(self.x, self.y)
    
    def __eq__(self, obj: object):
        """
        Checks equality between this Translation2d and another object.
        
        ``obj``:  The other object.
        """
        if isinstance(obj, Translation2d):
            return abs(obj.x - self.x) < 1E-9 and abs(obj.y - self.y) < 1E-9
        else:
            return False

class Pose2d:
    """
    Represents a 2d pose containing translational and rotational elements.
    """
    def __init__(self, translation = Translation2d(), rotation = Rotation2d()):
        """
        Constructs a pose with the specified translation and rotation.
        
        ``translation``: The translational component of the pose.
        
        ``rotation``: The rotational component of the pose.
        """
        self.translation = translation
        self.rotation = rotation

    @classmethod
    def fromCoords(cls, x: float, y: float, rotation: Rotation2d):
        """
        Convenience constructors that takes in x and y values directly instead of having to construct a Translation2d.
        
        ``x``: The x component of the translational component of the pose.
        
        ``y``: The y component of the translational component of the pose.
        
        ``rotation``: The rotational component of the pose.
        """
        return cls(Translation2d(x, y), rotation)
    
    def plus(self, other: "Transform2d"):
        """
        Transforms the pose by the given transformation and returns the new transformed pose.
        
        The matrix multiplication is as follows [x_new] [cos, -sin, 0][transform.x] [y_new] += [sin,
        cos, 0][transform.y] [t_new] [0, 0, 1][transform.t]
        
        ``other``: The transform to transform the pose by.
        """
        return self.transformBy(other)

    def minus(self, other: "Pose2d"):
        """
        Returns the Transform2d that maps the one pose to another.
        
        ``other``: The initial pose of the transformation.
        """
        pose = self.relativeTo(other)
        return Transform2d(pose.getTranslation(), pose.getRotation())

    def getTranslation(self):
        """
        Returns the translation component of the transformation.
        """
        return self.translation

    def getX(self):
        """
        Returns the X component of the transformation.
        """
        return self.translation.getX()

    def getY(self):
        """
        Returns the Y component of the transformation.
        """
        return self.translation.getY()

    def getRotation(self):
        """
        Returns the Rotation component of the transformation.
        """
        return self.rotation
    
    def transformBy(self, other: "Transform2d"):
        """
        Transforms the pose by the given transformation and returns the new pose. See + operator for
        the matrix multiplication performed.

        ``other``: The transform to transform the pose by.
        """
        return Pose2d(
            self.translation.plus(other.getTranslation().rotateBy(self.rotation)),
            self.rotation.plus(other.getRotation()))
    
    def relativeTo(self, other: "Pose2d"):
        """
         Returns the other pose relative to the current pose.

        This function can often be used for trajectory tracking or pose stabilization algorithms to
        get the error between the reference and the current pose.

        ``other``: The pose that is the origin of the new coordinate frame that the current pose will
        be converted into.

        Returns: The current pose relative to the new origin pose.
        """
        transform = Transform2d.from_Poses(other, self)
        return Pose2d(transform.getTranslation(), transform.getRotation())

    def exp(self, twist: "Twist2d"):
        """
        Obtain a new Pose2d from a (constant curvature) velocity.

        See <a href="https://file.tavsys.net/control/controls-engineering-in-frc.pdf">Controls
        Engineering in the FIRST Robotics Competition</a> section 10.2 "Pose exponential" for a
        derivation.

        The twist is a change in pose in the robot's coordinate frame since the previous pose
        update. When the user runs exp() on the previous known field-relative pose with the argument
        being the twist, the user will receive the new field-relative pose.

        "Exp" represents the pose exponential, which is solving a differential equation moving the
        pose forward in time.
        
        ``twist``: The change in pose in the robot's coordinate frame since the previous pose update.
    
        For example, if a non-holonomic robot moves forward 0.01 meters and changes angle by 0.5
        degrees since the previous pose update, the twist would be Twist2d{0.01, 0.0, toRadians(0.5)}
        
        Returns: The new pose of the robot.
        """
        dx = twist.dx
        dy = twist.dy
        dtheta = twist.dtheta

        sinTheta = math.sin(dtheta)
        cosTheta = math.cos(dtheta)

        if abs(dtheta) < 1E-9:
            s = 1.0 -1.0 / 6.0 * dtheta * dtheta
            c = 0.5 * dtheta
        else:
            s = sinTheta / dtheta
            c = (1 - cosTheta) / dtheta
        transform = Transform2d(
            Translation2d(dx * s - dy * c, dx * c + dy * s),
            Rotation2d.fromCoords(cosTheta, sinTheta)
        )
        return self.plus(transform)
    
    def log(self, end: "Pose2d"):
        """
        Returns a Twist2d that maps this pose to the end pose. If c is the output of a.Log(b), then
        a.Exp(c) would yield b.
        
        ``end``: The end pose for the transformation.

        Returns: The twist that maps this to end.
        """
        transform = end.relativeTo(self)
        dtheta = transform.getRotation().getRadians()
        halfDtheta = dtheta / 2.0

        cosMinusOne = transform.getRotation().getCos() - 1

        if abs(cosMinusOne) < 1E-9:
            halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta
        else:
            halfThetaByTanOfHalfDtheta = -(halfDtheta * transform.getRotation().getSin()) / cosMinusOne
        
        translationPart = transform.getTranslation().rotateBy(Rotation2d.fromCoords(halfThetaByTanOfHalfDtheta, -halfDtheta)).times(math.hypot(halfThetaByTanOfHalfDtheta, halfDtheta))

        return Twist2d(translationPart.getX(), translationPart.getY(), dtheta)

    def __repr__(self):
        return "Pose2d({}, {})".format(self.translation, self.rotation)
    
    def __eq__(self, obj: object):
        """
        Checks equality between this Pose2d and another object.
        
        ``obj``: The other object.
        """
        if isinstance(obj, Pose2d):
            return (
                obj.translation.equals(self.translation)
                and obj.rotation.equals(self.rotation)
            )
        else:
            return False

class Transform2d:
    """
    Represents a transformation for a Pose2d.
    """
    def __init__(self, translation = Translation2d(), rotation = Rotation2d()) -> None:
        """
        Constructs a transform with the given translation and rotation components.
        
        ``translation``: Translational component of the transform.
        
        ``rotation``: Rotational component of the transform.
        """
        self.translation = translation
        self.rotation = rotation
    
    @classmethod
    def from_Poses(cls, initial: Pose2d, last: Pose2d):
        """
        Constructs the transform that maps the initial pose to the final pose.
        
        ``initial``: The initial pose for the transformation.
        
        ``last``: The final pose for the transformation.
        """
        # We are rotating the difference between the translations
        # using a clockwise rotation matrix. This transforms the global
        # delta into a local delta (relative to the initial pose).
        translation = last.getTranslation().minus(initial.getTranslation()).rotateBy(initial.getRotation().unaryMinus())
        rotation = last.getRotation().minus(initial.getRotation())
        return cls(translation, rotation)
    
    def times(self, scalar: float):
        """
        Scales the transform by the scalar.

        ``scalar``: The scalar.

        Returns: the scaled Transform2d.
        """
        return Transform2d(self.translation.times(scalar), self.rotation.times(scalar))

    def plus(self, other: "Transform2d"):
        """
        Composes two transformations.

        ``other``: The transform to compose with this one.

        Returns: The composition of the two transformations.
        """
        return Transform2d.from_Poses(Pose2d.empty(), Pose2d.empty().transformBy(self).transformBy(other))

    def getTranslation(self):
        """
        Returns the translation component of the transformation.
        """
        return self.translation

    def getX(self):
        """
        Returns the X component of the transformation.
        """
        return self.translation.getX()

    def getY(self):
        """
        Returns the Y component of the transformation.
        """
        return self.translation.getY()

    def getRotation(self):
        """
        Returns the Rotation component of the transformation.
        """
        return self.rotation

    def inverse(self):
        """
        Invert the transformation.  This is useful for undoing a transformation.
        """
        # We are rotating the difference between the translations
        # using a clockwise rotation matrix. This transforms the global
        # delta into a local delta (relative to the initial pose).

        return Transform2d(
            self.getTranslation().unaryMinus().rotateBy(self.getRotation().unaryMinus()),
            self.getRotation().unaryMinus()
        )
    
    def __repr__(self):
        return "Transform2d({}, {})".format(self.translation, self.rotation)
    
    def __eq__(self, obj: object):
        """
        Checks equality between this Transform2d and another object.

        ``obj``: The other object
        """
        if isinstance(obj, Transform2d):
            return (
                obj.translation.equals(self.translation)
                and obj.rotation.equals(self.rotation))
        else:
            return False

class Twist2d:
    """
    A change in distance along arc since the last pose update. We can use ideas from differential
    calculus to create new Pose2ds from a Twist2d and vise versa.
 
    A Twist can be used to represent a difference between two poses.
    """
    def __init__(self, dx: float, dy: float, dtheta: float):
        """
        Constructs a Twist2d with the given values.
        
        ``dx``: Change in x direction relative to robot.
        
        ``dy``: Change in y direction relative to robot.
        
        ``dtheta``: Change in angle relative to robot.
        """
        self.dx = dx
        self.dy = dy
        self.dtheta = dtheta
    
    def __repr__(self):
        return "Twist2d(dX: {}, dy: {}, dTheta: {})".format(self.dx, self.dy, self.dtheta)
    
    def __eq__(self, obj: object):
        """
        Checks equality between this Twist2d and another object.

        ``obj``: The other object.
        """
        if isinstance(obj, Twist2d):
            return (abs(obj.dx - self.dx) < 1E-9
                and abs(obj.dy - self.dy) < 1E-9
                and abs(obj.dtheta - self.dtheta) < 1E-9)
        else:
            return False

class DifferentialDriveOdometry:
    """
    Class for differential drive odometry. Odometry allows you to track the robot's position on the
    field over the course of a match using readings from 2 encoders and a gyroscope.
    
    Teams can use odometry during the autonomous period for complex tasks like path following.
    Furthermore, odometry can be used for latency compensation when using computer-vision systems.
    
    It is important that you reset your encoders to zero before using this class. Any subsequent
    pose resets also require the encoders to be reset to zero.
    """
    def __init__(self, gyroAngle: Rotation2d, initialPoseMeters = Pose2d()):
        """
        Constructs a DifferentialDriveOdometry object.

        ``gyroAngle``: The angle reported by the gyroscope

        ``initialPoseMeters`` The starting position of the robot on the field.
        """
        self.poseMeters = initialPoseMeters
        self.gyroOffset = self.poseMeters.getRotation().minus(gyroAngle)
        self.previousAngle = initialPoseMeters.getRotation()
    
    def resetPosition(self, poseMeters: Pose2d, gyroAngle: Rotation2d):
        """
        Resets the robot's position on the field.
        
        You NEED to reset your encoders (to zero) when calling this method.
        
        The gyroscope angle does not need to be reset here on the user's robot code. The library
        automatically takes care of offsetting the gyro angle.
        
        ``poseMeters``: The position on the field that your robot is at.
        
        ``gyroAngle``: The angle reported by the gyroscope.
        """
        self.poseMeters = poseMeters
        self.previousAngle = poseMeters.getRotation()
        self.gyroOffset = poseMeters.getRotation().minus(gyroAngle)

        self.prevLeftDistance = 0.0
        self.prevRightDistance = 0.0
    
    def getPoseMeters(self):
        """
        Returns the position of the robot on the field (in meters).
        """
        return self.poseMeters
    
    def update(self, gyroAngle: Rotation2d, leftDistanceMeters, rightDistanceMeters):
        """
        Updates the robot position on the field using distance measurements from encoders. This method
        is more numerically accurate than using velocities to integrate the pose and is also
        advantageous for teams that are using lower CPR encoders.

        ``gyroAngle``: The angle reported by the gyroscope.

        ``leftDistanceMeters``: The distance traveled by the left encoder.

        ``rightDistanceMeters``: The distance traveled by the right encoder.
        """
        deltaLeftDistance = leftDistanceMeters - self.prevLeftDistance
        deltaRightDistance = rightDistanceMeters - self.prevRightDistance

        self.prevLeftDistance = leftDistanceMeters
        self.prevRightDistance = rightDistanceMeters

        averageDeltaDisatance = (deltaLeftDistance + deltaRightDistance) / 2.0
        angle = gyroAngle.plus(self.gyroOffset)

        newPose = self.poseMeters.exp(
            Twist2d(averageDeltaDisatance, 0.0, angle.minus(self.previousAngle).getRadians()))
        
        self.previousAngle = angle

        self.poseMeters = Pose2d(newPose.getTranslation, angle)
        return self.poseMeters

class ChassisSpeeds:
    """
    Represents the speed of a robot chassis. Although this struct contains similar members compared
    to a Twist2d, they do NOT represent the same thing. Whereas a Twist2d represents a change in pose
    w.r.t to the robot frame of reference, this ChassisSpeeds struct represents a velocity w.r.t to
    the robot frame of reference.
    
    A strictly non-holonomic drivetrain, such as a differential drive, should never have a dy
    component because it can never move sideways. Holonomic drivetrains such as swerve and mecanum
    will often have all three components.
    """
    def __init__(self, vxMetersPerSecond = 0.0, vyMetersPerSecond = 0.0, omegaRadiansPerSecond = 0.0):
        """
        Constructs a ChassisSpeeds object

        ``vxMetersPerSecond``: Foward velocity.
        ``vyMetersPerSecond``: Sideways velocity.
        ``omegaRadiansPerSecond``: Angular velocity.
        """
        self.vxMetersPerSecond = vxMetersPerSecond
        self.vyMetersPerSecond = vyMetersPerSecond
        self.omegaRadiansPerSecond = omegaRadiansPerSecond
    
    @classmethod
    def fromFieldRelativeSpeeds(cls,
            vxMetersPerSecond,
            vyMetersPerSecond,
            omegaRadiansPerSecond,
            robotAngle: Rotation2d):
        """
        Converts a user provided field-relative set of speeds into a robot-relative ChassisSpeeds object.
        
        ``vxMetersPerSecond``: The component of speed in the x direction relative to the field.
            Positive x is away from your alliance wall.
        
        ``vyMetersPerSecond``: The component of speed in the y direction relative to the field.
            Positive y is to your left when standing behind the alliance wall.
        
        ``omegaRadiansPerSecond``: The angular rate of the robot.
        
        ``robotAngle``: The angle of the robot as measured by a gyroscope. The robot's angle is
            considered to be zero when it is facing directly away from your alliance station wall.
            Remember that this should be CCW positive.

        Returns: ChassisSpeeds object representing the speeds in the robot's frame of reference.
        """
        return cls(
            vxMetersPerSecond * robotAngle.getCos() + vyMetersPerSecond * robotAngle.getSin(),
            -vxMetersPerSecond * robotAngle.getSin() + vyMetersPerSecond * robotAngle.getCos(),
            omegaRadiansPerSecond)
    
    def __repr__(self):
        return "ChassisSpeeds(Vx: {} m/s, Vy: {} m/s, Omega: {} rad/s)".format(
            self.vxMetersPerSecond, self.vyMetersPerSecond, self.omegaRadiansPerSecond)

class Trajectory:
    """
    Represents a time-parameterized trajectory. The trajectory contains of various States that
    represent the pose, curvature, time elapsed, velocity, and acceleration at that point.
    """
    
    states: 'list[State]'
    
    def __init__(self):
        """
        Constructs an empty trajectory
        """
        self.states = list()
        self.totalTimeSeconds = 0.0
    
    @classmethod
    def from_states(cls, states: 'list[State]'):
        """
        Constructs a trajectory from a vector of states.

        ``states``: A vector of states
        """
        traj = cls()
        traj.states = states
        traj.totalTimeSeconds = states[len(states) - 1].timeSeconds
        return traj
    
    def lerpVal(self, startValue, endValue, t):
        """
        Linearly interpolates between two values.

        ``StartValue``: The start value.
        ``endValue``: The end value.
        ``t``: The fraction for interpolation.

        Returns: The inerpolated value.
        """
        return startValue + (endValue - startValue) * t

    def lerpPose(self, startValue: Pose2d, endValue: Pose2d, t):
        """
        Linearly interpolates between two poses.

        ``StartValue``: The start pose.
        ``endValue``: The end pose.
        ``t``: The fraction for interpolation.

        Returns: The inerpolated pose.
        """
        return startValue.plus((endValue.minus(startValue)).times(t))
        
    
    def getInitialPose(self):
        """
        Returns the initial pose of the trajectory.
        """
        return self.sample(0).poseMeters
    
    def getTotalTimeSeconds(self):
        """
        Returns the overall duration of the trajectory.
        """
        return self.totalTimeSeconds
    
    def getStates(self):
        """
        Returns the states of the trajectory.
        """
        return self.states
    
    def sample(self, timeSeconds):
        """
        Sample the trajectory at a point in time.

        ``timeSeconds``: The point in time since the beginning of the trajectory to sample.

        Returns: The state at that point in time
        """
        if timeSeconds <= self.states[0].timeSeconds:
            return self.states[0]
        if timeSeconds >= self.totalTimeSeconds:
            return self.states[len(self.states) - 1]
        
        # To get the element that we want, we will use a binary search algorithm
        # instead of iterating over a for-loop. A binary search is O(std::log(n))
        # whereas searching using a loop is O(n).

        # This starts at 1 because we use the previous state later on for
        # interpolation.
        low = 1
        high = len(self.states) - 1

        while low is not high:
            mid = (low + high) / 2
            if self.states[mid].timeSeconds < timeSeconds:
                # This index and everything under it are less than the requested
                # timestamp. Therefore, we can discard them.
                low = mid + 1
            else:
                # t is at least as large as the element at this index. This means that
                # anything after it cannot be what we are looking for.
                high = mid
        # High and Low should be the same.

        # The sample's timestamp is now greater than or equal to the requested
        # timestamp. If it is greater, we need to interpolate between the
        # previous state and the current state to get the exact state that we
        # want.
        sample = self.states[low]
        prevSample = self.states[low - 1]

        # If the difference in states is negligible, then we are spot on!
        if abs(sample.timeSeconds - prevSample.timeSeconds) < 1E-9:
            return sample
        # Interpolate between the two states for the state that we want.
        return prevSample.interpolate(
            sample,
            (timeSeconds - prevSample.timeSeconds) / (sample.timeSeconds - prevSample.timeSeconds))
    
    def transformBy(self, transform: Transform2d):
        """
        Transforms all poses in the trajectory by the given transform. This is useful for converting a
        robot-relative trajectory into a field-relative trajectory. This works with respect to the
        first pose in the trajectory.

        ``transform`` The transform to transform the trajectory by.

        Returns: The transformed trajectory.
        """
        firstState = self.states[0]
        firstPose = firstState.poseMeters

        # Calculate the transformed first pose.
        newFirstPose = firstPose.plus(transform)
        newStates = list()

        newStates.append(
            self.State(
                firstState.timeSeconds,
                firstState.velocityMetersPerSecond,
                firstState.accelerationMetersPerSecondSq,
                newFirstPose,
                firstState.curvatureRadPerMeter
            )
        )

        for i in range(1, len(self.states)):
            state = self.states[i]
            newStates.append(
                state.timeSeconds,
                state.velocityMetersPerSecond,
                state.accelerationMetersPerSecondSq,
                newFirstPose.plus(state.poseMeters.minus(firstPose)),
                state.curvatureRadPerMeter)
        
        return Trajectory.from_states(newStates)
    
    def relativeTo(self, pose: Pose2d):
        """
        Transforms all poses in the trajectory so that they are relative to the given pose. This is
        useful for converting a field-relative trajectory into a robot-relative trajectory.

        ``pose``: The pose that is the origin of the coordinate frame that the current trajectory
        will be transformed into.

        Returns: The transformed trajectory.
        """
        return Trajectory.from_states(
            [self.State(
                state.timeSeconds,
                state.velocityMetersPerSecond,
                state.accelerationMetersPerSecondSq,
                state.poseMeters.relativeTo(pose),
                state.curvatureRadPerMeter) for state in self.states])
    
    def concatenate(self, other: "Trajectory"):
        """
        Concatenates another trajectory to the current trajectory. The user is responsible for making
        sure that the end pose of this trajectory and the start pose of the other trajectory match (if
        that is the desired behavior).

        ``other``: The trajectory to concatenate.
        
        Returns: The concatenated trajectory.
        """
        # If this is a default constructed trajectory with no states, then we can
        # simply return the rhs trajectory.
        if not self.states:
            return other
        
        # Deep copy the current states.
        states = [self.State(
            state.timeSeconds,
            state.velocityMetersPerSecond,
            state.accelerationMetersPerSecondSq,
            state.poseMeters,
            state.curvatureRadPerMeter) for state in self.states]
        
        # Here we omit the first state of the other trajectory because we don't want
        # two time points with different states. Sample() will automatically
        # interpolate between the end of this trajectory and the second state of the
        # other trajectory.
        for i in range(1, len(other.getStates())):
            s = other.getStates()[i]
            states.add(
                self.State(
                    s.timeSeconds + self.totalTimeSeconds,
                    s.velocityMetersPerSecond,
                    s.accelerationMetersPerSecondSq,
                    s.poseMeters,
                    s.curvatureRadPerMeter))
        return Trajectory.from_states(states)
    
    class State:
        """
        Represents a time-parameterized trajectory. The trajectory contains of various States that
        represent the pose, curvature, time elapsed, velocity, and acceleration at that point.
        """
        def __init__(self,
        timeSeconds = 0.0,
        velocityMetersPerSecond = 0.0,
        accelerationMetersPerSecondSq = 0.0,
        poseMeters = Pose2d(),
        curvatureRadPerMeter = 0.0):
            """
            Constructs a State with the specified parameters.

            ``timeSeconds``: The time elapsed since the beginning of the trajectory.            
            ``velocityMetersPerSecond``: The speed at that point of the trajectory.
            ``accelerationMetersPerSecondSq``: The acceleration at that point of the trajectory.
            ``poseMeters``: The pose at that point of the trajectory.
            ``curvatureRadPerMeter``: The curvature at that point of the trajectory.
            """
            self.timeSeconds = timeSeconds
            self.velocityMetersPerSecond = velocityMetersPerSecond
            self.accelerationMetersPerSecondSq = accelerationMetersPerSecondSq
            self.poseMeters = poseMeters
            self.curvatureRadPerMeter = curvatureRadPerMeter
        
        def interpolate(self, endValue: 'Trajectory.State', i):
            """
            Interpolates between two States.

            ``endValue``: The end value for the interpolation.

            ``i``: The interpolant (fraction).

            Returns: The interpolated state.
            """
            # Find the new t value
            newT = Trajectory.lerpVal(self.timeSeconds, endValue.timeSeconds, i)

            # Find the delta time between the current state and the interpolated state.
            deltaT = newT - self.timeSeconds

            # If delta time is negative, flip the order of interpolation.
            if deltaT < 0:
                return endValue.interpolate(self, 1 - i)
            
            reversing = (self.velocityMetersPerSecond < 0
                or (abs(self.velocityMetersPerSecond) < 1E-9 and self.accelerationMetersPerSecondSq < 0))
            
            # Calculate the new velocity
            # v_f = v_0 + at
            newV = self.velocityMetersPerSecond + (self.accelerationMetersPerSecondSq * deltaT)

            # Calculate the change in position
            # delta_s = v_0 t + 0.5 at^2
            newS = (self.velocityMetersPerSecond * deltaT 
                + 0.5 * self.accelerationMetersPerSecondSq * deltaT ** 2
                * (-1.0 if reversing else 1.0))
            
            # Return the new state. To find the new position for the new state, we need
            # to interpolate between the two endpoint poses. The fraction for
            # interpolation is the change in position (delta s) divided by the total
            # distance between the two endpoints.
            interpolationFrac = newS / endValue.poseMeters.getTranslation().getDistance(self.poseMeters.getTranslation())

            return Trajectory.State(
                newT,
                newV,
                self.accelerationMetersPerSecondSq,
                Trajectory.lerpPose(self.poseMeters, endValue.poseMeters, interpolationFrac),
                Trajectory.lerpVal(self.curvatureRadPerMeter, endValue.curvatureRadPerMeter, interpolationFrac))
        def __repr__(self):
            return "State(Sec: {}, Vel m/s: {}, Accel m/s/s: {}, Pose: {}, Curvature: {})".format(
               self.timeSeconds,
               self.velocityMetersPerSecond,
               self.accelerationMetersPerSecondSq,
               self.poseMeters,
               self.curvatureRadPerMeter)
        
        def __eq__(self, obj: object):
            if self == obj:
                return True
            if not isinstance(obj, Trajectory.State):
                return False
            obj: Trajectory.State
            return (obj.timeSeconds == self.timeSeconds and
                obj.velocityMetersPerSecond == self.velocityMetersPerSecond and
                obj.accelerationMetersPerSecondSq == self.accelerationMetersPerSecondSq and
                obj.curvatureRadPerMeter == self.curvatureRadPerMeter and
                obj.poseMeters == self.poseMeters)
    def __repr__(self):
        stateList = ", \n".join(self.states)
        return "Trajectory - Seconds: {}, States:\n{}".format(self.totalTimeSeconds, stateList)
    
    def __eq__(self, obj: object):
        return isinstance(obj, Trajectory) and self.states == obj.getStates()

class RamseteController:
    """
    Ramsete is a nonlinear time-varying feedback controller for unicycle models that drives the model
    to a desired pose along a two-dimensional trajectory. Why would we need a nonlinear control law
    in addition to the linear ones we have used so far like PID? If we use the original approach with
    PID controllers for left and right position and velocity states, the controllers only deal with
    the local pose. If the robot deviates from the path, there is no way for the controllers to
    correct and the robot may not reach the desired global pose. This is due to multiple endpoints
    existing for the robot which have the same encoder path arc lengths.
    
    Instead of using wheel path arc lengths (which are in the robot's local coordinate frame),
    nonlinear controllers like pure pursuit and Ramsete use global pose. The controller uses this
    extra information to guide a linear reference tracker like the PID controllers back in by
    adjusting the references of the PID controllers.
    
    The paper "Control of Wheeled Mobile Robots: An Experimental Overview" describes a nonlinear
    controller for a wheeled vehicle with unicycle-like kinematics; a global pose consisting of x, y,
    and theta; and a desired pose consisting of x_d, y_d, and theta_d. We call it Ramsete because
    that's the acronym for the title of the book it came from in Italian ("Robotica Articolata e
    Mobile per i SErvizi e le TEcnologie").
    
    See https://file.tavsys.net/control/controls-engineering-in-frc.pdf section on Ramsete unicycle
    controller for a derivation and analysis.
    """
    poseError = Pose2d.empty()
    poseTolerance = Pose2d.empty()
    enabled = True

    def __init__(self, b = 2.0, zeta = 7.0):
        """
        Construct a Ramsete unicycle controller.

        ``b``: Tuning parameter (b &gt; 0 rad²/m²) for which larger values make convergence more
        aggressive like a proportional term.
        
        ``zeta``: Tuning parameter (0 rad⁻¹ &lt; zeta &lt; 1 rad⁻¹) for which larger values provide
        more damping in response.
        """
        self.b = b
        self.zeta = zeta
    
    def atRefrence(self):
        """
        Returns true if the pose error is within tolerance of the reference.
        """
        eTranslate = self.poseError.getTranslation()
        eRotate = self.poseError.getRotation()
        tolTranslate = self.poseTolerance.getTranslation()
        tolRotate = self.poseTolerance.getRotation()
        return (abs(eTranslate.getX()) < tolTranslate.getX()
            and abs(eTranslate.getY()) < tolTranslate.getY()
            and abs(eRotate.getRadians()) < tolRotate.getRadians())
    
    def setTolerance(self, poseTolerance: Pose2d):
        """
        Sets the pose error which is considered tolerable for use with atReference().

        ``poseTolerance``: Pose error which is tolerable.
        """
        self.poseTolerance = poseTolerance
    
    def calculate(self, currentPose: Pose2d, poseRef: Pose2d, linearVelocityRefMeters, angularVelocityRefRadiansPerSecond):
        """
        Returns the next output of the Ramsete controller.

        The reference pose, linear velocity, and angular velocity should come from a drivetrain
        trajectory.

        ``currentPose``: The current pose.
        ``poseRef``: The desired pose.
        ``linearVelocityRefMeters``: The desired linear velocity in meters per second.
        ``angularVelocityRefRadiansPerSecond``: The desired angular velocity in radians per second.

        Returns: The next controller output.
        """
        if not self.enabled:
            return ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond)

        self.poseError = poseRef.relativeTo(currentPose)

        # Aliases for equation readability
        eX = self.poseError.getX()
        eY = self.poseError.getY()
        eTheta = self.poseError.getRotation().getRadians()
        vRef = linearVelocityRefMeters
        omegaRef = angularVelocityRefRadiansPerSecond

        k = 2.0 * self.zeta * math.sqrt((omegaRef ** 2) + self.b * (vRef ** 2))

        return ChassisSpeeds(
            vRef * self.poseError.getRotation().getCos() + k * eX,
            0.0,
            omegaRef + k * eTheta + self.b * vRef * RamseteController.sinc(eTheta) * eY)
    
    def calculateFromState(self, currentPose: Pose2d, desiredState: Trajectory.State):
        """
        Returns the next output of the Ramsete controller.

        The reference pose, linear velocity, and angular velocity should come from a drivetrain
        trajectory.

        ``currentPose``: The current pose.
        ``desiredState``: The desired pose, linear velocity, and angular velocity from a trajectory.

        Returns: The next controller output.
        """
        return self.calculate(currentPose,
            desiredState.poseMeters,
            desiredState.velocityMetersPerSecond,
            desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter)
        
    def setEnabled(self, enabled: bool):
        """
        Enables and disables the controller for troubleshooting purposes.

        ``enabled``: If the controller is enabled or not.
        """
        self.enabled = enabled
    
    @classmethod
    def sinc(cls, x):
        """
        Returns sin(x) / x.
        """
        if abs(x) < 1e-9:
            return 1.0 - 1.0 / 6.0 * x * x
        else:
            return math.sin(x) / x

class SimpleMotorFeedforward:
    """
    A helper class that computes feedforward outputs for a simple permanent-magnet DC motor.
    """
    def __init__(self, ks, kv, ka = 0.0):
        """
        Creates a new SimpleMotorFeedforward with the specified gains. Units of the gain values will
        dictate units of the computed feedforward.


        ``ks``: The static gain.
        ``kv``: The velocity gain.
        ``ka``: The acceleration gain.
        """
        self.ks = ks
        self.kv = kv
        self.ka = ka
    
    def calculate(self, velocity, acceleration):
        """
        Calculates the feedforward from the gains and setpoints.

        ``velocity``: The velocity setpoint.

        ``acceleration``: The acceleration setpoint.

        Returns: The computed feedforward.
        """
        return math.copysign(self.ks, velocity) + self.kv * velocity + self.ka * acceleration
    
    def maxAchievableVelocity(self, maxVoltage, acceleration):
        """
        Calculates the maximum achievable velocity given a maximum voltage supply and an acceleration.
        Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
        simultaneously achievable - enter the acceleration constraint, and this will give you a
        simultaneously-achievable velocity constraint.

        ``maxVoltage``: The maximum voltage that can be supplied to the motor.

        ``acceleration``: The acceleration of the motor.

        Returns: The maximum possible velocity at the given acceleration.
        """
        return (maxVoltage - self.ks - acceleration * self.ka) / self.kv
    
    def minAchievableVelocity(self, maxVoltage, acceleration):
        """
        Calculates the minimum achievable velocity given a maximum voltage supply and an acceleration.
        Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
        simultaneously achievable - enter the acceleration constraint, and this will give you a
        simultaneously-achievable velocity constraint.

        ``maxVoltage``: The maximum voltage that can be supplied to the motor.

        ``acceleration``: The acceleration of the motor.

        Returns: The minimum possible velocity at the given acceleration.
        """
        # Assume min velocity is negative, ks flips sign
        return (-maxVoltage + self.ks - acceleration * self.ka) / self.kv
    
    def maxAchievableAcceleration(self, maxVoltage, velocity):
        """
        Calculates the maximum achievable acceleration given a maximum voltage supply and an velocity.
        Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
        simultaneously achievable - enter the velocity constraint, and this will give you a
        simultaneously-achievable acceleration constraint.

        ``maxVoltage``: The maximum voltage that can be supplied to the motor.

        ``velocity``: The velocity of the motor.

        Returns: The maximum possible acceleration at the given velocity.
        """
        return (maxVoltage - math.copysign(self.ks, velocity) - velocity * self.kv) / self.ka
    
    def minAchievableAcceleration(self, maxVoltage, velocity):
        """
        Calculates the minimum achievable acceleration given a maximum voltage supply and an velocity.
        Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
        simultaneously achievable - enter the velocity constraint, and this will give you a
        simultaneously-achievable acceleration constraint.

        ``maxVoltage``: The maximum voltage that can be supplied to the motor.

        ``velocity``: The velocity of the motor.

        Returns: The minimum possible acceleration at the given velocity.
        """
        return self.maxAchievableAcceleration(-maxVoltage, velocity)

class DifferentialDriveWheelSpeeds:
    """
    Represents the wheel speeds for a differential drive drivetrain.
    """
    def __init__(self, leftMetersPerSecond = 0.0, rightMetersPerSecond = 0.0):
        """
        Constructs a DifferentialDriveWheelSpeeds
        """
        self.leftMetersPerSecond = leftMetersPerSecond
        self.rightMetersPerSecond = rightMetersPerSecond
    
    def desaturate(self, attainableMaxSpeedMetersPerSecond):
        """
        Renormalizes the wheel speeds if any either side is above the specified maximum.

        Sometimes, after inverse kinematics, the requested speed from one or more wheels may be
        above the max attainable speed for the driving motor on that wheel. To fix this issue, one can
        reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
        absolute threshold, while maintaining the ratio of speeds between wheels.

        ``attainableMaxSpeedMetersPerSecond``: The absolute max speed that a wheel can reach.
        """
        realMaxSpeed = max([abs(self.leftMetersPerSecond), abs(self.rightMetersPerSecond)])

        if realMaxSpeed > attainableMaxSpeedMetersPerSecond:
            self.leftMetersPerSecond = self.leftMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond
            self.rightMetersPerSecond = self.rightMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond
    
    def __repr__(self):
        return "DifferentialDriveWheelSpeeds(Left: {} m/s, Right: {} m/s)".format(self.leftMetersPerSecond, self.rightMetersPerSecond)

class DifferentialDriveKinematics:
    """
    Helper class that converts a chassis velocity (dx and dtheta components) to left and right wheel
    velocities for a differential drive.
    
    Inverse kinematics converts a desired chassis speed into left and right velocity components
    whereas forward kinematics converts left and right component velocities into a linear and angular
    chassis speed.
    """
    def __init__(self, trackWidthMeters):
        """
         Constructs a differential drive kinematics object.

        ``trackWidthMeters``: The track width of the drivetrain. Theoretically, this is the distance
        between the left wheels and right wheels. However, the empirical value may be larger than
        the physical measured value due to scrubbing effects.
        """
        self.trackWidthMeters = trackWidthMeters
    
    def toChassisSpeeds(self, wheelSpeeds: DifferentialDriveWheelSpeeds):
        """
        Returns a chassis speed from left and right component velocities using forward kinematics.

        ``wheelSpeeds``: The left and right velocities.

        Returns: The chassis speed.
        """
        return ChassisSpeeds(
            (wheelSpeeds.leftMetersPerSecond + wheelSpeeds.rightMetersPerSecond) / 2,
            0,
            (wheelSpeeds.rightMetersPerSecond - wheelSpeeds.leftMetersPerSecond) / self.trackWidthMeters)
    
    def toWheelSpeeds(self, chassisSpeeds: ChassisSpeeds):
        """
        Returns left and right component velocities from a chassis speed using inverse kinematics.

        ``chassisSpeeds``: The linear and angular (dx and dtheta) components that represent the chassis' speed.

        Returns: The left and right velocities.
        """
        return DifferentialDriveWheelSpeeds(
            chassisSpeeds.vxMetersPerSecond - self.trackWidthMeters / 2 * chassisSpeeds.omegaRadiansPerSecond,
            chassisSpeeds.vxMetersPerSecond + self.trackWidthMeters / 2 * chassisSpeeds.omegaRadiansPerSecond)

class MathUtil:
    @classmethod
    def inputModulus(cls, input, minimumInput, maximumInput):
        """
        Returns modulus of input.
        
        ``input`` Input value to wrap.
        
        ``minimumInput`` The minimum value expected from the input.
        
        ``maximumInput`` The maximum value expected from the input.
        
        Returns: The wrapped value.
        """
        modulus = maximumInput - minimumInput
        # Wrap input if it's above the maximum input
        numMax = int((input - minimumInput) / modulus)
        input -= numMax * modulus

        # Wrap input if it's below the minimum input
        numMin = int((input - maximumInput) / modulus)
        input -= numMin * modulus

        return input
    
    @classmethod
    def clamp(cls, value, low, high):
        """
        Returns value clamped between low and high boundaries.

        ``value``: Value to clamp.
        
        ``low``: The lower boundary to which to clamp value.
        
        ``high``: The higher boundary to which to clamp value.
        
        Returns : The clamped value.
        """
        return max(low, min(value, high))

class PIDController:
    """
    Implements a PID control loop.
    """
    maximumIntegral = 1.0
    minimumIntegral = -1.0
    continuous: bool
    positionError: float
    velocityError: float
    prevError: float
    totalError: float
    positionTolerance = 0.05
    velocityTolerance = math.inf
    setpoint: float
    measurement: float

    def __init__(self, kp, ki, kd, period = 0.02):
        """
        Allocates a PIDController with the given constants for kp, ki, and kd.

        
        ``kp``: The proportional coefficient.
        
        ``ki``: The integral coefficient.
        
        ``kd``: The derivative coefficient.
        ``period``: The period between controller updates in seconds. Must be non-zero and positive.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        if period <= 0:
            raise ValueError("Controller period must be a non-zero positive number!")
        self.period = period
    
    def setPID(self, kp, ki, kd):
        """
        Sets the PID Controller gain parameters.
        
        Set the proportional, integral, and differential coefficients.

        ``kp``: The proportional coefficient.
        
        ``ki``: The integral coefficient.
        
        ``kd``: The derivative coefficient.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
    
    def setP(self, kp):
        """
        Sets the Proportional coefficient of the PID controller gain.
        
        ``kp``: proportional coefficient.
        """
        self.kp = kp
    
    def setI(self, ki):
        """
        Sets the Integral coefficient of the PID controller gain.
        
        ``ki``: integral coefficient.
        """
        self.ki = ki
    
    def setD(self, kd):
        """
        Sets the Derivative coefficient of the PID controller gain.
        
        ``kd``: derivative coefficient.
        """
        self.kd = kd
    
    def getP(self):
        """
        Get the Proportional coefficient.
        """
        return self.kp
    
    def getI(self):
        """
        Get the Integral coefficient.
        """
        return self.ki

    def getD(self):
        """
        Get the Derivative coefficient.
        """
        return self.kd
    
    def getPeriod(self):
        """
        Returns the period of this controller.
        """
        return self.period

    def setSetpoint(self, setpoint):
        """
        Sets the setpoint for the PID controller gain.
        
        ``setpoint``: The desired setpoint.
        """
        self.setpoint = setpoint
    
    def getSetpoint(self):
        """
        Returns the current setpoint of the PIDController.
        """
        return self.setpoint
    
    def atSetpoint(self):
        """
        Returns true if the error is within the tolerance of the setpoint.

        This will return false until at least one input value has been computed.
        
        Returns: Whether the error is within the acceptable bounds.
        """
        if self.continuous:
            errorBound = (self.maximumInput - self.minimumInput) / 2.0
            positionError = MathUtil.inputModulus(self.setpoint - self.measurement, -errorBound, errorBound)
        else:
            positionError = (self.setpoint - self.measurement)
        
        velocityError = (positionError - self.prevError) / self.period

        return (abs(positionError) < self.positionTolerance
            and abs(velocityError) < self.velocityTolerance)
    
    def enableContinuousInput(self, minimumInput, maximumInput):
        """
        Enables continuous input.

        Rather then using the max and min input range as constraints, it considers them to be the
        same point and automatically calculates the shortest route to the setpoint.
        
        ``minimumInput``: The minimum value expected from the input.
        
        ``maximumInput``: The maximum value expected from the input.
        """
        self.continuous = True
        self.minimumInput = minimumInput
        self.maximumInput = maximumInput
    
    def disableContinuousInput(self):
        """
        Disables continuous input.
        """
        self.continuous = False
    
    def isContinuousInputEnabled(self):
        """
        Returns true if continuous input is enabled.
        """
        return self.continuous
    
    def setIntegratorRange(self, minimumIntegral, maximumIntegral):
        """
        Sets the minimum and maximum values for the integrator.
        When the cap is reached, the integrator value is added to the controller output rather than
        the integrator value times the integral gain.
        
        ``minimumIntegral``: The minimum value of the integrator.
        
        ``maximumIntegral``: The maximum value of the integrator.
        """
        self.minimumIntegral = minimumIntegral
        self.maximumIntegral = maximumIntegral
    
    def setTolerance(self, positionTolerance, velocityTolerance = math.inf):
        """
        Sets the error which is considered tolerable for use with atSetpoint().
        
        ``positionTolerance``: Position error which is tolerable.
        
        ``velocityTolerance``: Velocity error which is tolerable.
        """
        self.positionTolerance = positionTolerance
        self.velocityTolerance = velocityTolerance
    
    def getPositionError(self):
        """
        Returns the difference between the setpoint and the measurement.
        """
        return self.positionError

    def getVelocityError(self):
        """
        Returns the velocity error.
        """
        return self.velocityError
    
    def calculate(self, measurement, setpoint = None):
        """
        Returns the next output of the PID controller.
        
        ``measurement``: The current measurement of the process variable.
        
        ``setpoint``: The new setpoint of the controller.
        """
        if setpoint is not None:
            self.setSetpoint(setpoint)
        self.measurement = measurement
        self.prevError = self.positionError

        if self.continuous:
            errorBound = (self.maximumInput - self.minimumInput) / 2.0
            self.positionError = MathUtil.inputModulus(self.setpoint - self.measurement, -errorBound, errorBound)
        else:
            self.positionError = self.setpoint - measurement
        
        self.velocityError = (self.positionError - self.prevError) / self.period

        if self.ki is not 0:
            self.totalError = MathUtil.clamp(
                self.totalError + self.positionError * self.period,
                self.minimumIntegral / self.ki,
                self.maximumIntegral / self.ki)
        
        return self.kp * self.positionError + self.ki * self.totalError + self.kd * self.velocityError
    
    def reset(self):
        """
        Resets the previous error and the integral term.
        """
        self.prevError = 0.0
        self.totalError = 0.0

class Timer:
    """
    A timer class.
    """
    startTime: float
    accumulatedTime: float
    running: bool

    @classmethod
    def getTime(cls):
        """
        Return the system clock time in seconds.
        """
        return time.time()
    
    @classmethod
    def delay(cls, seconds):
        """
        ``seconds``: Length of time to pause
        """
        time.sleep(seconds)
    
    def __init__(self):
        self.reset()
    
    def getMsClock(self):
        return time.time() * 1000
    
    def get(self):
        """
        Get the current time from the timer. If the clock is running it is derived from the current
        system clock the start time stored in the timer class. If the clock is not running, then return
        the time when it was last stopped.
        """
        if self.running:
            return self.accumulatedTime + (self.getMsClock() - self.startTime) / 1000
        else:
            return self.accumulatedTime
    
    def reset(self):
        """
        Reset the timer by setting the time to 0.
        
        Make the timer startTime the current time so new requests will be relative now.
        """
        self.accumulatedTime = 0
        self.startTime = self.getMsClock()
    
    def start(self):
        """
        Start the timer running. Just set the running flag to true indicating that all time requests
        should be relative to the system clock. Note that this method is a no-op if the timer is
        already running.
        """
        if not self.running:
            self.startTime = self.getMsClock()
            self.running = True
    
    def stop(self):
        """
        Stop the timer. This computes the time as of now and clears the running flag, causing all
        subsequent time requests to be read from the accumulated time rather than looking at the system
        clock.
        """
        self.accumulatedTime = self.get()
        self.running = False
    
    def hasElapsed(self, seconds):
        """
        Check if the period specified has passed.
        """
        return self.get() >= seconds
    
    def advanceIfElapsed(self, seconds):
        """
        Check if the period specified has passed and if it has, advance the start time by that period.
        This is useful to decide if it's time to do periodic work without drifting later by the time it
        took to get around to checking.
        """
        if self.get() >= seconds:
            # Advance the start time by the period.
            # Don't set it to the current time... we want to avoid drift.
            self.startTime += seconds * 1000
            return True
        else:
            return False