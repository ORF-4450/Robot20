package Team4450.Robot20;

import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.SRXMagneticEncoderRelative.PIDRateType;
import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class VelocityDrive extends MotorSafety
{
	public final double maxSpeed; 			// meters per second.
	public final double maxAngularSpeed;	// radians per second.
	public final double trackWidth; 		// meters.
	
	private final SpeedController	leftController, rightController;
	
	private final SRXMagneticEncoderRelative	leftEncoder, rightEncoder;

	private final PIDController leftPIDController = new PIDController(1, 0, 0);
	private final PIDController rightPIDController = new PIDController(1, 0, 0);
	
	private final DifferentialDriveKinematics kinematics;
	
	// Gains are for example purposes only - must be determined for your own robot!
	private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 1);
	
	/**
	 * Constructs a Velocity drive object.
	 * @param leftController	Left side motor controller;
	 * @param rightController	Right side motor controller;
	 * @param leftEncoder		Left side encoder.
	 * @param rightEncoder		Right side encoder.
	 * @param trackWidth		Track width of robot in inches.
	 * @param maxSpeed			Max speed of robot in m/s.
	 * @param maxAngularSpeed	Max angular speed of robot in rad/s.
	 */
	public VelocityDrive(SpeedController leftController, 
						 SpeedController rightController,
						 SRXMagneticEncoderRelative leftEncoder, 
						 SRXMagneticEncoderRelative rightEncoder,
						 double trackWidth, double maxSpeed, double maxAngularSpeed) 
	{
		this.leftController = leftController;
		this.rightController = rightController;
		this.leftEncoder = leftEncoder;
		this.rightEncoder = rightEncoder;
		this.trackWidth = Util.inchesToMeters(trackWidth);
		this.maxSpeed = maxSpeed;
		this.maxAngularSpeed = maxAngularSpeed;
		
		kinematics = new DifferentialDriveKinematics(trackWidth);
	}
	
	/**
	 * Sets the desired wheel speeds.
	 *
	 * @param speeds The desired wheel speeds.
	 */
	private void setSpeeds(DifferentialDriveWheelSpeeds speeds) 
	{
	    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
	    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
	    
	    final double leftSpeed = leftEncoder.getRate(PIDRateType.velocityMPS);
	    final double rightSpeed = rightEncoder.getRate(PIDRateType.velocityMPS);
	
	    final double leftOutput = leftPIDController.calculate(leftSpeed, speeds.leftMetersPerSecond);
	    final double rightOutput = rightPIDController.calculate(rightSpeed, speeds.rightMetersPerSecond);
	    
	    leftController.setVoltage(leftOutput + leftFeedforward);
	    rightController.setVoltage(rightOutput + rightFeedforward);
	    
	    feed();
	}
	
	/**
	 * Drives the robot with the given linear velocities for left and right motors. This is tank style.
	 * @param leftSpeed		Linear velocity as a % of max speed. Ranges -1 to +1, + is forward unit is m/s.
	 * @param rightSpeed	Linear velocity as a % of max speed. Ranges -1 to +1, + is forward unit is m/s.
	 */
	public void tankDrive(double leftSpeed, double rightSpeed)
	{
		setSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed * maxSpeed, rightSpeed * maxSpeed));
	}
	
	/**
	 * Drives the robot with the given linear velocities for left and right motors. This is tank style.
	 * @param leftSpeed		Linear velocity as a % of max speed. Ranges -1 to +1, + is forward unit is m/s.
	 * @param rightSpeed	Linear velocity as a % of max speed. Ranges -1 to +1, + is forward unit is m/s.
	 * @param squaredInputs True to scale the input speeds with the Util.squareInput() function to make
	 * 						inputs more gentle.
	 */
	public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs)
	{
		if (squaredInputs)
			tankDrive(Util.squareInput(leftSpeed), Util.squareInput(rightSpeed));
		else
			tankDrive(leftSpeed, rightSpeed);
	}
	
	/**
	 * Drives the robot with the given linear velocity and angular velocity.
	 * This is arcade style.
	 * @param fbSpeed 	Linear velocity as a % of max speed (forward/backward). Ranges -1 to +1, + is forward.
	 * Unit is m/s.
	 * @param rot		Angular velocity as a % if max angular speed. Ranges -1 to +1, + is right. Unit is rad/s.
	*/
	public void arcadeDrive(double fbSpeed, double rot)
	{
		// Note we invert rot because we like to work + as clockwise, but ChassisSpeeds expects + to be counter
		//  clockwise.
		DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(fbSpeed * maxSpeed, 
				0.0, -rot * maxAngularSpeed));
		
	    setSpeeds(wheelSpeeds);
	}
	
	/**
	 * Not Implemented.
	 * @param power
	 * @param curve
	 * @param quickTurn
	 */
	public void curvatureDrive(double xSpeed, double zRotation, boolean quickTurn)
	{
		
	}
	
	/**
	 * Stop motors.
	 */
	public void stopMotor()
	{
	    leftController.stopMotor();
	    rightController.stopMotor();
	    feed();
	}

	@Override
	public String getDescription()
	{
		return "";
	}
}
