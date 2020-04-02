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

/**
 * Velocity drive controls drive base motors in tank configuration based on
 * desired velocity, specified as 0-1 input times the max velocity of the
 * robot summed with feed forward computed from the same 0-1 input and the
 * ks and kv values determined by the Characterization process. Motors are
 * controlled by setting voltage as determined by the feed forward and PID
 * controllers trying to match desired velocity to measured velocity.
 * You must measure max velocity, max angular velocity, ks and kv for each
 * robot that uses this class to get correct results. Use the FIRST robot
 * characterization tool for this.
 */
public class VelocityDrive extends MotorSafety
{
	public final double maxSpeed; 			// meters per second.
	public final double maxAngularSpeed;	// radians per second.
	public final double trackWidth; 		// meters.
	public final double p, i, d, ks, kv;
	
	private final SpeedController	leftController, rightController;
	
	private final SRXMagneticEncoderRelative	leftEncoder, rightEncoder;

	private final PIDController leftPIDController;
	private final PIDController rightPIDController;
	
	private final DifferentialDriveKinematics kinematics;
	
	private final SimpleMotorFeedforward feedforward;
	
	/**
	 * Constructs a Velocity drive object.
	 * @param leftController	Left side motor controller;
	 * @param rightController	Right side motor controller;
	 * @param leftEncoder		Left side encoder.
	 * @param rightEncoder		Right side encoder.
	 * @param trackWidth		Track width of robot in inches.
	 * @param maxSpeed			Max speed of robot in m/s.
	 * @param maxAngularSpeed	Max angular speed of robot in radians/s.
	 * @param p					P term for PID controllers. 1 is a good default.
	 * @param i					I term for PID controllers. 0 is a good default.
	 * @param d					D term for PID controllers. 0 is a good default.
	 * @param ks				Feed forward static gain (volts). 1 is a good default.
	 * @param kv				Feed forward velocity gain (volts * seconds / distance). 1 is a good default.
	 */
	public VelocityDrive(SpeedController leftController, 
						 SpeedController rightController,
						 SRXMagneticEncoderRelative leftEncoder, 
						 SRXMagneticEncoderRelative rightEncoder,
						 double trackWidth, double maxSpeed, double maxAngularSpeed,
						 double p, double i, double d, double ks, double kv) 
	{
		this.leftController = leftController;
		this.rightController = rightController;
		this.leftEncoder = leftEncoder;
		this.rightEncoder = rightEncoder;
		this.trackWidth = Util.inchesToMeters(trackWidth);
		this.maxSpeed = maxSpeed;
		this.maxAngularSpeed = maxAngularSpeed;
		this.p = p;
		this.i = i;
		this.d = d;
		this.ks = ks;
		this.kv = kv;
		
		leftPIDController = new PIDController(p, i, d);
		rightPIDController = new PIDController(p, i, d);
		
		kinematics = new DifferentialDriveKinematics(trackWidth);
		
		feedforward = new SimpleMotorFeedforward(ks, kv);
	}
	
	/**
	 * Sets the desired wheel speeds by using current speeds and PID controllers
	 * to calculate the motor voltage to sync actual to desired.
	 *
	 * @param speeds The desired wheel speeds.
	 */
	private void setSpeeds(DifferentialDriveWheelSpeeds speeds) 
	{
	    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
	    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
	    
	    final double leftSpeed = leftEncoder.getVelocity(PIDRateType.velocityMPS);
	    final double rightSpeed = rightEncoder.getVelocity(PIDRateType.velocityMPS);
	
	    final double leftOutput = leftPIDController.calculate(leftSpeed, speeds.leftMetersPerSecond);
	    final double rightOutput = rightPIDController.calculate(rightSpeed, speeds.rightMetersPerSecond);
	    
	    leftController.setVoltage(leftOutput + leftFeedforward);
	    rightController.setVoltage(rightOutput + rightFeedforward);
	    
	    Util.consoleLog("lt=%.3f la=%.3f lo=%.3f lf=%.3f - rt=%.3f ra=%.3f ro=%.3f rf=%.3f",
	    		speeds.leftMetersPerSecond, leftSpeed, leftOutput, leftFeedforward,
	    		speeds.rightMetersPerSecond, rightSpeed, rightOutput, rightFeedforward);
	    
	    //feed();
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
	 * @param rot		Angular velocity as a % of max angular speed. Ranges -1 to +1, + is right.
	*/
	public void arcadeDrive(double fbSpeed, double rot)
	{
		// Note we invert rot because we like to work + as clockwise, but ChassisSpeeds expects + to be counter
		// clockwise. Also note that the Y axis speed is zero since tank robot can't move sideways.
		DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(fbSpeed * maxSpeed, 
				0.0, -rot * maxAngularSpeed));
		
	    setSpeeds(wheelSpeeds);
	}
	
	/**
	 * Not Implemented.
	 * @param fbSpeed
	 * @param curve
	 * @param quickTurn
	 */
	public void curvatureDrive(double fbSpeed, double curve, boolean quickTurn)
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
