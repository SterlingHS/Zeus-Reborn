/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6651.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
// import com.kauailabs.navx.frc.NavX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	// SmartDashBoard Init
	SmartDashboard SmartDashBoard;

	// UltraSound Init
	AnalogInput ai;

	// Encoders Init
	Encoder enc1, enc2, enc3, enc4;

	// Port init for Relays - RGB LEDs
	Relay LEDRed, LEDGreen, LEDBlue;
	int RedPort = 0;
	int GreenPort = 2;
	int BluePort = 3;
	
	// Mecanum Drive Init and motor controllers
	public static MecanumDrive DTMec;
	WPI_TalonSRX talon10;
	WPI_TalonSRX talon11;
	WPI_TalonSRX talon12;
	WPI_TalonSRX talon13;
	
	// Gyro Init
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	double angle, field_orientation;
	int resetGyroId = 2;

	// NavX Init
	AHRS NavX;
	
	// Joystick Init
	Joystick PS4 = new Joystick(0);
	int butterflyButtonId = 1;
	
	// MaxPower - Use less than 1 to slow down the robot
	double MAXPOWER=1;
	double AM_MAXPOWER=0.5;
	
	// Pneumatic Init
	Compressor c;
	DoubleSolenoid butterflySolenoid;
	boolean changeOfState = true;
	DoubleSolenoid.Value UP=DoubleSolenoid.Value.kForward, DOWN=DoubleSolenoid.Value.kReverse;
	DoubleSolenoid.Value butterflyState = DOWN;

	// AUTONOMOUS Mode variables
	int stage;
	int LEFT=1, RIGHT=-1;

	@Override
	public void robotInit() {
		// UltraSount Init in port 2
		ai = new AnalogInput(2);

		// Right side Controllers
		talon10 = new WPI_TalonSRX(10);
		talon12 = new WPI_TalonSRX(12);

		// Left side Controllers
		talon11 = new WPI_TalonSRX(11);
		talon13 = new WPI_TalonSRX(13);

		// DriveTrain Init
		DTMec = new MecanumDrive(talon10, talon11, talon12, talon13);
		
		// Pneumatic Init
		c = new Compressor(0);
		c.setClosedLoopControl(true);  // Start compressor control
		butterflySolenoid = new DoubleSolenoid(0, 1);
		butterflySolenoid.set(butterflyState);

		// LED Strip Init
		LEDRed = new Relay(RedPort);
		LEDRed.set(Relay.Value.kForward);
		LEDRed.set(Relay.Value.kOn);

		LEDGreen = new Relay(GreenPort);
		LEDGreen.set(Relay.Value.kForward);
		LEDGreen.set(Relay.Value.kOff);

		LEDBlue = new Relay(BluePort);
		LEDBlue.set(Relay.Value.kForward);
		LEDBlue.set(Relay.Value.kOff);

		// NavX Init
		try
		{
			NavX = new AHRS(I2C.Port.kMXP);
			NavX.enableLogging(true);
		}
		catch (RuntimeException ex )
		{
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
		// GYRO Init
		// gyro.calibrate();
		gyro.reset();
		field_orientation = 0;

		// gyro.calibrate(); // Calibrate once in a while

		// Encoders Init
		enc1 = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		enc2 = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
		enc3 = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
		enc4 = new Encoder(6, 7, false, Encoder.EncodingType.k4X);
		encoder_reset();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		// LED Init
		LEDRed.set(Relay.Value.kOff);
		LEDGreen.set(Relay.Value.kOff);
		LEDBlue.set(Relay.Value.kOn);

		// Encoders Init
		encoder_reset();

		// GYRO Init
		gyro.reset();

		// Butterfly UP
		butterflySolenoid.set(UP);

		stage = 1;
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		AM_rangecalibrate();
		
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {

		// Joystick axis - definition
		int X_axis = 1, Y_axis = 0, Z_axis = 2;
		double forward = PS4.getRawAxis(X_axis)*MAXPOWER; 
		double slide = PS4.getRawAxis(Y_axis)*MAXPOWER; 
		double turn = -PS4.getRawAxis(Z_axis)*MAXPOWER;
		
		// Reset Gyro
		if (PS4.getRawButton(resetGyroId) == true) 
		{
			gyro.reset();
			NavX.reset();
			encoder_reset();
			field_orientation = 0;
		}

		// Input from Gyro
		angle = NavX.getYaw();

		// Test to check mode (Butterfly is DOWN, Tank Mode is UP)
		if (butterflyState == DOWN) // Tank mode
		{
			drive_tank(forward, slide); // slide turns the robot in tank drive
		}
		else
		{
			drive_meccanum(forward, slide, turn, angle);
		}
		// Change mode between Butterfly and Tank
		if (PS4.getRawButton(butterflyButtonId) == true && changeOfState == true) 
		{
			if (butterflyState == DOWN) 
			{
				butterflyState = UP;
				LEDRed.set(Relay.Value.kOff);
				LEDGreen.set(Relay.Value.kOn);
				LEDBlue.set(Relay.Value.kOff);
			}
			else 						
			{
				butterflyState = DOWN;
				LEDRed.set(Relay.Value.kOn);
				LEDGreen.set(Relay.Value.kOff);
				LEDBlue.set(Relay.Value.kOff);
			}
			changeOfState = false;
			butterflySolenoid.set(butterflyState);
		}
		
		// Reset state for driving mode
		// It will change states only when button has been released and pushed again
		if (PS4.getRawButton(butterflyButtonId) == false && changeOfState == false)  
			changeOfState = true;

		// Update SmartDashBoard
		updateDashboard();
	}



	public void drive_tank(double forward, double turn){
		DTMec.driveCartesian(forward, 0, turn);
	}

	public void drive_meccanum(double forward, double slide, double turn, double angle){	
		DTMec.driveCartesian(forward, slide, turn, -angle);
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	
//  AUTONOMOUS FUNCTIONS

	public void AM_forward_until_wall(double projectedDistance) // projectedDistance in inches
	{
		double power = 0;

		if (get_encoder_distance()<projectedDistance)
			power = -AM_MAXPOWER;
		else
		{
			if (get_ultrasound_distance()>30)
				power = -AM_MAXPOWER/2;
			else 
			{
				power = 0 ;
				stage = stage + 1;  // Finish with stage, go to the next stage...
				encoder_reset();
			}
		}
		DTMec.driveCartesian(power, 0, get_angle()/40);
		updateDashboard();
	}

	public void AM_forward_distance(double projectedDistance) // projectedDistance in inches
	{
		double power = 0;

		if (get_encoder_distance()<projectedDistance)
			power = -AM_MAXPOWER;
		else 
			{
				power = 0 ;
				stage = stage + 1;  // Finish with stage, go to the next stage...
				encoder_reset();
			}
		DTMec.driveCartesian(power, 0, get_angle()/40);
		updateDashboard();
	}

	public void AM_sideways_distance(double direction, double projectedDistance) // projectedDistance in inches
	{
		double power = 0;

		if (get_encoder_distance_sideways()<projectedDistance)
			power = -AM_MAXPOWER;
		else 
		{
			power = 0 ;
			stage = stage + 1;  // Finish with stage, go to the next stage...
			encoder_reset();
		}

		// LEFT: direction = 1
		// RIGHT: direction = -1
		DTMec.driveCartesian(0, power*direction, get_angle()/40);
		updateDashboard();
	}




	public void AM_rangecalibrate() {
		DTMec.driveCartesian(0, 0, -0);
		updateDashboard();
	}

//  UPDATE OF SENSORS AND DASHBOARD 
	public void updateDashboard(){
		double distance_travelled = get_encoder_distance();
		double ultrasound = get_ultrasound_distance();
		SmartDashboard.putNumber("Distance Travelled: ", distance_travelled);
		SmartDashboard.putNumber("Ultrasound: ", ultrasound);
		SmartDashboard.putNumber("Angle: ", gyro.getAngle());
		SmartDashboard.putNumber("enc1: ", enc1.get());
		SmartDashboard.putNumber("enc2: ", enc2.get());
		SmartDashboard.putNumber("enc3: ", enc3.get());
		SmartDashboard.putNumber("enc4: ", enc4.get());
		SmartDashboard.putNumber("Stage: ", stage);
		SmartDashboard.putBoolean(  "IMU_Connected",        NavX.isConnected());
        SmartDashboard.putBoolean(  "IMU_IsCalibrating",    NavX.isCalibrating());
        SmartDashboard.putNumber(   "IMU_Yaw",              NavX.getYaw());
		SmartDashboard.putNumber(   "IMU_Pitch",            NavX.getPitch());
		SmartDashboard.putNumber(   "IMU_Roll",             NavX.getRoll());
		SmartDashboard.putNumber(   "IMU_CompassHeading",   NavX.getCompassHeading());
		SmartDashboard.putNumber(   "IMU_FusedHeading",     NavX.getFusedHeading());
		SmartDashboard.putNumber(   "IMU_TotalYaw",         NavX.getAngle());
		SmartDashboard.putNumber(   "IMU_YawRateDPS",       NavX.getRate());
		SmartDashboard.putNumber(   "IMU_Accel_X",          NavX.getWorldLinearAccelX());
        SmartDashboard.putNumber(   "IMU_Accel_Y",          NavX.getWorldLinearAccelY());
		SmartDashboard.putBoolean(  "IMU_IsMoving",         NavX.isMoving());
		SmartDashboard.putBoolean(  "IMU_IsRotating",       NavX.isRotating());
		SmartDashboard.putNumber(   "Velocity_X",           NavX.getVelocityX());
        SmartDashboard.putNumber(   "Velocity_Y",           NavX.getVelocityY());
        SmartDashboard.putNumber(   "Displacement_X",       NavX.getDisplacementX());
        SmartDashboard.putNumber(   "Displacement_Y",       NavX.getDisplacementY());
	}

	public double get_encoder_distance(){
		double encoder;

		encoder = (-enc1.get()+enc3.get())/2;
		if (encoder<0) encoder = -encoder;
		return encoder*180/1570;
	}

	public double get_ultrasound_distance(){
		double ultrasound;

		ultrasound = ai.getAverageVoltage()*24/0.286;

		return ultrasound;
	}

	public double get_encoder_distance_sideways(){
		double encoder;

		encoder = (-enc1.get()+enc3.get())/2;
		if (encoder<0) encoder = -encoder;
		return encoder*110/1570;
	}

	public void encoder_reset(){
		enc1.reset();
		enc2.reset();
		enc3.reset();
		enc4.reset();
	}

	public double get_angle(){
		return NavX.getYaw();
	}

}

