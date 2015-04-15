package org.usfirst.frc.team2240.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.AnalogInput;

import java.lang.Math;

public class Robot extends IterativeRobot {
	//--------------------------------CONSTANTS--------------------------------//
	private final int 	   frontLeftChannel		= 0;
    private final int 	   rearLeftChannel		= 2;
    private final int 	   frontRightChannel	= 1;
    private final int 	   rearRightChannel		= 3;
    private final int 	   stickChannel			= 0;
    private	final double   ROT_PERCENTAGE  		= .9;
    private double   	   FAB_PERCENTAGE 		= 1.0;
    private	final double   LAR_PERCENTAGE		= 1.0;
    private	final double   ROT_DEADBAND    		= .15;
    private	final double   FAB_DEADBAND 		= .1; //was .1 had to change for crazy joystick input
    private	final double   LAR_DEADBAND 		= .25;
    private	final double   ULTRASONIC_SCALE		= (1024/5.0);
	//--------------------------------VARIABLES--------------------------------//
    private double 	axisS 			    = 0.0;
    private	float  	lCount				= 0;
    private float  	rCount				= 0;
    private float	cDifference			= 0;
    private double	aUElevVal			= -1.0;		//right up (negative)
    private double  aDElevVal			= 1;		//right down (positive)
    private	double	bUElevVal			= -.87;	//left up 85 too slow 88.95 too fast last change 85 to 87
    private double  bDElevVal			= .83;		//left down 80 too slow 85 too fast last change 85 to 83
    private	int		elevDirection		= 0;
    private int    	i;
    private int		autoMode			= 0;
    private	double	rfValue				= 0.0;
    private boolean autoDone			= false;
    private boolean broke				= false;
    int session;
    Image frame;
	//--------------------------------ROBOT CODE-------------------------------//
    public void robotInit() { //Init code ran once on startup
    	mecanumDrive 	= 	new RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);
    	mecanumDrive.setInvertedMotor(MotorType.kFrontLeft, true);	// invert the left side motors
    	mecanumDrive.setInvertedMotor(MotorType.kRearLeft, true);
    	mecanumDrive.setSafetyEnabled(true);
		mecanumDrive.setExpiration(0.1);
    	
		ultraSensor = new AnalogInput(0);
		
        sMotorA			=	new	Talon(4); //left
    	sMotorB			=	new	Talon(5); //right
    	wMotorA			=	new Relay(0);  //left
    	wMotorB			=	new	Relay(1);  //right
    	lEnc 			= 	new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    	rEnc 			= 	new Encoder(2, 3, false, Encoder.EncodingType.k4X);
    	stick 			= 	new Joystick(stickChannel);
    	bottomLS		=	new DigitalInput(4);
        topLS			=	new	DigitalInput(5);
    	
    	lEnc.setMaxPeriod(0.1); //Seconds until device stops
    	rEnc.setMaxPeriod(0.1);
    	lEnc.setDistancePerPulse(5);
    	rEnc.setDistancePerPulse(5);
    	lEnc.setSamplesToAverage(7);
    	rEnc.setSamplesToAverage(7);
    	
    	try{
    	frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

        session = NIVision.IMAQdxOpenCamera("cam0",
                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
        NIVision.IMAQdxConfigureGrab(session);
    	}catch(Throwable ex){
    	}
        
    	lEnc.reset();
    	rEnc.reset();
    }
	public void autonomousInit() {
		
	}
    public void autonomousPeriodic() { //Autonomous code called periodically
    	if(!autoDone){
    		auto("Forward", 1.0, .15);
    		autoDone = true;
    	}
    	/*if(autoMode == 0 && !autoDone){
    		auto("Clamp");
    		Timer.delay(0.2);
    		auto("Lift");
    		Timer.delay(0.2);
    		auto("Rotate Left", 1.0, 1.0);
    		Timer.delay(0.2);
    		auto("Forward", 1.0, 2.25);
    		Timer.delay(0.2);
    		auto("Unclamp");
    		Timer.delay(0.2);
    		auto("Backward", 1.0, 0.5);
    		Timer.delay(0.2);
    		auto("Lower");
    	}else if(autoMode == 1 && !autoDone)
    	{
    		auto("Clamp");
    		Timer.delay(0.2);
    		auto("Lift");
    		Timer.delay(0.2);
    		auto("Strafe Right", 1.0, 1.85);
    		Timer.delay(0.2);
    		auto("Backward", 1.0, 0.75);
    		Timer.delay(0.2);
    		auto("Unclamp");
    		Timer.delay(0.2);
    		auto("Forward", 1.0, 0.30);
    		auto("Lower");
    		autoDone = true;
    	}else if(autoMode == 1 && !autoDone){
    		auto("Clamp");
    		Timer.delay(0.2);
    		auto("Lift");
    		Timer.delay(0.2);
    		auto("Strafe Left", 1.0, 1.85);
    		Timer.delay(0.2);
    		auto("Forward", 1.0, 0.75);
    		Timer.delay(0.2);
    		auto("Unclamp");
    		Timer.delay(0.2);
    		auto("Backward", 1.0, 0.30);
    		auto("Lower");
    		autoDone = true;
    	}else if(autoMode == 2 && !autoDone){
    		autoDone = true;
    	}
    	sMotorA.set(0.0);
        sMotorB.set(0.0);*/
        mecanumDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    }
    public void teleopInit() {
    }
	public void teleopPeriodic() { //Tele-op code called periodically
			rfValue = ultraSensor.getAverageVoltage() * ULTRASONIC_SCALE;
			
	        //NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);
			if(!broke){
				try{
					NIVision.IMAQdxStartAcquisition(session);
					NIVision.IMAQdxGrab(session, frame, 1);
					CameraServer.getInstance().setImage(frame);
				}catch(Throwable ex){
					broke = true;
				}
			}
			
			/*NIVision.imaqDrawShapeOnImage(frame, frame, rect,
	                DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);*/
	        
			axisS = stick.getRawAxis(3);
	    	axisS = ((axisS*-1) + 1)/2;
	    	if(stick.getRawButton(7)){
	    		FAB_PERCENTAGE = 0.0;
	    	}
	    	mecanumDrive.mecanumDrive_Cartesian((axisS * (LAR_PERCENTAGE * deadBand(-stick.getX(), LAR_DEADBAND))), (axisS * (FAB_PERCENTAGE * deadBand(stick.getY(), FAB_DEADBAND))), (axisS * (ROT_PERCENTAGE * deadBand(-stick.getZ(), ROT_DEADBAND))), 0);
	    	FAB_PERCENTAGE = 1.0;
	    	rCount = rEnc.get();
	    	lCount = lEnc.get(); 	
	    	cDifference = (rCount - lCount)/500; //Negative = left is ahead, positive = right is ahead
	    	
	    	SmartDashboard.putNumber("Left Elevator Up Value", aUElevVal);
	    	SmartDashboard.putNumber("Left Elevator Down Value", aDElevVal);
	    	SmartDashboard.putNumber("Right Elevator Up Value", bUElevVal);
	    	SmartDashboard.putNumber("Right Elevator Down Value", bDElevVal);
	    	SmartDashboard.putNumber("Elevator Direction", elevDirection);
	    	SmartDashboard.putNumber("Left Encoder Count Raw:", lEnc.getRaw());
	    	SmartDashboard.putNumber("Right Encoder Count Raw:", rEnc.getRaw());
	    	SmartDashboard.putNumber("Left Encoder Count:", lEnc.get());
	    	SmartDashboard.putNumber("Right Encoder Count:", rEnc.get());
	    	SmartDashboard.putNumber("Encoder Count Difference:", cDifference);
	    	SmartDashboard.putNumber("Left Encoder Rate:", lEnc.getRate());
	    	SmartDashboard.putNumber("Right Encoder Rate:", rEnc.getRate());
	    	SmartDashboard.putBoolean("Left Encoder Running?:", lEnc.getStopped());
	    	SmartDashboard.putBoolean("Right Encoder Running?:", rEnc.getStopped());
	    	SmartDashboard.putBoolean("Limit Switch:", bottomLS.get());
	    	SmartDashboard.putBoolean("Limit Switch:", topLS.get());
	    	SmartDashboard.putNumber("Ultrasonic Sensor:", rfValue);
	    	SmartDashboard.putNumber("Stick thingy", stick.getPOV());

	    	if(((stick.getPOV() == 135 || stick.getPOV() == 180 || stick.getPOV() == 225) || stick.getRawButton(11)) && !bottomLS.get()){
	    		elevDirection = 1; //down
	    	}else if(((stick.getPOV() == 315 || stick.getPOV() == 0 || stick.getPOV() == 45) || stick.getRawButton(12)) && !topLS.get()){
	    		elevDirection = 2; //up
	    	}else{
	    		elevDirection = 3; // not moving.
	    	}
	    	if(elevDirection != 1 && elevDirection != 2 && elevDirection != 3){
	    		elevDirection = 3;
	    	}
	    		    	
	    	/*aUElevVal = 1.0; //right
	    	aDElevVal = -1.0;
	    	bUElevVal = 0.88; //left
	    	bDElevVal = -0.925;*/
	    	
	    	/*if(Math.abs(cDifference) >=7){
	    		if(cDifference < 0) { //left is ahead
	    			if(elevDirection == 1) { //up
		    			aElevVal = 1.0;
		    			bElevVal = 0.5;
	    			}else if (elevDirection == 2){ //down
		    			aElevVal = 0.5;f
		    			bElevVal = 1.0;
	    			}
	    		}else{ //right is ahead
	    			if(elevDirection == 1){
	    				aElevVal = 1.0; // left
		    	    	bElevVal = 0.5; // right
	    			}else if(elevDirection == 2){
	    				aElevVal = 0.5; // left
		    	    	bElevVal = 1.0; // right
	    			}
	    		}
	    	}else{
	    		aElevVal = 1.0;
	        	bElevVal = 1.0;
	    	}*/
	    	switch(elevDirection){
	    	case 1:
	    		sMotorA.set(aDElevVal);
        		sMotorB.set(bDElevVal);
	    		break;
	    	case 2:
	    		sMotorA.set(aUElevVal);
        		sMotorB.set(bUElevVal);
	    		break;
	    	case 3:
	    			if(stick.getRawButton(4) && stick.getRawButton(9)){
	            		sMotorA.set(.5);
	            	}else if(stick.getRawButton(6) && stick.getRawButton(9)){
	            		sMotorA.set(-.5);
	            	}else{
	            		sMotorA.set(0.0);
	            	}
	            	if(stick.getRawButton(3) && stick.getRawButton(9)){
	            		sMotorB.set(.5);	
	            	}else if(stick.getRawButton(5) && stick.getRawButton(9)){
	            		sMotorB.set(-.5);
	            	}else{
	            		sMotorB.set(0.0);
	            	}
	    		break;
	    	default:
	    		sMotorA.set(0.0);
	    		sMotorB.set(0.0);
	    		break;
	    	}
        	if(stick.getRawButton(1)){
        		wMotorA.setDirection(Relay.Direction.kForward);
        		wMotorA.set(Relay.Value.kOn);
        		wMotorB.setDirection(Relay.Direction.kForward);
        		wMotorB.set(Relay.Value.kOn);
        	}else if(stick.getRawButton(2)){
        		wMotorA.setDirection(Relay.Direction.kReverse);
        		wMotorA.set(Relay.Value.kOn);
        		wMotorB.setDirection(Relay.Direction.kReverse);
        		wMotorB.set(Relay.Value.kOn);
        	}else{
        		wMotorA.set(Relay.Value.kOff);
        		wMotorB.set(Relay.Value.kOff);
        	}
    }
    //-------------------------------OTHER METHODS-------------------------------//
    private double deadBand(double axisValue, double Amount)
    {
    	if(Math.abs(axisValue) < Amount){
    		return 0.0;
    	}else{
    		return axisValue;
    	}
    }
    private void auto(String command){
    	if(command == "Clamp"){
    		for(i = 0;i<5500;i++){
    			wMotorA.setDirection(Relay.Direction.kForward);
        		wMotorA.set(Relay.Value.kOn);
        		wMotorB.setDirection(Relay.Direction.kForward);
        		wMotorB.set(Relay.Value.kOn);
    		}
    		wMotorA.set(Relay.Value.kOff);
    		wMotorB.set(Relay.Value.kOff);
    	}else if(command == "Unclamp"){
    		for(i = 0;i<5500;i++){
    			wMotorA.setDirection(Relay.Direction.kReverse);
        		wMotorA.set(Relay.Value.kOn);
        		wMotorB.setDirection(Relay.Direction.kReverse);
        		wMotorB.set(Relay.Value.kOn);
    		}
    		wMotorA.set(Relay.Value.kOff);
    		wMotorB.set(Relay.Value.kOff);
    	}else if(command == "Lift"){
    		for(i = 0;i<3000;i++){
    	    	sMotorA.set(aUElevVal);
    	    	sMotorB.set(bUElevVal);
    		}
    		sMotorA.set(0.0);
    		sMotorB.set(0.0);
    	}else if(command == "Lower"){
    		while(!bottomLS.get()){
    			sMotorA.set(-aDElevVal);
    			sMotorB.set(-bDElevVal);
    		}
    		sMotorA.set(0.0);
    		sMotorB.set(0.0);
    	}
    }
    private void auto(String command, double power, double time){
    	if(command == "Forward"){
    		for(i=0;i<(time * 10000);i++){
    			mecanumDrive.mecanumDrive_Cartesian(0, -power, 0, 0);	
    		}
    		mecanumDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    	}else if(command == "Backward"){
    		for(i=0;i<(time * 10000);i++){
    			mecanumDrive.mecanumDrive_Cartesian(0, power, 0, 0);	
    		}
    		mecanumDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    	}else if(command == "Strafe Left"){    	
    		for(i=0;i<(time * 10000);i++){
				mecanumDrive.mecanumDrive_Cartesian(power, 0, 0, 0);	
			}
			mecanumDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    	}else if(command == "Strafe Right"){
    		for(i=0;i<(time * 10000);i++){
    			mecanumDrive.mecanumDrive_Cartesian(-power, 0, 0, 0);	
    		}
    		mecanumDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    	}else if(command == "Rotate Right"){
    		for(i=0;i<(time * 10000);i++){
    			mecanumDrive.mecanumDrive_Cartesian(0, 0, 0, -power);	
    		}
    		mecanumDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    	}else if(command == "Rotate Left"){
    		for(i=0;i<(time * 10000);i++){
    			mecanumDrive.mecanumDrive_Cartesian(0, 0, 0, power);	
    		}
    		mecanumDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    	}
    }
    private   RobotDrive 			mecanumDrive;
    private   Joystick 				stick;
    private	  Talon					sMotorA;//Robot Left Screw Motor
    private   Talon					sMotorB;//Robot Right Screw Motor
    private	  Relay					wMotorA;//Robot Left Snowblower Motor
    private   Relay					wMotorB;//Robot Right Snowblower Motor
    private   Encoder				lEnc;//Left Encoder for elevator
    private	  Encoder				rEnc;//Right Encoder for elevator
    private	  DigitalInput			bottomLS; // Left limit switch for elevator
    private   DigitalInput			topLS; // RIght limit switch for elevator
    private	  AnalogInput			ultraSensor;
}