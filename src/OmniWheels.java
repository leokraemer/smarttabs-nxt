
public class OmniWheels extends AbstractOmniDrive {

	public static void main(String[] args) {
		new OmniWheels().go();
	}

	public OmniWheels() {
		motor1 = getMotor("1");
		motor2 = getMotor("2");
		motor3 = getMotor("A");
		motor4 = getMotor("B");
		setMaxSpeed();
	}

	/**
	 * Wheels:
	 * Front
	 * ---1---
	 * |  |  |
	 * 2- - -3
	 * |  |  |
	 * ---4---
	 * Back 
	 * see: http://www.firstroboticscanada.org/main/wp-content/uploads/Omnidirectional-Drive-Systems.pdf
	 */
	
	@Override
	protected void setMotorSpeed(float vx2, float vy2, float turningspeed2) {
		tempv1 = vy + turningspeed;
		tempv2 = vx + turningspeed;
		// motors are mounted 'backwards', therefore we must
		// negate the speed
		tempv3 = - vy + turningspeed;
		tempv4 = - vx + turningspeed;
	}				
	
    @Override
    protected void driveLiftingarmToAngle(float angle) {
    	// TODO Auto-generated method stub
    	
    }
}
