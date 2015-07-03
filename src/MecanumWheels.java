
public class MecanumWheels extends AbstractOmniDrive{

	public static void main(String[] args) {
		new MecanumWheels().go();
	}

	public MecanumWheels() {
		motor1 = getMotor("A");
		motor2 = getMotor("B");
		motor3 = getMotor("1");
		motor4 = getMotor("2");
		setMaxSpeed();
	}

	/**
	 * decode incoming messages and handle motors accordingly
	 * 
	 * Wheels:
	 * 
	 * A---B
	 *  |||
	 *  |||
	 * 1---2
	 * 
	 * see: http://thinktank.wpi.edu/resources/346/ControllingMecanumDrive.pdf
	 */

	@Override
	protected void setMotorSpeed(float vx2, float vy2, float turningspeed2) {
		tempv1 = vy + vx + turningspeed;
		tempv2 = vy - vx - turningspeed;
		tempv3 = - vy + vx - turningspeed;
		tempv4 = - vy - vx + turningspeed;
	}
}
