/**
I developed the initial code in java at home and converted it to C.
see "gr9improved.c"

*/
public class gr9improved {

	static int leftMotor = 0;
	static int rightMotor = 1;
	static int[] motor = new int[2];

	public static void main(String[] args) {
		setMotors(-127, -127);
		//setMotors(127, -127);
	}
	private static void setMotors(int chan1, int chan2){
		float power;
		float angle;
		power = (float)(Math.min(Math.sqrt((chan1 * chan1) + (chan2 * chan2)),127));
		angle = (float)(Math.toDegrees(Math.atan(chan2 / chan1)));
		if (chan2 == 0) {
			motor[leftMotor] = 0;
			motor[rightMotor] = 0;
		} else {
			if (chan1 > 0) {
				if (chan2 > 0) {
					motor[leftMotor]  = (int)(power * (Math.min(angle + 45, 90) / 90));
					motor[rightMotor] = (int)(power * ((angle) / 90));
				} else {
					motor[leftMotor]  = (int)(power * ((-angle) / 90));
					motor[rightMotor] = (int)(power * (Math.min(-angle + 45, 90) / 90));
				}
			} else if (chan1 < 0) {
				if (chan2 > 0) {
					motor[leftMotor]  = -(int)(power * (Math.min(-angle + 45, 90) / 90));
					motor[rightMotor] = -(int)(power * ((-angle) / 90));
				} else {
					motor[leftMotor]  = -(int)(power * ((angle) / 90));
					motor[rightMotor] = -(int)(power * (Math.min(angle + 45, 90) / 90));
				}
			} else {
				motor[leftMotor] = chan2;
				motor[rightMotor] = chan2;
			}
		}
		System.out.println(angle);
		System.out.println(power);
		System.out.println("L="+motor[leftMotor]+", R="+motor[rightMotor]+"\n");
	}
}

