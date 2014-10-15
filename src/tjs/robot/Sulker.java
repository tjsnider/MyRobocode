package tjs.robot;

import static robocode.util.Utils.normalRelativeAngle;
import static robocode.util.Utils.normalRelativeAngleDegrees;

import java.awt.Color;

import robocode.DeathEvent;
import robocode.HitRobotEvent;
import robocode.HitWallEvent;
import robocode.Robot;
import robocode.ScannedRobotEvent;

/**
 * Robot to go sit in a corner and fire towards the thick of combat
 * 
 * Based on Corners by Mathew A. Nelson and Flemming N. Larsen
 * 
 * @author Ted Snider
 *
 */
public class Sulker extends Robot {
	int others; // Number of other robots in the game
	private double maxX;
	private double maxY;
	static int corner = 0; // Which corner we are currently using
	// static so that it keeps it between rounds.

	public void run() {
		// 	Set colors
		setBodyColor(Color.black);
		setGunColor(Color.red);
		setRadarColor(Color.white);
		setBulletColor(Color.black);
		setScanColor(Color.white);

		// Save # of other bots
		others = getOthers();

		// get boundries
		maxX = getBattleFieldWidth() - 18.0;
		maxY = getBattleFieldHeight() - 18.0;
		
		// Move to a corner
		goCorner();
		
		// uncouple radar from gun
		setAdjustRadarForGunTurn(true);

		// Initialize gun turn speed to 3
		int radarIncrement = 3;

		// Spin radar back and forth
		while (true) {
			for (int i = 0; i < 30; i++) {
				turnRadarLeft(radarIncrement);
			}
			radarIncrement *= -1;
		}
	}
	
	/**
	 * goCorner:  modified to more reliably reach a corner
	 */
	public void goCorner() {
		// turn to face the wall to the "right" of our desired corner.
		turnRight(normalRelativeAngleDegrees(corner - getHeading()));
		// Move to that wall
		ahead(5000);
		// Turn to face the corner
		turnLeft(90);
		// Move to the corner
		ahead(5000);
		// rotate radar
		turnRadarLeft(90);
	}
	
	/**
	 * onHitRobot
	 * 
	 * need to be able to resume moving toward a corner
	 * 
	 */
	public void onHitRobot(HitRobotEvent e) {
		// are we in a corner?
		double xpos = getX();
		double ypos = getY();
		out.println("("+xpos+", "+ypos+")");
		if (!(((xpos == maxX) || (xpos == 0.0)) && // left or right corner
			  ((ypos == maxY) || (ypos == 0.0)))) {
			out.println("Not in a corner; heading to corner "+corner);
			goCorner();
		}
	}
	
	public void onHitWall(HitWallEvent e) {
		double xpos = getX();
		double ypos = getY();
		out.println("("+xpos+", "+ypos+")");
	}

	/**
	 * onScannedRobot
	 * 
	 * 1. check heat
	 * 2. calculate shot power based in distance...weaker/faster for further targets
	 * 3. determine an amount to lead target based on current velocity
	 * 
	 * @param ScannedRobotEvent e - event pertaining to robot scanned
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		// No point if heat is > 0
		if (getGunHeat() == 0) {
			double robotDistance = e.getDistance();
			double bullet = 1.0;
			
			// lower power moves faster and easier to reach distant targets
			if (robotDistance > 200 || getEnergy() < 15) {
				bullet = 1.0;
			} else if (robotDistance > 50) {
				bullet = Math.min(2, getEnergy());
			} else {
				bullet = Math.min(3, getEnergy());
			}
			
			// calculate rough linear prediction targeting
			double absoluteBearing = toRadians(getHeading()) + e.getBearingRadians();
			double adjustment = (e.getVelocity() * Math.sin(e.getHeadingRadians() - absoluteBearing) / 13.0);
			turnGunRight(toDegrees(normalRelativeAngle(absoluteBearing - toRadians(getGunHeading()) + adjustment)));
			
			// kill
			fire(bullet);
		}
	}

	/**
	 * onDeath:  We died.  Decide whether to try a different corner next game.
	 */
	public void onDeath(DeathEvent e) {
		// Well, others should never be 0, but better safe than sorry.
		if (others == 0) {
			return;
		}

		// If 75% of the robots are still alive when we die, we'll switch corners.
		if ((others - getOthers()) / (double) others < .75) {
			corner += 90;
			if (corner == 270) {
				corner = -90;
			}
			out.println("I died and did poorly... switching corner to " + corner);
		} else {
			out.println("I died but did well.  I will still use corner " + corner);
		}
	}
	
	/**
	 * since this is a basic bot, I can only get bearing, etc., in degrees, 
	 * but my algorithms all seem to use radians.
	 * 
	 *  yay
	 *  
	 *  conversion is not exact, but close enough
	 *  
	 * @param degrees
	 * @return double - radian equivalent of the degree measure 
	 */
	private double toRadians(double degrees) {
		return degrees * 0.0174532925;
	}
	
	/**
	 * since this is a basic bot, I can only set bearing, etc., in degrees,
	 * but my algorithms all seem to use radians.
	 * 
	 * yay
	 * 
	 * conversion is not exact, but close enough
	 * 
	 * @param radians
	 * @return double - degree equivalent of the radian measure
	 */
	private double toDegrees(double radians) {
		return radians * 57.2957795;
	}
}
