package tjs.robot;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static robocode.util.Utils.normalRelativeAngle;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.RoundRectangle2D;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.Consumer;

import robocode.AdvancedRobot;
import robocode.BattleEndedEvent;
import robocode.BulletHitBulletEvent;
import robocode.BulletHitEvent;
import robocode.BulletMissedEvent;
import robocode.Condition;
import robocode.CustomEvent;
import robocode.DeathEvent;
import robocode.HitRobotEvent;
import robocode.HitWallEvent;
import robocode.RobotDeathEvent;
import robocode.RoundEndedEvent;
import robocode.Rules;
import robocode.ScannedRobotEvent;
import robocode.StatusEvent;
import robocode.WinEvent;

/**
 * Robot to go sit in a corner and fire towards the thick of combat
 * 
 * Based on Corners by Mathew A. Nelson and Flemming N. Larsen
 * 
 * @author Ted Snider
 *
 */
public class Sulker extends AdvancedRobot {
	// movement and battlefield dimension attributes.
	Point2D destination;
	Rectangle2D boundries;
	double margin = 40;
	double minX = margin;
	double minY = margin;
	double rectHeight;
	double rectWidth;
	private boolean found;
	int others; // Number of other robots in the game
	int moveToggle = 1; // allow movement direction to flip back and forth
	int radarToggle = 1; // allow radar to flip back and forth

	// Maps for handling scanning of enemy bots
	Map<String, Map<String, Number>> scanned; // contains vital statistics of
												// potential targets
	Map<String, Number> enemy; // current working potential target
	Map<String, List<Map<String, Number>>> history = new HashMap<String, List<Map<String, Number>>>();
								// history of scanned targets
	Map<String, Number> identity; // identity map for parallel stream reduce methods

	Map<String, Integer> stats;
	static List<Map<String, Integer>> statHistory = new ArrayList<Map<String, Integer>>();

	// Since I'm always turning something to match a given bearing and all that
	// varies is what...
	Consumer<Double> gun = bearing -> {
		setTurnGunRightRadians(normalRelativeAngle(bearing - getGunHeadingRadians()));
	};
	Consumer<Double> body = bearing -> {
		setTurnRightRadians(normalRelativeAngle(bearing - getHeadingRadians()));
	};
	Consumer<Double> radar = bearing -> {
		setTurnRadarRightRadians(normalRelativeAngle(bearing - getRadarHeadingRadians()));
	};

	// list of movement states utilized by move() function
	List<Consumer<Point2D>> movementStates;
	// current movement strategy
	int moveState = initializeMovementStates();
	
	// list of radar sweep strategies
	List<Consumer<Point2D>> radarStrategies = Arrays.asList(
			p -> {
				if (found) {
					found = false;
					radarToggle = 0 - radarToggle;
				}
				turnRadarRight(45 * radarToggle);
			},
			p -> {
				Point2D next = plotNextCorner(p);
				turnToHeading(atan2(next.getX() - p.getX(), next.getY() - p.getY()), radar);
				radarState = 2;
			},
			p -> {
				Point2D prev = plotPreviousCorner(p);
				turnToHeading(atan2(prev.getX() - p.getX(), prev.getY() - p.getY()), radar);
				radarState = 1;
			},
			p -> {
				turnToHeading(enemy.get("originalBearing").doubleValue() + PI/2, radar);
				radarState = 4;
			},
			p -> {
				turnToHeading(enemy.get("originalBearing").doubleValue() - PI/2, radar);
				radarState = 3;
			});
	// current radar strategy
	int radarState = 1;
	
	// list of targeting strategies
	List<Consumer<Map<String, Number>>> targetingStrategies = Arrays.asList(
	// weighted linear prediction
			e -> {
				linearPrediction(e);
			}, e -> {
				gaussianDistributionPrediction(e);
			});

	// List of options for gun ready states
	List<Consumer<Map<String, Number>>> gunActions = Arrays.asList(g -> {
		turnToHeading(g.get("bearing").doubleValue(), gun);
		gunState = 1;
	}, g -> {
		setFire(g.get("power").doubleValue());
		stats.put("shots", stats.get("shots") + 1);
		gunState = 0;
	});
	int gunState = 0;

	public void run() {
		initialize();

		double x = getX();
		double y = getY();

		sweep(new Point2D.Double(x, y));
		move(new Point2D.Double(x, y));

		while (true) {
			execute();
		}
	}

	private void shoot() {
		// out.println("Shooting. Strategy: "+targetingState);
		smartFire();
	}

	private void sweep(Point2D p) {
		// out.println("Sweeping. Strategy: "+radarState);
		radarStrategies.get(radarState).accept(p);
	}

	private void move(Point2D p) {
		if (others == 1) { // Begin the duel!
			moveState = 5;
			radarState = 0;
		}
		// out.println("Moving. Strategy: "+moveState);
		movementStates.get(moveState).accept(p);
	}
	
	/**
	 * onCustomEvent
	 * 
	 * handle user-defined events.
	 * 
	 * Currently when radar completes a sweep or when movement needs rechecked
	 * 
	 */
	public void onCustomEvent(CustomEvent e) {
		String cond = e.getCondition().getName();
		
		switch (cond) {
			case "sweepComplete" : 
				sweep(new Point2D.Double(getX(), getY()));
				shoot();
				break;
			case "moveComplete" :
				move(new Point2D.Double(getX(), getY()));
				break;
			case "turnComplete" :
				setMaxVelocity(Rules.MAX_VELOCITY);
				move(new Point2D.Double(getX(), getY()));
				break;
		}
	}
	
	/**
	 * onStatus
	 * 
	 * fires every turn
	 * 
	 */
	public void onStatus(StatusEvent e) {
		if (boundries != null) {
			double heading = getHeadingRadians();
			double velocity = Rules.MAX_VELOCITY;
			double xo = getX();
			double yo = getY();
			double xd = xo + Math.sin(heading) * velocity;
			double yd = yo + Math.cos(heading) * velocity; 
			if (!boundries.contains(xo, yo)) {
				recenter(new Point2D.Double(xo, yo));
			} else if (getTurnRemainingRadians() != 0) {
				while (!boundries.contains(xd, yd) && velocity >= 0) {
					velocity--;
					xd = xo + Math.sin(heading) * velocity;
					yd = yo + Math.cos(heading) * velocity; 
				}
				if (velocity < 0) {
					recenter(new Point2D.Double(xo, yo));
				}
			} 
			setMaxVelocity(velocity);
		}
	}

	/**
	 * onHitRobot
	 * 
	 * need to be able to resume moving toward a corner
	 * 
	 */
	public void onHitRobot(HitRobotEvent e) {
		recenter(new Point2D.Double(getX(), getY()));
	}

	public void onHitWall(HitWallEvent e) {
/*		// plot a course from the center out to this point
		destination.setLocation(boundries.getCenterX(), boundries.getCenterY());
		Point2D here = new Point2D.Double(getX(), getY());
		turnToHeading(atan2(here.getX() - destination.getX(), here.getY() - destination.getY()), body);
		setBack(destination.distance(here));
*/	}

	/**
	 * onRobotDeath: update other robot count variable for strategic adjustments
	 */
	public void onRobotDeath(RobotDeathEvent e) {
		others = getOthers();
		scanned.remove(e.getName());
		if (others == 1) { // Begin the duel!
			moveState = 5;
			radarState = 0;
		}
		out.println(e.getName() + " died.  " + others + " remain.");
	}

	/**
	 * onScannedRobot
	 * 
	 * 1. check heat 2. calculate shot power based in distance...weaker/faster
	 * for further targets 3. determine an amount to lead target based on
	 * current velocity
	 * 
	 * @param ScannedRobotEvent
	 *            e - event pertaining to robot scanned
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		// out.println("Scanned a robot.");
		found = true;
		enemy = new HashMap<String, Number>();
		enemy.put("time", getTime());
		enemy.put("distance", e.getDistance());
		enemy.put("originalBearing", e.getBearingRadians());
		enemy.put("absoluteBearingRadians",
				getHeadingRadians() + e.getBearingRadians());
		enemy.put("velocity", e.getVelocity());
		enemy.put("heading", e.getHeadingRadians());
		enemy.put("headingRadians", e.getHeadingRadians());
		enemy.put("energy", e.getEnergy());
		enemy.put("origX",
				getX() + Math.sin(enemy.get("absoluteBearingRadians").doubleValue())
						* enemy.get("distance").doubleValue());
		enemy.put("origY",
				getY() + Math.cos(enemy.get("absoluteBearingRadians").doubleValue())
						* enemy.get("distance").doubleValue());
		// lower power moves faster and easier to reach distant targets
		double bullet = 1.0;
		double energy = getEnergy();
		if (enemy.get("distance").doubleValue() > 400 || energy < 15) {
			bullet = 1.0;
		} else if (enemy.get("distance").doubleValue() > 200) {
			bullet = 2 < energy ? 2 : energy;
		} else {
			bullet = 3 < energy ? 3 : energy;
		}
		enemy.put("power", bullet);
		enemy.put(
				"wallcrawling",
				enemy.get("velocity").doubleValue() != 0.0
						&& (enemy.get("heading").doubleValue() == 0.0
								|| enemy.get("heading").doubleValue() == PI / 2
								|| enemy.get("heading").doubleValue() == PI || enemy
								.get("heading").doubleValue() == 1.5 * PI) ? 1
						: 0);
		// add scan to list of scanned robots
		String name = e.getName();
		scanned.put(name, enemy);

		// add scan to history of that enemy's scans
		if (!(history.containsKey(name))) {
			List<Map<String, Number>> empty = new LinkedList<Map<String, Number>>();
			history.put(name, empty);
		} else {
			Map<String, Number> lastScan = history.get(name).get(
					history.get(name).size() - 1);
			if (enemy.get("energy").doubleValue() != lastScan.get("energy")
					.doubleValue() && Math.random() > 0.3) {
				moveToggle = 0 - moveToggle;
			}
			enemy.put("prevVelo", lastScan.get("velocity").doubleValue());
			enemy.put("prevX", lastScan.get("origX").doubleValue());
			enemy.put("prevY", lastScan.get("origY").doubleValue());
			enemy.put("prevHeading", lastScan.get("heading").doubleValue());
		}
		history.get(name).add(enemy);

		// analyze history for wall-crawling behavior -- easier to hit with
		// simple linear prediction
		long amtcrawling = history.get(name).stream()
				.filter(r -> r.get("wallcrawling").intValue() == 1).count();
		double percentage = amtcrawling * 100 / history.get(name).size();
		boolean wallcrawler = percentage > 70;
		if (wallcrawler) {
			targetingStrategies.get(0).accept(enemy);
		} else {
			targetingStrategies.get(1).accept(enemy);
		}
		
		//out.println("Scanned robot: "+enemy);
	}

	public void onBulletHit(BulletHitEvent e) {
		stats.put("hits", stats.get("hits") + 1);
	}

	public void onBulletMissed(BulletMissedEvent e) {
		stats.put("misses", stats.get("misses") + 1);
	}

	public void onBulletHitBullet(BulletHitBulletEvent e) {
		stats.put("bullets", stats.get("bullets") + 1);
	}

	public void onDeath(DeathEvent e) {
		if (stats.get("shots") != 0 && stats.get("written").intValue() == 0) {
			out.println("Statistics -- " + stats);
			out.println("Accuracy: " + stats.get("hits") * 100
					/ stats.get("shots"));
			stats.put("written", 1);
			statHistory.add(stats);

			int hitstat = statHistory.stream()
					.mapToInt(s -> s.get("hits").intValue()).sum();
			int shotstat = statHistory.stream()
					.mapToInt(s -> s.get("shots").intValue()).sum();
			out.println("Battle Shots Fired: " + shotstat);
			out.println("Battle Hits Scored: " + hitstat);
			out.println("Battle Accurracy: " + hitstat * 100 / shotstat);
		}
	}

	public void onRoundEnded(RoundEndedEvent e) {
		if (stats.get("shots") != 0 && stats.get("written").intValue() == 0) {
			out.println("Statistics -- " + stats);
			out.println("Accuracy: " + stats.get("hits") * 100
					/ stats.get("shots"));
			stats.put("written", 1);
			statHistory.add(stats);

			int hitstat = statHistory.stream()
					.mapToInt(s -> s.get("hits").intValue()).sum();
			int shotstat = statHistory.stream()
					.mapToInt(s -> s.get("shots").intValue()).sum();
			out.println("Battle Shots Fired: " + shotstat);
			out.println("Battle Hits Scored: " + hitstat);
			out.println("Battle Accurracy: " + hitstat * 100 / shotstat);
		}
	}

	public void onBattleEnded(BattleEndedEvent e) {
	}

	/**
	 * Stolen directly from TrackFire bot
	 * 
	 */
	public void onWin(WinEvent e) {
		// Victory dance
		turnRight(36000);
	}

	public void onPaint(Graphics2D g) {
		double x = getX();
		double y = getY();
		double heading = getHeadingRadians();
		double headlightX = x + Math.sin(heading) * 100;
		double headlightY = y + Math.cos(heading) * 100;
		g.setColor(Color.white);
		g.drawLine((int)x, (int)y, (int)headlightX, (int)headlightY);
		
		g.setColor(Color.yellow);
		g.drawLine((int)x, (int)y, (int)destination.getX(), (int)destination.getY());

		RoundRectangle2D corner1;
		RoundRectangle2D corner2;
		RoundRectangle2D corner3;
		RoundRectangle2D corner4;
		// dodging about in a corner has a looser definition of corner than
		// other movement states
			corner1 = new RoundRectangle2D.Double(0, 0, 100, 100, 36, 36);
			corner2 = new RoundRectangle2D.Double(0,
					getBattleFieldWidth() - 100, 100, 100, 36, 36);
			corner3 = new RoundRectangle2D.Double(getBattleFieldHeight() - 100,
					getBattleFieldWidth() - 100, 100, 100, 36, 36);
			corner4 = new RoundRectangle2D.Double(getBattleFieldHeight() - 100,
					0, 100, 100, 36, 36);
		g.setColor(Color.magenta);
		g.draw(corner1);
		g.draw(corner2);
		g.draw(corner3);
		g.draw(corner4);
		g.draw(boundries);

		/*
		 * g.setColor(Color.green); scanned.entrySet().stream() .filter(r ->
		 * (getTime() - r.getValue().get("time").longValue()) < 5) .forEach( r
		 * -> g.drawLine((int)x, (int)y,
		 * ((Integer)r.getValue().get("x")).intValue(),
		 * ((Integer)r.getValue().get("y")).intValue()));
		 */
	}

	private void recenter(Point2D origin) {
		moveState = 6;
		move(origin);
	}

	/**
	 * linearPrediction: uses weighted linear prediction to predict movement and
	 * weight targets
	 * 
	 * @param e
	 *            - ScannedRobotEvent
	 */
	private void linearPrediction(Map<String, Number> e) {
		double x = getX();
		double y = getY();
		double robotDistance = e.get("distance").doubleValue();
		double robotVelocity = e.get("velocity").doubleValue();
		double bullet = e.get("power").doubleValue();
		double absoluteBearing = e.get("absoluteBearingRadians").doubleValue();

		// calculate rough linear prediction targeting
		double adjustment = Math.asin((robotVelocity
				* Math.sin(e.get("headingRadians").doubleValue()
						- absoluteBearing) / Rules.getBulletSpeed(bullet)));
		// check for walls
		Point2D targPoint = checkWallCollision(new Point2D.Double(x, y),
				absoluteBearing + adjustment, robotDistance);
		double angle = atan2(targPoint.getX() - x, targPoint.getY() - y);
		enemy.put("bearing", angle);
		enemy.put("adjustment", adjustment);
		enemy.put("x", new Integer((int) targPoint.getX()));
		enemy.put("y", new Integer((int) targPoint.getY()));

		if (enemy.get("energy").doubleValue() == 0.0) {
			enemy.put("score", 1000.0);
		} else {
			enemy.put("score", -(adjustment + angle / (PI / 4) + robotDistance));
		}
		// out.println("Added scanned robot: "+scan);
	}

	/**
	 * gaussianDistributionPrediction: linear prediction as above, but adjusted
	 * to factor in random guessing with a gaussian distribution of probability
	 * 
	 * @param e
	 */
	private void gaussianDistributionPrediction(Map<String, Number> e) {
		double x = getX();
		double y = getY();
		double robotDistance = e.get("distance").doubleValue();
		double robotVelocity = 8; // e.get("velocity").doubleValue();
		double angle = 0;
		double adjustment = 0;
		Point2D targPoint = new Point2D.Double(e.get("origX").doubleValue(), e
				.get("origY").doubleValue());

		if (e.get("energy").doubleValue() != 0.0) {
			double bullet = e.get("power").doubleValue();
			double absoluteBearing = e.get("absoluteBearingRadians")
					.doubleValue();
			Random guess = new Random(Instant.now().toEpochMilli());
			// nextGaussian gives results -3 to 3 (three standard deviations)
			// making 0 most likely result,
			// but robot is fairly unlikely to stand still. So, divide by 3 to
			// get results from -1 to 1,
			// offset by, oh, .25 (assumption: robot is going to move, but less
			// that maximum possible),
			// then flip positive/negative at random to model forward and
			// backward @ relative equality
			double secondGuess = ((guess.nextGaussian() / 3) + 0.25)
					* (guess.nextBoolean() ? 1 : -1);

			// calculate rough linear prediction targeting
			adjustment = Math.asin((robotVelocity
					* Math.sin(e.get("headingRadians").doubleValue()
							- absoluteBearing) / Rules.getBulletSpeed(bullet)));
			if (Double.isNaN(adjustment)) {
				out.println("Velocity " + robotVelocity);
				out.println("Heading(radians) " + e.get("headingRadians"));
				out.println("Absolute Bearing: " + absoluteBearing);
				out.println("Time to Target: " + Rules.getBulletSpeed(bullet));
				out.println("sin(heading-absolutebearing): "
						+ Math.sin(e.get("headingRadians").doubleValue()
								- absoluteBearing));
				out.println("All together: "
						+ (robotVelocity
								* Math.sin(e.get("headingRadians")
										.doubleValue() - absoluteBearing) / Rules
									.getBulletSpeed(bullet)));
			}
			adjustment = adjustment * secondGuess;
			// check for walls
			targPoint = checkWallCollision(new Point2D.Double(x, y),
					absoluteBearing + adjustment, robotDistance);
		}
		angle = enemy.get("absoluteBearingRadians").doubleValue();
		enemy.put("bearing", angle);
		enemy.put("adjustment", adjustment);
		enemy.put("x", new Integer((int) targPoint.getX()));
		enemy.put("y", new Integer((int) targPoint.getY()));

		if (enemy.get("energy").doubleValue() == 0.0) {
			enemy.put("score", 1000.0);
		} else {
			enemy.put("score", -(adjustment + angle / (PI / 4) + robotDistance));
		}
	}

	/**
	 * smartFire: score targets, determine best target, calculate how much gun,
	 * and fire.
	 * 
	 */
	private void smartFire() {
		// abort if heat is > 0, no energy or no scanned robots yet
		if (getEnergy() > 0 && getGunHeat() == 0 && scanned.size() > 0
				&& getGunTurnRemaining() == 0) {
			List<Map<String, Number>> victims = new ArrayList<Map<String, Number>>(
					scanned.keySet().size());
			scanned.entrySet()
					.stream()
					.filter(r -> (getTime() - r.getValue().get("time")
							.longValue()) < 10)
					.forEach(r -> victims.add(r.getValue()));
			if (victims.size() > 0) {
				Map<String, Number> victim = victims.stream().reduce(
						identity,
						(a, b) -> (a.get("score").doubleValue() > b
								.get("score").doubleValue()) ? a : b);
				gunActions.get(gunState).accept(victim);
			}
		}
	}

	private Point2D checkWallCollision(Point2D.Double origin, double bearing,
			double distance) {
		double targx = origin.getX() + Math.sin(bearing) * distance;
		if (targx < 18.0) {
			targx = 18.0;
		} else if (targx > (getBattleFieldWidth() - 18.0)) {
			targx = getBattleFieldWidth() - 18.0;
		}
		double targy = origin.getY() + Math.cos(bearing) * distance;
		if (targy < 18.0) {
			targy = 18.0;
		} else if (targy > (getBattleFieldHeight() - 18.0)) {
			targy = getBattleFieldHeight() - 18.0;
		}
		return new Point2D.Double(targx, targy);
	}

	/**
	 * inCorner: calculates whether or not current position is close enough to
	 * being in a corner.
	 * 
	 * @param p
	 *            - Point2D representation of current location
	 * @return - boolean indicating if we are in position approximately in a
	 *         corner
	 */
	private boolean inCorner(Point2D p) {
		RoundRectangle2D corner1;
		RoundRectangle2D corner2;
		RoundRectangle2D corner3;
		RoundRectangle2D corner4;
		corner1 = new RoundRectangle2D.Double(0, 0, 100, 100, 36, 36);
		corner2 = new RoundRectangle2D.Double(0,
				getBattleFieldWidth() - 100, 100, 100, 36, 36);
		corner3 = new RoundRectangle2D.Double(getBattleFieldHeight() - 100,
				getBattleFieldWidth() - 100, 100, 100, 36, 36);
		corner4 = new RoundRectangle2D.Double(getBattleFieldHeight() - 100,
				0, 100, 100, 36, 36);
		return corner1.contains(p) || corner2.contains(p)
				|| corner3.contains(p) || corner4.contains(p);
	}

	/**
	 * layCourse: combination method to turn toward and move to a point
	 * 
	 * @param origin - Point2D representation of the starting coordinates
	 * @param target - Point2D representation of the destination coordinates
	 */
	private void layCourse(Point2D origin, Point2D target) {
		double bearing = atan2(target.getX()-origin.getX(), target.getY()-origin.getY());
		turnToHeading(bearing, body);
		setAhead(origin.distance(target));
	}

	/**
	 * plotNearestCorner(Point2D): calculates nearest corner from
	 * passed-in location
	 * 
	 * @param p - current coordinates
	 * @return Point2D - coordinates of nearest corner
	 */
	private Point2D plotNearestCorner(Point2D p) {
		Point2D corner = new Point2D.Double((p.getX() < boundries.getCenterX()) ? boundries.getMinX() : boundries.getMaxX(), 
											(p.getY() < boundries.getCenterY()) ? boundries.getMinY() : boundries.getMaxY());
		return corner;
	}

	/**
	 * plotNextCorner(Point2D): calculates coordinates of next clockwise corner from
	 * passed-in location
	 * 
	 * @param p - current coordinates
	 * @return Point2D - coordinates of next clockwise corner
	 */
	private Point2D plotNextCorner(Point2D p) {
		Point2D corner = new Point2D.Double((p.getY() < boundries.getCenterY()) ? boundries.getMinX() : boundries.getMaxX(), 
											(p.getX() < boundries.getCenterX()) ? boundries.getMaxY() : boundries.getMinY());
		return corner;
	}

	/**
	 * plotPreviousCorner(Point2D): calculates direction and distance of
	 * previous corner in corners array from the passed-in location
	 * 
	 * @param p - current coordinates
	 * @return double array of polar coordinates for course (angle, distance)
	 */
	private Point2D plotPreviousCorner(Point2D p) {
		Point2D corner = new Point2D.Double((p.getY() < boundries.getCenterY()) ? boundries.getMaxX() : boundries.getMinX(), 
											(p.getX() < boundries.getCenterX()) ? boundries.getMinY() : boundries.getMaxY());
		return corner;
	}

	/**
	 * Constantly turning things right to match new bearings
	 * 
	 * @param theta
	 *            - bearing to match
	 * @param operator
	 *            - body part (body, gun, radar) to turn
	 */
	private void turnToHeading(double theta, Consumer<Double> operator) {
		operator.accept(theta);
	}

	private void initialize() {
		rectHeight = getBattleFieldHeight() - (2 * margin);
		rectWidth = getBattleFieldWidth() - (2 * margin);
		boundries = new Rectangle2D.Double(minX, minY, rectWidth, rectHeight);
		destination = new Point2D.Double(minX, minY);

		// Set colors
		setBodyColor(Color.black);
		setGunColor(Color.white);
		setRadarColor(Color.red);
		setBulletColor(Color.white);
		setScanColor(Color.white);

		// Initialize statistics
		stats = new HashMap<String, Integer>(6);
		stats.put("shots", 0);
		stats.put("hits", 0);
		stats.put("misses", 0);
		stats.put("bullets", 0);
		stats.put("written", 0);
		stats.put("round", getRoundNum());

		// Save # of other bots
		others = getOthers();
		// initialize map of potential targets
		scanned = new HashMap<String, Map<String, Number>>(others);
		// initialize "blank" identity map
		identity = new HashMap<String, Number>(1);
		identity.put("score", new Double(-50000.0));

		// decouple the gun from the body
		setAdjustGunForRobotTurn(true);
		// decouple radar from gun
		setAdjustRadarForGunTurn(true);
		
		addCustomEvent(new Condition("moveComplete") { 
			public boolean test() { 
				return getDistanceRemaining() == 0; 
			}
		});
		
		addCustomEvent(new Condition("turnComplete") {
			public boolean test() {
				return getTurnRemainingRadians() == 0;
			}
		});
		
		addCustomEvent(new Condition("sweepComplete") {
			public boolean test() {
				return getRadarTurnRemainingRadians() == 0;
			}
		});
	}

	private int initializeMovementStates() {
		if (movementStates == null) {
			movementStates = new ArrayList<Consumer<Point2D>>();
		}

		// 0 - rebasing to nearest corner
		movementStates.add(p -> {
			if (!inCorner(p)) {
				destination.setLocation(plotNearestCorner(p));
				layCourse(p, destination);
			} else {
				moveState = 4;
			}
		});
		// 1 - rebasing to next corner
		movementStates.add(p -> {
			if (!inCorner(p)) {
				destination.setLocation(plotNextCorner(p));
				layCourse(p, destination);
			} else {
				moveState = 4;
			}
		});
		// 2 - rebasing to previous corner
		// UNUSED
		movementStates.add(p -> {
			if (!inCorner(p)) {
				destination.setLocation(plotPreviousCorner(p));
				layCourse(p, destination);
			} else {
				moveState = 4;
			}
		});
		// 3 - traveling to selected base corner
		// UNUSED
		movementStates.add(p -> {
			out.println("WOAH. IN UNUSED MOVEMENT CASE 3.");
		});
		// 4 - dodging about in the corner
		movementStates.add(p -> {
			if (inCorner(p)) {
				if (getVelocity() >= 0 || Math.random() > .1) {
					double x = boundries.getMinX() + boundries.getCenterX()
							* Math.random();
					double y = boundries.getMinY() + boundries.getCenterY()
							* Math.random();
					destination.setLocation(x, y);
					layCourse(p, destination);
				}
			} else {
				moveState = 0;
			}
		});
		// 5 - stalking last enemy
		movementStates.add(p -> {
			if (enemy != null) {
/*				if (getTurnRemainingRadians() == 0 && getDistanceRemaining() == 0) {
					double bearing = enemy.get("originalBearing").doubleValue();
					if (enemy.get("distance").doubleValue() > 200) {
						bearing = bearing + PI/2 - (PI/8 * moveToggle);
					} else {
						bearing = bearing + PI/2;
					}
					setMaxVelocity(0);
					turnToHeading(bearing, body);
					setAhead(150);
				} else if (getTurnRemainingRadians() == 0) {
					setMaxVelocity(Rules.MAX_VELOCITY);
				} */
				destination.setLocation(enemy.get("origX").doubleValue(), enemy.get("origY").doubleValue());
				layCourse(p, destination);
			}
			if (radarState != 0) { radarState = 0; }
		});
		// 6 - recentering
		movementStates.add(p -> {
			if (!boundries.contains(p)) {
				layCourse(p, new Point2D.Double(boundries.getCenterX(), boundries.getCenterY()));
			} else if (others == 1) {
				moveState = 5;
			} else {
				moveState = 0;
			}
		});
		
		return 0;
	}
}
