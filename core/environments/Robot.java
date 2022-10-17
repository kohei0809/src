package core.environments;

import core.IGraph;
import core.RobotSpec;

public class Robot {

	static int _id;

	IGraph map;
	int position;
	RobotSpec spec;

	private int id, batteryWattage;
	private int consumedEnergy = 0;
	private RobotStates state;
	private Battery battery;
	private Cleaner cleaner;

	public Robot(RobotSpec spec) {
		id = _id++;
		battery = new Battery(spec.getCapacity());
		batteryWattage = spec.getWattage();
		cleaner = new Cleaner();
		state = RobotStates.inactive;
		this.spec = spec;
	}

	public Robot(RobotSpec spec, int id) {
		this.id = id;
		battery = new Battery(spec.getCapacity());
		batteryWattage = spec.getWattage();
		cleaner = new Cleaner();
		state = RobotStates.inactive;
		this.spec = spec;
	}

	// Robot move methods
	public void move(int dest) {
		if (state != RobotStates.active)
			throw new IllegalStateException("Robot State is not Active. Current state: " + state);

		if (dest == position)
			return;

		if (map.getChildNodes(position).contains(dest)) {
			if (getBatteryLevel() == 0) {
				return;
			}

			/*int px = position / 5 - 2;
			int py = position % 5 - 2;
			int dx = dest / 5 - 2;
			int dy = dest % 5 - 2;
			
			System.out.println("position=" + position + ", dest=" + dest);
			System.out.println("(" + px + "," + py + ") => (" + dx + "," + dy + ")");*/

			position = dest;
			battery.discharge(batteryWattage);
			consumedEnergy = batteryWattage;
		} else
			throw new IllegalArgumentException(
					"Cannot move ID[" + id + "]: Node(" + position + ") -> Node(" + dest + ")");
	}

	public void clean() {

	}

	// Robot state methods
	public void activate() {
		if (state != RobotStates.inactive)
			throw new IllegalStateException("Robot state is not Inactive.");

		if (map == null)
			throw new IllegalStateException("Field Data is null.");

		state = RobotStates.active;
	}

	public void connectBase() {
		state = RobotStates.docking;
	}

	public void disconnectBase() {
		state = RobotStates.active;
	}

	// Initializing methods
	public void setFieldStructure(IGraph map) {
		if (state != RobotStates.inactive)
			throw new IllegalStateException("Robot state is not Inactive.");

		this.map = map;
	}

	// properties
	public void setID(int id) {
		this.id = id;
	}

	public void setPosition(int value) {
		if (state != RobotStates.inactive)
			throw new IllegalStateException("Robot State is not Inactive");

		position = value;
	}

	public void setRobotState(RobotStates s) {
		state = s;
	}

	public void setBattery(Battery b) {
		battery = b;
	}

	public void setBatteryWattage(int w) {
		batteryWattage = w;
	}

	public void setCleaner(Cleaner c) {
		cleaner = c;
	}

	public void setRobotSpec(RobotSpec s) {
		spec = s;
	}

	public int getID() {
		return id;
	}

	public int getPosition() {
		return position;
	}

	public RobotStates getRobotState() {
		return state;
	}

	public Battery getBattery() {
		return battery;
	}

	public int getBatteryLevel() {
		return battery.getLevel();
	}

	public int getBatteryWattage() {
		return batteryWattage;
	}

	public Cleaner getCleaner() {
		return cleaner;
	}

	public int getStepVacuumedLitter() { // step amount
		return cleaner.getLitter();
	}

	public int getAccumulatedLitter() { // accumulate amount
		return cleaner.getAccumulatedLitter();
	}

	public RobotSpec getRobotSpec() {
		return spec;
	}

	public int getEnergyCost() {
		int value = consumedEnergy;
		consumedEnergy = 0;
		return value;
	}

}
