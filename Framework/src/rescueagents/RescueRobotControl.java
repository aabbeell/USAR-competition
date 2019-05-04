package rescueagents;

import interfaces.CellInfo;
import interfaces.RobotPerception;
import rescueframework.AbstractRobotControl;
import rescueframework.Action;
import world.Robot;

import java.util.Arrays;

/**
 * RobotControl class to implement custom robot control strategies for rescue
 * robots. The main aim of rescue robots is to discover injured people and carry
 * them to the exit.
 */
public class RescueRobotControl extends AbstractRobotControl {
    private AMSService amsService;
    private RobotPerception internalWorldMap;
    private FlyingDroneControl fallbackExplore; // untill we find an injured TODO: rescuenál is
    public CellInfo targetCell = null;
    public Robot me;

    /**
     * Default constructor saving world robot object and perception interface
     *
     * @param robot      The robot object in the world
     * @param perception Robot perceptions
     */
    public RescueRobotControl(Robot robot, RobotPerception perception) {
        super(robot, perception);
        this.amsService = AMSService.getAMSService();
        amsService.resetMemory();
        me = robot;
        internalWorldMap = AMSService.getInternalMap();
        amsService.rescueList.add(this);
        fallbackExplore = new FlyingDroneControl(robot, perception);
        this.setRobotName("Rescue");
    }

    public boolean noMoreInjureds() {
        System.out.println("OFFOLJAM_E " + internalWorldMap.getDiscoveredInjureds().size());
        if (0 == internalWorldMap.getDiscoveredInjureds().size()) {
            if (amsService.rescuedInjuredList.size() != 0) {
                if (amsService.injuredList.size() == 0) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Custom step strategy of the robot, implement your robot control here!
     *
     * @return one of the following actions: <b>Action.STEP_UP</b> for step up,
     *         <b>Action.STEP_RIGHT</b> for step right, <b>Action.STEP_DOWN</b> for
     *         step down, <b>Action.STEP_LEFT</b> for step left
     *         <b>Action.PICK_UP</b> for pick up injured, <b>Action.PUT_DOWN</b> for
     *         put down injured, <b>Action.IDLE</b> for doing nothing.
     */
    @Override
    public Action step() {
        path = null;
        System.out.println("rescue--------------------------");
        System.out.println(Arrays.toString(internalWorldMap.getDiscoveredInjureds().toArray()));

        if (noMoreInjureds()) {
            return Action.IDLE;
        }

        // Rescue injured peopleNÁLA INJURED
        if (!robot.hasInjured()) {
            if (robot.getLocation().hasInjured()) { // TODO: if(targetcell = location of this rescue)

                AMSService.log(this, "picking up injured...");
                amsService.updateRescuedInjureds(robot.getLocation().getInjured(), true);

                return Action.PICK_UP;
            } else {
                // AMSService.log(this, "calculating shortest injured path...");
                // TODO: get the one with the lowest health!!!

                // path = internalWorldMap.getShortestInjuredPath(robot.getLocation());
                // if(path.getLastCell() != targetCell){
                // if(targetCell!=null){
                // amsService.updateRescuedInjureds(targetCell.getInjured(), false);
                // }
                // targetCell = path.getLastCell();
                // amsService.updateRescuedInjureds(targetCell.getInjured(), true);
                // }
                path =amsService.getShortestInjuredPath(me);

            }

            // HA VAN NÁLA INJURED
        } else {
            if (robot.getLocation().isExit()) {
                // AMSService.log(this, "putting down injured on exit cell");
                return Action.PUT_DOWN;
            } else {
                // AMSService.log(this, "calculating shortest exit path...");
                path = internalWorldMap.getShortestExitPath(robot.getLocation());
            }
        }

        // No path found - discover the whole map
        if (path == null) {
            // AMSService.log(this, "calculating shortest unknown path...");
            //fallbackExplore.step(); // TODO: replace this if drones go around the house
            path = internalWorldMap.getShortestUnknownPath(robot.getLocation());
        }

        if (path != null) {
            // Move the robot along the path
            // AMSService.log(this, "calculating next step along the path...");
            return amsService.moveRobotAlongPath(robot, path);
        } else {
            // If no path found - the robot stays in place and does nothing
            // AMSService.log(this, "no path found. Stopping.");
            return Action.IDLE;
        }
    }
}
