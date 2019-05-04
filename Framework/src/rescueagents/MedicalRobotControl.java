/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package rescueagents;

import interfaces.CellInfo;
import interfaces.RobotPerception;
import rescueframework.AbstractRobotControl;
import rescueframework.Action;
import world.Path;
import world.Robot;

import java.util.ArrayList;

/**
 * RobotControl class to implement custom robot control strategies for medical
 * robots The main aim of medical robots is to discover injured people and keep
 * them alive until they are rescued.
 */
public class MedicalRobotControl extends AbstractRobotControl {
    RescueRobotControl fallbackRescue;
    FlyingDroneControl fallbackExplore; // untill we find an injured TODO: rescuenál is
    private AMSService amsService;
    public RobotPerception internalWorldMap;
    public Robot me;

    /**
     * Default constructor saving world robot object and perception interface
     *
     * @param robot      The robot object in the world
     * @param perception Robot perceptions
     */
    public MedicalRobotControl(Robot robot, RobotPerception perception) {
        super(robot, perception);
        me = robot;
        fallbackRescue = new RescueRobotControl(robot, perception);
        fallbackExplore = new FlyingDroneControl(robot, perception);
        this.amsService = AMSService.getAMSService();
        internalWorldMap = amsService.getInternalMap();
        amsService.resetMemory();
        this.setRobotName("Medic");
    }

    // HA NINCS TÖBB SEBESÜLT AKKOR OFFOLD MAGAD
    public boolean noMoreInjureds() {
        System.out.println("no more injured: medic " + internalWorldMap.getDiscoveredInjureds().size());
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
     *         put down injured, <b>Action.HEAL</b> for heal injured.
     *         <b>Action.IDLE</b> for doing nothing.
     */
    @Override
    public Action step() {

        amsService.updateMedic(this);
        amsService.updateInjureds(internalWorldMap.getDiscoveredInjureds()); // updates AMS with injuredList

        if (noMoreInjureds()) {
            return Action.IDLE;
        }

        // if no discovered injuried then help explore

        if ((amsService.getTimeStep() < 10) && amsService.rescueList.size() == 0) {
            System.out.println("nincs senki am");
            return fallbackExplore.step();
        }
        // if no medicine then convert to rescue
        if (robot.getMedicine() == 0) {
            System.out.println("nincs medicine am");
            return fallbackRescue.step();
        }

        CellInfo dest = amsService.calcDestination(this, true); // calculate the destionation
        path = amsService.pathToDest(robot.getLocation(), dest); // and path to dest

        rescueagents.AMSService.log(this, "destination:    " + dest);
        AMSService.log(this, "my location:    " + robot.getLocation());

        // heals if the patient need some
        if (robot.getLocation().getX() == dest.getX() && robot.getLocation().getY() == dest.getY()) {
            if(robot.getLocation().hasInjured()){
                AMSService.log(this, "HEEEAAAAAALLLLING");
                AMSService.log(this, "Medicine: " + robot.getMedicine());
                return Action.HEAL;
            }else {
                amsService.updateInjureds(internalWorldMap.getDiscoveredInjureds());
                dest = amsService.calcDestination(this, true);
                path = amsService.pathToDest(robot.getLocation(), dest); // and path to dest
            }
        }

        ArrayList<Integer> strList = amsService.getStrengths(this);
        int maxV = strList.get(amsService.getMaxIndex(strList));
        if (maxV < 100) {
            System.out.println("ink nem healelek");
            return fallbackRescue.step();
        }


        // Move the robot along the path and help the rescue if can
        if (path != null) {
            // Move the robot along the path and help the rescue if can

            boolean helpingmode = true;
            if (helpingmode) {

                // FELVESZI HA AZ EXIT FELE MEGY és éppen injureden áll és nincs nála injured

                boolean onInjured = robot.getLocation().hasInjured();
                boolean hasInjured = robot.hasInjured();
                boolean nextCellHasInjured = true;
                if(path.getNextCell(me.getLocation())!=null){
                    nextCellHasInjured = path.getNextCell(robot.getLocation()).hasInjured(); //TODO: NULL PTR EXEP-nincs path.getnextCell-nincs path-??
                }
                int pathLength = path.getLength();
                int distFromThis = internalWorldMap.getShortestExitPath(robot.getLocation()).getLength();

                int distFromNext;
                if (internalWorldMap.getShortestExitPath(path.getNextCell(robot.getLocation())) != null) {
                    distFromNext = internalWorldMap.getShortestExitPath(path.getNextCell(robot.getLocation()))
                            .getLength();
                    System.out.println("kövie mezőből nem talál pathet");
                } else {
                    distFromNext = distFromThis + 1;
                }

                // CellInfo next2 = path.getNextCell(path.getNextCell(robot.getLocation()));
                // int distFromNext2 = internalWorldMap.getShortestExitPath(next2).getLength();

                AMSService.log(this, "onInjured : " + onInjured);
                AMSService.log(this, "hasInjured : " + hasInjured);
                AMSService.log(this, "pathlenght: " + pathLength);
                AMSService.log(this, "distFromThis " + distFromThis);
                AMSService.log(this, "distFromNext " + distFromThis);
                // AMSService.log(this, "distFromNext2 " + distFromThis);
                AMSService.log(this, "nextCellHasInjured " + nextCellHasInjured);

                if (onInjured) {
                    System.out.println("on injured");
                    if (!hasInjured) {
                        System.out.println("no carried injured");
                        if (!nextCellHasInjured) {
                            System.out.println("nextcellclear");
                            if (distFromNext < distFromThis) {
                                System.out.println("distance is lower : PICK UP");
                                amsService.updateRescuedInjureds(robot.getLocation().getInjured(), true);
                                return Action.PICK_UP;
                            }
                        }

                    }
                }

                // HA VAN NÁLA VALAKI és az exittől távolodik
                if (hasInjured) {
                    System.out.println("carrying injured but ");
                    if (nextCellHasInjured || distFromThis < distFromNext) {
                        System.out.println("hasinjured or dist higher");
                        amsService.updateRescuedInjureds(robot.getLocation().getInjured(), false);
                        //return Action.PUT_DOWN;
                    }
                }

            }

            // más esetben simán megy az utján
            return amsService.moveRobotAlongPath(robot, path);

        } else {
            // If no path found - the robot stays in place and does nothing
            AMSService.log(this, "no path found. Stopping.");
            return Action.IDLE;
        }

    }
}
