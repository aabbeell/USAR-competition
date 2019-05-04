package rescueagents;

import interfaces.CellInfo;
import interfaces.InjuredInfo;
import interfaces.RobotInterface;
import rescueframework.AbstractRobotControl;
import rescueframework.Action;
import rescueframework.RescueFramework;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.regex.Pattern;
import world.*;
import java.util.Arrays;

import java.util.*;
import java.util.regex.Pattern;
import world.*;
import world.Map;

import java.util.logging.Logger;

/**
 * Agent Management Service (AMS) It provides an internal world model that is
 * automatically updated by agent perceptions. It may also provide other
 * services for agents (logging, routing etc.).
 */
public class AMSService {

    // Hiperparameter
    public static int CELL_INVALID_TIME = 200;
    public static int param1 = 333; // this must be scaled with the mapsize
    public static int param2 = 13;
    public static int param3 = 4;
    public static int param4 = 179; // this must be scaled with the mapsize
    public static int param5 = 3;
    public static int expDist = 12;
    public static int expAvg = 20;

    static AMSService amsService;
    public static Map internalWorldModel;
    public static HashMap<Cell, Integer> lastVisitedTime;

    public Integer timeStep = 0;
    public List<InjuredInfo> injuredList = new ArrayList<InjuredInfo>(); // TODO: delete unreachable injureds
    public List<Integer> injuredHealthList = new ArrayList<Integer>();
    public List<Boolean> tagged4RescueList = new ArrayList<Boolean>();

    public HashMap<InjuredInfo, Integer> injuredHealthMap = new HashMap<InjuredInfo, Integer>();
    public List<Integer> next = new ArrayList<Integer>();

    public List<InjuredInfo> rescuedInjuredList = new ArrayList<InjuredInfo>();

    public List<MedicalRobotControl> medicList = new ArrayList<MedicalRobotControl>();
    public List<RescueRobotControl> rescueList = new ArrayList<RescueRobotControl>();

    public static int lastCalledOncePerStep = -1;

    public AMSService() {
        resetMemory();
    }

    public void resetMemory() {

        // RESET INTERNAL STATE WHEN NEW THING HAPPEN

        injuredList = new ArrayList<InjuredInfo>();
        rescuedInjuredList = new ArrayList<InjuredInfo>();
        medicList = new ArrayList<MedicalRobotControl>();
        rescueList = new ArrayList<RescueRobotControl>();
        injuredHealthList = new ArrayList<Integer>();
        tagged4RescueList = new ArrayList<Boolean>();
        lastCalledOncePerStep = -1;
        injuredList = new ArrayList<InjuredInfo>();
        injuredHealthMap = new HashMap<InjuredInfo, Integer>();
        rescuedInjuredList = new ArrayList<InjuredInfo>();
        medicList = new ArrayList<MedicalRobotControl>();
        rescueList = new ArrayList<RescueRobotControl>();

    }

    public static AMSService getAMSService() {
        if (amsService == null) {
            amsService = new AMSService();
        }
        return amsService;
    }

    // here goes everything that needs to be called exactly once in a step
    public void oncePerStep() {
        if (lastCalledOncePerStep < getTimeStep()) {
            // HERE
            updateInjuredHealthMap();

            lastCalledOncePerStep = getTimeStep();
        }
    }

    /**
     * Get the actual internal world model
     * 
     * @return the internal world model
     */
    public static Map getInternalMap() {
        return internalWorldModel;
    }

    /**
     * Initialize the internal world model. This is called by the framework.
     * 
     * @param initialWorldMap the Map object created during initialization
     */
    public static void setInternalWorldModel(Map initialWorldMap) {
        internalWorldModel = initialWorldMap;

        // Init lastVisitedMap
        lastVisitedTime = new HashMap<Cell, Integer>();
        for (int i = 0; i < internalWorldModel.cells.length; i++) {
            for (int j = 0; j < internalWorldModel.cells[i].length; j++) {
                lastVisitedTime.put(initialWorldMap.cells[i][j], Integer.MIN_VALUE);
            }
        }
    }

    /**
     * Log a message with timestamp to the console
     * 
     * @param message The message to log to the console
     */
    public static void log(AbstractRobotControl control, String message) {
        RescueFramework.log(control.getRobotName() + " " + message);
    }

    public static void log(RobotInterface rob, String message) {
        RescueFramework.log(rob.getName() + " " + message);
    }

    /*****************************************************
     * Common robot control functions can be placed here.
     *****************************************************/

    /**
     * Calculate the next step along a path
     * 
     * @param path
     * @return move action to be performed
     */
    public Action moveRobotAlongPath(RobotInterface robot, Path path) {

        CellInfo nextCell = null;
        if (path.getNextCell(robot.getLocation()) != null) {
            nextCell = path.getNextCell(robot.getLocation());
        }

        if (nextCell == null || nextCell.hasObstacle() || nextCell.hasRobot()) {

            log(robot, "The road is blocked. Stopping.");

            return Action.IDLE;
        } else {
            return nextCell.directionFrom(robot.getLocation());
        }

    }

    public Action moveRobotAlongPathDrone(RobotInterface robot, Path path) {

        CellInfo nextCell = null;
        if (path.getNextCell(robot.getLocation()) != null) {
            nextCell = path.getNextCell(robot.getLocation());
        }

        if (nextCell == null) {
            return Action.IDLE;
        } else {
            return nextCell.directionFrom(robot.getLocation());
        }

    }

    public Path getShortestPath(Cell start, Cell end) {
        List<Cell> list = new ArrayList<>();
        list.add(end);
        return internalWorldModel.getShortestPath(start, list, true);
    }

    public Path getShortestPath(int startX, int startY, int endX, int endY) {
        Cell start = internalWorldModel.getCell(startX, startY);
        Cell end = internalWorldModel.getCell(endX, endY);
        List<Cell> list = new ArrayList<>();
        list.add(end);
        return internalWorldModel.getShortestPath(start, list, true);
    }

    // sets droneID in control, returns with drone count
    public int setDroneIDs(FlyingDroneControl fdc) {
        List<Robot> robots = internalWorldModel.getRobots();
        List<Robot> drones = new ArrayList<Robot>();
        for (int i = 0; i < robots.size(); i++) {
            if (robots.get(i).getType() == Robot.Type.DRONE) {
                drones.add(robots.get(i));
            }
        }

        for (int i = 0; i < drones.size(); i++) {
            if (fdc.robot.id == drones.get(i).id) {
                fdc.droneID = i;
            }
        }

        return drones.size();
    }

    public ArrayList<Cell> getInjuredCells(Robot me){
        ArrayList<Cell> injuredCells = new ArrayList<Cell>();

        for(InjuredInfo injured : injuredList){
            if(!injured.getLocation().hasRobot()){
                injuredCells.add((Cell) injured.getLocation());
            }
        }
        return injuredCells;
    }

    public Path getShortestInjuredPath(Robot me ){

        //return internalWorldModel.getShortestPath(me.getLocation(), getInjuredCells(me), true);//        Path best = null;
        float bestScore = 100*200;
        Path best = null;

        for(Cell c : getInjuredCells(me)){
            Path path = getShortestPath(me.getLocation(), c);
            int length;

            if(path != null){
                length = path.getLength();
            }else {
                length = -1;
            }
            float hp = 0;
            if (c.getInjured() != null) {
                 hp = c.getInjured().getHealthRatio();
            }

            if(hp*10 - (length*2) < 0){
                hp=101;
            }

            if(length != -1){
                if(hp*length < bestScore){
                    bestScore = hp*length;
                    best = path;
                }
            }
        }

        return best;
    }

        //return internalWorldModel.getShortestPath(me.getLocation(), getInjuredCells(me), true);


    public int getTimeStep() {
        String s = internalWorldModel.getTotalScore();
        String[] splitted = s.split(Pattern.quote("|"));
        String[] time = splitted[0].trim().split(" ");
        String time2 = time[1];
        return Integer.parseInt(time2);
    }

    // Starts from a mystical value depending number of agents
    public int getScore() {
        String s = internalWorldModel.getTotalScore();
        String[] splitted = s.split(Pattern.quote("|"));
        String[] score = splitted[4].trim().split(" ");
        String score2 = score[1];
        return Integer.parseInt(score2);
    }

    // Sets all visible cells' times to current time
    public void RefreshCellTimes() {
        for (int i = 0; i < internalWorldModel.cells.length; i++) {
            for (int j = 0; j < internalWorldModel.cells[i].length; j++) {
                Cell c = internalWorldModel.cells[i][j];
                if (c.robotSeesIt()) {
                    lastVisitedTime.put(c, getTimeStep());
                }
            }
        }
    }

    public void updateInjureds(List<InjuredInfo> i) {
        injuredList = i;
        //
        for (InjuredInfo injured : i) {

            // //mission: undiscoveredpath
            // if(medicList.get(1).internalWorldMap.getShortestInjuredPath(medicList.get(1).me.getLocation())
            // == null){
            // injuredList.remove(injured);
            // }
            // System.out.println(injured.getHealth());
            //
            // //
            if (rescuedInjuredList.contains(injured)) {
                injuredList.remove(injured);
            }
        }

        oncePerStep();
    }

    public void updateRescuedInjureds(InjuredInfo injured, boolean add) {
        if (add) {
            rescuedInjuredList.add(injured);
        } else {
            rescuedInjuredList.remove(injured);
            injuredList.add(injured);
        }
    }

    public void updateMedic(MedicalRobotControl me) {
        if (!medicList.contains(me)) {
            medicList.add(me);
        }
    }

    /*
     * public ArrayList<ArrayList<Integer>> getDistanceFromMtoP(){
     * 
     * ArrayList<ArrayList<Integer>> MtoP = new ArrayList<ArrayList<Integer>>();
     * 
     * for (InjuredInfo injured : injuredList) { // TODO : check if already saved
     * 
     * if (!injured.isSaved() && injured.isAlive()){ //lehet h rossz a feltétel
     * ArrayList distFromI = new ArrayList<Integer>(); int life =
     * injured.getHealth();
     * 
     * for (MedicalRobotControl medic : medicList){ if(medic.me.hasMedicine()){ int
     * distToMedic =
     * medic.internalWorldMap.getShortestInjuredPath(medic.me.getLocation(), life,
     * life).getLength(); distFromI.add(distToMedic);
     * //System.out.println("getDistanceFromMtoP :  Medic " + medic.toString() );
     * //System.out.println("getDistanceFromMtoP :  distToMedic " + distToMedic); }
     * } MtoP.add(distFromI); }
     * 
     * }
     * 
     * System.out.println("getDistanceFromMtoP: MtoP: " +
     * Arrays.toString(MtoP.toArray()));
     * 
     * return MtoP; }
     * 
     */

    public ArrayList<Integer> getStrengths(MedicalRobotControl thisMedic) {

        // ArrayList<ArrayList<Integer>> distPM = getDistanceFromMtoP();
        ArrayList<Integer> strList = new ArrayList<Integer>();

        for (int ii = 0; ii < injuredList.size(); ii++) {
            double strength = calcStrength(injuredList.get(ii), thisMedic);
            strList.add((int) strength);

        }
        System.out.println("getStrength: strlist: " + Arrays.toString(strList.toArray()));
        return strList;
    }

    public double calcStrength(InjuredInfo injured, MedicalRobotControl thisMedic) {

        int life = injured.getHealth();
        double urgency = 0;
        double corrigatedDist;
        double avgDist = 1;
        int dist;
        int medicCount = 0;

        double param1d = param1; // this must be scaled with the mapsize
        double param2d = param2 / 10.;
        double param3d = param3;
        double param4d = param4; // this must be scaled with the mapsize
        double param5d = param5;
        double expDistd = expDist / 10.;
        double expAvgd = expAvg / 10.;

        // CALCULATING DISTANCE FROM THIS INJURED TO THIS MEDIC

        CellInfo medicloc = thisMedic.me.getLocation();

        Path path = thisMedic.internalWorldMap.getShortestInjuredPath(medicloc, life, life);

        if (path == null) {
            return 0;
        } else {
            dist = path.getLength();
        }

        // CALCULATING avg DISTANCE FROM THIS INJURED TO OTHER MEDICS
        for (MedicalRobotControl i : medicList) {
            path = i.internalWorldMap.getShortestInjuredPath(i.me.getLocation(), life, life);
            int distanceMtoI = 0;

            if (path != null) { // ha nem elérhető az adott medicből az injured
                distanceMtoI = path.getLength();

            } else {
                distanceMtoI = dist * 2; // TODO: EMIATT LEHET H HÜLYÉN VISELKEDIK(Odavissza mozog ha 2mezőn ha az
                                         // egyiken blokkolja a másik útját)
            }
            avgDist += distanceMtoI;
            medicCount++;
        }

        avgDist -= dist;
        avgDist /= medicCount; // ne osszunk nullával

        // TODO: a corrigated dist még minding nem mükszik
        corrigatedDist = Math.pow(avgDist, expAvgd) / Math.pow(dist, expDistd); // corrigating distance with avgDist -»
                                                                                // the bigger the difference(between
                                                                                // acgDist and Dist) the more it matters

        // CLIPPING HEALTH (below param1 hp it is MORE**2 surgent to save him)
        if (life > param1d) { // param 1
            return ((1000. - life) * corrigatedDist / 2);
        } else {
            urgency = 500 + Math.pow(500 - life, param2d); // param 2
        }

        // CLIPPING DISTANCE + HP (if hp lower than param4 and dist lower than param3 -»
        // [param4] more likely to get healed //TODO: maybe ha felveszuk akkor hp-től
        // függetlenül érdemes átmenni rajta
        if (dist < param3d && life < param4d) { // param 3 and param4
            urgency = Math.pow(urgency, param5d);
        }

        // if the avg distance to other medics is smaller then dont go there
        double strength = corrigatedDist * urgency; // TODO: problem with corDist scaling: should be big when the other
                                                    // medics are close and small when distant

        // ZERO IT OUT IF SAVED OR cant be saved(dead or low hp)
        if (life < dist || injured.isSaved()) {
            strength = 0;
        }

        // TODO: gyogyitás mértékébe számoljuk bele a medicinánk mennyiségét

        System.out.println("calcStr: mylocation" + thisMedic.me.getLocation());
        System.out.println("calcStr: injured location" + injured.getLocation());
        System.out.println("calcStr: medic:count" + medicList.size());

        System.out.print(" +hp:" + injured.getHealth());
        System.out.print(" =urgency:  " + urgency);
        System.out.print(" *(dist:" + dist);
        System.out.print(" *avgDist:" + avgDist);
        System.out.print(" )=corDist:" + corrigatedDist);
        System.out.print(" =STR:  " + strength);
        System.out.println();
        System.out.println();

        return strength;
    }

    public int getMaxIndex(ArrayList<Integer> strengths) {
        InjuredInfo strongest;

        int max = 0;
        int maxIndex = 0;

        for (int i = 0; i < strengths.size(); i++) {
            if (max < strengths.get(i)) {
                max = strengths.get(i);
                maxIndex = i;
            }
        }

        return maxIndex;
    }

    public CellInfo calcDestination(MedicalRobotControl thisMedic, boolean medoid) {

        ArrayList<Integer> strList = getStrengths(thisMedic);

        if (!medoid) {
            int clusterX = 0;
            int clusterY = 0;
            int sumS = 0;

            for (int ii = 0; ii < strList.size(); ii++) {
                sumS += strList.get(ii);
                clusterY += injuredList.get(ii).getLocation().getY() * strList.get(ii); // INJUREDS : MUST BE ONLY ALIVE
                                                                                        // / NOT SAVED ones
                clusterX += injuredList.get(ii).getLocation().getX() * strList.get(ii);

            }
            if (sumS != 0) {
                clusterX /= sumS;
                clusterY /= sumS;
            }

            System.out.println("calcDestination: dest " + clusterX + " : " + clusterY);

            return internalWorldModel.getCell(Math.round(clusterX), Math.round(clusterY)); // RETURNS THE CLUSTER CENTER

        }
        return injuredList.get(getMaxIndex(strList)).getLocation(); // RETURNS THE LOCATION OF THE PRIOR1

    }

    public Path pathToDest(CellInfo start, CellInfo dest) {
        ArrayList<Cell> destList = new ArrayList<>();
        destList.add((Cell) dest);
        return internalWorldModel.getShortestPath(start, destList, false);
    }

    // Call it once per step
    public void updateInjuredHealthMap() {
        // Everyone loses a life point (have to be here, before adding new)
        for (InjuredInfo injured : injuredHealthMap.keySet()) {
            injuredHealthMap.replace(injured,
                    injuredHealthMap.get(injured) <= 0 ? 0 : injuredHealthMap.get(injured) - 1); // LOL

        }

        // Add new
        for (InjuredInfo injured : injuredList) {
            if (!containsInjured(injuredHealthMap.keySet(), injured)) {
                injuredHealthMap.put(injured, injured.getHealth());
            }
        }

        // Delete
        injuredHealthMap.keySet().removeIf(injured -> !containsInjured(injuredList, injured));
    }

    private boolean containsInjured(Collection<InjuredInfo> set, InjuredInfo injured) {
        for (InjuredInfo i : set) {
            if (((Injured) i).id == ((Injured) injured).id)
                return true;
        }
        return false;
    }
}