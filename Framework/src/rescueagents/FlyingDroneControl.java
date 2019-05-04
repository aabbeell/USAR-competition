package rescueagents;

import interfaces.RobotPerception;
import rescueframework.AbstractRobotControl;
import rescueframework.Action;
import world.*;
import world.Robot;

import java.util.ArrayList;
import java.util.List;

class Point
{
    public int x;
    public int y;

    public Point(int nx, int ny)
    {
        x = nx;
        y = ny;
    }
}

//Struct for a rectangle area
class Area
{
    public Point BL; //Bottom-left
    public Point TR; //etc.
    public Point BR;
    public Point TL;
    public Point Midpoint;

    public Area(Point nTL, Point nBR)
    {
        TL = nTL;
        BR = nBR;
        BL = new Point(TL.x,BR.y);
        TR = new Point(BR.x,TL.y);
        Midpoint = new Point((BR.x+TL.x)/2,(BR.y+TL.y)/2);
    }
}

public class FlyingDroneControl extends AbstractRobotControl {
    private AMSService amsService;
    private Map internalWorldMap;

    public Robot robot;
    public static int droneCount = 0;
    public int droneID = -1;
    public Area patrollingArea = null;


    public FlyingDroneControl(Robot robot2, RobotPerception perception)
    {
        //INIT
        super(robot2,perception);
        this.amsService = AMSService.getAMSService();
        internalWorldMap = AMSService.getInternalMap();

        this.setRobotName("Drone " + droneID);
        robot = robot2;

        this.patrollingArea = null;
    }

    @Override
    public Action step() {
        if(droneID==-1)
        {
            amsService.setDroneIDs(this);
        }
        if(patrollingArea==null)
        {
            setPatrollingArea();
        }

        //If there are no injured left in the end, stop
        if(amsService.getTimeStep()>200 && amsService.injuredList.size()==0)
        {
            return Action.IDLE;
        }


        Path path = Patrol();
        if(path!=null)
        {
            return amsService.moveRobotAlongPathDrone(robot, path);
        }
        else
        {
            return Action.IDLE;
        }
    }

    public void setPatrollingArea()
    {
        int droneCount = amsService.setDroneIDs(this);
        int xSize = internalWorldMap.getWidth();
        int ySize = internalWorldMap.getHeight();

        //If it has -1 as ID (for example a medic working as a drone), it gets all the field
        if(droneID==-1)
        {
            int minY = 0;
            int maxY = ySize-1;
            int minX = 0;
            int maxX = xSize-1;

            patrollingArea = new Area(new Point(minX,minY),new Point(maxX,maxY));
            return;
        }

        if(xSize>ySize)
        {
            int minY = 0;
            int maxY = ySize-1;
            int minX = (droneID*xSize)/droneCount;
            int maxX = (((droneID+1)*xSize)/droneCount)-1;

            patrollingArea = new Area(new Point(minX,minY),new Point(maxX,maxY));
        }
        else
        {
            int minX = 0;
            int maxX = xSize-1;
            int minY = (droneID*ySize)/droneCount;
            int maxY = (((droneID+1)*ySize)/droneCount)-1;

            patrollingArea = new Area(new Point(minX,minY),new Point(maxX,maxY));
        }
    }

    public Path Patrol()
    {
        List<Cell> oldestCells = getOldestCells();
        if(oldestCells.size()>0)
        {
            return BFS.search(robot.getLocation(),oldestCells,internalWorldMap);
        }
        else
        {
            return null;
        }
    }

    private List<Cell> getOldestCells()
    {
        amsService.RefreshCellTimes();
        int oldestValue = amsService.getTimeStep() - AMSService.CELL_INVALID_TIME;
        List<Cell> oldestCells = new ArrayList<Cell>();
        
        //We only care about our area
        List<Cell> reachables = BFS.getReachables(robot.getLocation(),internalWorldMap);

        for (int i = patrollingArea.TL.x; i <= patrollingArea.TR.x; i++)
        {
            for(int j = patrollingArea.TL.y;j<= patrollingArea.BL.y;j++)
            {
                Cell cell = internalWorldMap.getCell(i,j);

                //If not reachable, throw away
                if(!isReachablesHasCell(reachables,cell))
                {
                    continue;
                }

                Integer value = AMSService.lastVisitedTime.get(cell);

                if(value<oldestValue)
                {
                    oldestCells.clear();
                    oldestCells.add(cell);
                    oldestValue = value;
                }
                else if(value == oldestValue)
                {
                    oldestCells.add(cell);
                }
            }
        }
        
        return oldestCells;
    }
    
    boolean isReachablesHasCell(List<Cell> reachables, Cell c)
    {
        for (Cell reachable : reachables) {
            if (reachable.getX() == c.getX() && reachable.getY() == c.getY()) {
                return true;
            }
        }
        return false;
    }
}
