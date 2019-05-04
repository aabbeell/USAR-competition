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

public class FlyingDroneControl_v1 extends AbstractRobotControl {
    private AMSService amsService;
    private Map internalWorldMap;

    public Robot robot;
    public static int droneCount = 0;
    public int droneID = -1;
    public Area patrollingArea = null;



    public void CalculateArea()
    {

    }

    public FlyingDroneControl_v1(Robot robot2, RobotPerception perception)
    {
        //INIT
        super(robot2,perception);
        this.amsService = AMSService.getAMSService();
        internalWorldMap = AMSService.getInternalMap();

        this.setRobotName("Drone " + droneID);
        droneCount++;
        robot = robot2;

        this.patrollingArea = null;
    }

    @Override
    public Action step() {
        //AMSService.log(this,"" +amsService.getNumberOfDrones());
        //AMSService.log(this,"" + robot.getBatteryLifeTime());
        //System.out.println(amsService.getScore());
        if(patrollingArea==null)
        {
            setPatrollingArea();
        }

        Path path = Patrol();
        if(path!=null)
        {
            return amsService.moveRobotAlongPath(robot, path);
        }
        else
        {
            return Action.IDLE;
        }
    }

    public void setPatrollingArea()
    {
        int droneCount = amsService.setDroneIDs();
        droneID = robot.id;
        int xSize = internalWorldMap.getWidth();
        int ySize = internalWorldMap.getHeight();

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
            return BFS.search(robot.getLocation(),oldestCells,internalWorldMap.getWidth(),internalWorldMap.getHeight(), internalWorldMap);
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

        for (int i = patrollingArea.TL.x; i <= patrollingArea.TR.x; i++)
        {
            for(int j = patrollingArea.TL.y;j<= patrollingArea.BL.y;j++)
            {
                Cell cell = internalWorldMap.getCell(i,j);

                //If not reachable, throw away
                if(BFS.search(robot.getLocation(),cell,internalWorldMap.getWidth(),internalWorldMap.getHeight(), internalWorldMap)==null)
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
}
