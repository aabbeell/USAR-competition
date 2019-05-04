package rescueagents;

import world.Cell;
import world.Map;
import world.Path;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

//AStar does not handle undiscovered cells, so here we go
public class BFS {
    private static boolean containsCell(List<Cell> queue, Cell c)
    {
        for (int i = 0; i < queue.size(); i++)
        {
            if(queue.get(i).getX()== c.getX() && queue.get(i).getY() == c.getY())
            {
                return true;
            }
        }

        return false;
    }

    public static Path search(Cell start, Cell target, int xmax,int ymax, Map map)
    {
        List<Cell> al = new ArrayList<Cell>();
        al.add(target);
        return search(start,al,xmax,ymax, map);
    }

    public static Path search(Cell start, List<Cell> targets, int xmax, int ymax, Map map)
    {
        ArrayList<Cell> visited = new ArrayList<Cell>();
        visited.add(start);
        ArrayList<Cell> queue = new ArrayList<Cell>();
        queue.add(start);

        HashMap<Cell,Cell> parent = new HashMap<Cell,Cell>();
        parent.put(start,null);

        while((cellInList(targets,visited))==null && !queue.isEmpty())
        {
            Cell c = queue.get(0);
            Cell[] neighbours = new Cell[4];


            if(true)
            {
                if(c.getX()>0) neighbours[0] =  map.getCell(c.getX()-1,c.getY());
                if(c.getY()>0) neighbours[1] = map.getCell(c.getX()+1,c.getY());
                if(c.getX()<xmax-1) neighbours[2] = map.getCell(c.getX(),c.getY()-1);
                if(c.getY()<ymax-1) neighbours[3] = map.getCell(c.getX(),c.getY()+1);
            }

            for (int i = 0; i < neighbours.length; i++)
            {
                if(neighbours[i]!=null && !containsCell(visited,neighbours[i]) && isValidNeighbour(c,neighbours[i]))
                {
                    queue.add(neighbours[i]);
                    visited.add(neighbours[i]);
                    parent.put(neighbours[i],c);
                }
            }

            queue.remove(0);
        }

        //Route not exists
        if(cellInList(targets,visited)==null)
        {
            return null;
        }

        //Construct path
        ArrayList<Cell> reversepath = new ArrayList<Cell>();
        reversepath.add(cellInList(targets,visited));

        Cell parentcell = parent.get(reversepath.get(0));
        while(parentcell!=null)
        {
            reversepath.add(parentcell);
            parentcell = parent.get(parentcell);
        }

        Path path = new Path();

        for (int i = reversepath.size()-1; i >= 0; i--)
        {
            path.addLastCell(reversepath.get(i));
        }

        return path;
    }

    private static Cell cellInList(List<Cell> targets,List<Cell> queue)
    {
        for (int i = 0; i < queue.size(); i++)
        {
            if(containsCell(targets,queue.get(i)))
            {
                return queue.get(i);
            }
        }

        return null;
    }

    private static boolean isValidNeighbour(Cell actual, Cell neighbour)
    {
        if(neighbour.hasObstacle())
        {
            return false;
        }

        if(neighbour.getX()==actual.getX()+1 && actual.hasWall(1))
        {
            return false;
        }
        if(neighbour.getX()==actual.getX()-1 && actual.hasWall(3))
        {
            return false;
        }
        if(neighbour.getY()==actual.getY()+1 && actual.hasWall(2))
        {
            return false;
        }
        if(neighbour.getY()==actual.getY()-1 && actual.hasWall(0))
        {
            return false;
        }

        return true;
    }
}
