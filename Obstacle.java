package curvature_total;

import java.util.ArrayList;

public class Obstacle
{
    int id;
    public ArrayList<point2> vertices;
    public ArrayList<point2> originalVertices;//replace the filled version after all the vertices are discovered to help performance
    point2 interiorPoint; //used for sight blocking or normal vector side determination

    double smallestX;
    double smallestY;
    double largestX; //Usually used for scanning, give a sense of the the size of the shape
    double largestY;
    
    public Obstacle()
    {
        vertices = new ArrayList<point2>();
    }

    //Assuming the obstacles are all convex
	void updateInteriorPoint()
	{
	    point2 sum = new point2(0, 0);
	
	    for (int i = 0; i < vertices.size(); i++)
	    {
	        sum = point2.plus(sum, vertices.get(i));
	    }
	    interiorPoint = point2.divide(sum, (double) vertices.size());
	}

    final boolean LineOfSight(point2 p1, point2 p2)
    {
        //bounding box testing;
        if (p1.x > largestX && p2.x > largestX)
        {
            return true;
        }
        if (p1.y > largestY && p2.y > largestY)
        {
            return true;
        }
        if (p1.x < smallestX && p2.x < smallestX)
        {
            return true;
        }
        if (p1.y < smallestY && p2.y < smallestY)
        {
            return true;
        }

        if (originalVertices == null || originalVertices.size() == 0)
        { //It's empty, so it can block nothing.
             return true;
        }
        else
        {
            point2 previousPoint = originalVertices.get(originalVertices.size() - 1);
            for (int i = 0; i < originalVertices.size(); i++)
            {
                if (HasIntersection(p1, p2, originalVertices.get(i), previousPoint))
                {
                    return false;
                }
                else
                {
                    previousPoint = originalVertices.get(i);
                }
            }

            return true;
        }
    }

    boolean MapBuildingLineOfSight(point2 p1, point2 p2)
    {
    	  //bounding box testing;
        if (p1.x > largestX && p2.x > largestX)
        {
            return true;
        }
        if (p1.y > largestY && p2.y > largestY)
        {
            return true;
        }
        if (p1.x < smallestX && p2.x < smallestX)
        {
            return true;
        }
        if (p1.y < smallestY && p2.y < smallestY)
        {
            return true;
        }

          if (vertices.size() <= 1 )
          { //It's empty or not discovered, so it can not block anything.
               return true;
          }
          else
        {
            point2 previousPoint = vertices.get(vertices.size() - 1);
            for (int i = 0; i < vertices.size(); i++)
            {
                if ( HasIntersection(p1, p2, vertices.get(i), previousPoint))
                {
                    return false;
                }
                else
                {
                    previousPoint = vertices.get(i);
                }
            }

            return true;
        }
    }
    
    //Overloaded P2 TODO: not the best implementation
    //"direction" is used to return a value actually
    //used only in Robot's stateupdate routine for collision avoidance and finding a sliding direction
    boolean LineOfSight(point2 p1, point2 p2, point2 direction)
    {
        //bounding box testing; seems to be a cut the cost to 60%
        if (p1.x > largestX && p2.x > largestX)
        {
            return true;
        }
        if (p1.y > largestY && p2.y > largestY)
        {
            return true;
        }
        if (p1.x < smallestX && p2.x < smallestX)
        {
            return true;
        }
        if (p1.y < smallestY && p2.y < smallestY)
        {
            return true;
        }

         if (vertices.size() == 0)
         { //It's empty, so it can block nothing.
            return true;
         }
         else
        {       
        	//TODO: a line can actually intersect with an obstacle in two places. We should return the closest direction
        	//instead of the first one detected.
            point2 previousPoint = vertices.get(vertices.size() - 1);
            for (int i = 0; i < vertices.size(); i++)
            {
                if (HasIntersection(p1, p2, vertices.get(i), previousPoint))
                {
                    //direction = point2.minus(vertices.get(i), previousPoint);
                    direction.x = point2.minus(vertices.get(i), previousPoint).x;
                    direction.y = point2.minus(vertices.get(i), previousPoint).y;
                    return false;
                }
                else
                {
                    previousPoint = vertices.get(i);
                }
            }

            return true;
        }
    }

    void updateBoundingBox()
    {
        smallestX = largestX = vertices.get(0).x;
        smallestY = largestY = vertices.get(0).y;
        for (int i = 1; i < vertices.size(); i++)
        {
            if (vertices.get(i).x < smallestX)
            {
                smallestX = vertices.get(i).x;
            }
            if (vertices.get(i).y < smallestY)
            {
                smallestY = vertices.get(i).y;
            }
            if (vertices.get(i).y > largestY)
            {
                largestY = vertices.get(i).y;
            }
            if (vertices.get(i).x > largestX)
            {
                largestX = vertices.get(i).x;
            }

        }

    }

    //TODO: simplify it. currently use with caution
    boolean IsInteriorPoint(point2 testPoint)
    {
        int intersectionPoints = 0;
        point2 previousPoint = vertices.get(vertices.size() - 1);
        for (int i = 0; i < vertices.size(); i++)
        {
            //just make sure point2(-9999,-9999) is not inside of any polygon, it is a faraway point
            //Sometimes the test line falls on a vertex, the test result is wrong
            if (HasIntersection(testPoint, new point2( -99999.1234, -99999.4321), vertices.get(i), previousPoint))
            {
                intersectionPoints++;
            }
            previousPoint = vertices.get(i);
        }

        if (intersectionPoints % 2 == 0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }


    //Helper function for HasIntersection()
    private final boolean OnSegment(point2 a, point2 b, point2 c)
    {
        if ((Math.min(a.x, b.x) <= c.x && c.x <= Math.max(a.x, b.x)) && (Math.min(a.y, b.y) <= c.y && c.y <= Math.max(a.y, b.y)))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    //Line segment intersection test, cool stuff
    //Solution
    private final boolean HasIntersection(point2 p1, point2 p2, point2 p3, point2 p4) //using final might help performance, I am not sure
    {
 
        double d1 = (p1.x - p3.x) * (p4.y - p3.y) - (p1.y - p3.y) * (p4.x - p3.x);
        double d2 = (p2.x - p3.x) * (p4.y - p3.y) - (p2.y - p3.y) * (p4.x - p3.x);
        double d3 = (p3.x - p1.x) * (p2.y - p1.y) - (p3.y - p1.y) * (p2.x - p1.x);
        double d4 = (p4.x - p1.x) * (p2.y - p1.y) - (p4.y - p1.y) * (p2.x - p1.x);

        // double d1 = point2.cross(new point2(p1.x-p3.x,p1.y-p3.y),new point2(p4.x-p3.x,p4.y-p3.y));
        // double d2 = point2.cross(new point2(p2.x-p3.x,p2.y-p3.y),new point2(p4.x-p3.x,p4.y-p3.y));
        // double d3 = point2.cross(new point2(p3.x-p1.x,p3.y-p1.y),new point2(p2.x-p1.x,p2.y-p1.y));
        // double d4 = point2.cross(new point2(p4.x-p1.x,p4.y-p1.y),new point2(p2.x-p1.x,p2.y-p1.y));

        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
        {
            return true;
        }
        else if ((d1 == 0) && OnSegment(p3, p4, p1))
        {
            return true;
        }
        else if ((d2 == 0) && OnSegment(p3, p4, p2))
        {
            return true;
        }
        else if ((d3 == 0) && OnSegment(p1, p2, p3))
        {
            return true;
        }
        else if ((d4 == 0) && OnSegment(p1, p2, p4))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    //Line-line intersection point
    point2 GetIntersection(point2 p1, point2 p2, point2 p3, point2 p4)
    {
        //if two lines are parallel, the return value will be NaN and Infinite
        //the line segments (p1,p2) and (p3,p4) do not have to intersect. The lines defined by them will intersect if not parallel.
        return new point2(((p1.x * p2.y - p1.y * p2.x) * (p3.x - p4.x) - (p3.x * p4.y - p3.y * p4.x) * (p1.x - p2.x)) /
                          ((p1.x - p2.x) * (p3.y - p4.y) - (p3.x - p4.x) * (p1.y - p2.y)),
                          ((p1.x * p2.y - p1.y * p2.x) * (p3.y - p4.y) - (p3.x * p4.y - p3.y * p4.x) * (p1.y - p2.y)) /
                          ((p1.x - p2.x) * (p3.y - p4.y) - (p3.x - p4.x) * (p1.y - p2.y)));
    }

  
}

//example mission space
/*
17 15 17 35 43 35 43 15
28 -0.1 28.5 13 31.5 13 32 -0.1
28.5 38 28 50.1 32 50.1 31.5 38
-0.1 23 -0.1 27 13.5 26.5 13.5 23.5
45 23 45 27 60.1 27 60.1 23

*********************************   general obstacle

13 15 13 50 20 50 20 15
30 7 30 18 33 28 37 28 37 7
32 35 32 45 43 45 43 35
46 10 46 18 54 18 54 10

*/

/********************************************** parallel door
5 -1 5 20 7 20 7 -1
15 -1 15 20 17 20 17 -1
25 -1 25 20 27 20 27 -1
35 -1 35 20 37 20 37 -1
45 -1 45 20 47 20 47 -1
7 26 7 51 8 51 8 26
27 26 27 51 28 51 28 26
47 26 47 51 48 51 48 26
57 26 57 51 58 51 58 26

*********************************************
7.5 7.5 2.5 12.5 7.5 17.5 12.5 12.5
7.5 20 2.5 25 7.5 30 12.5 25
7.5 32.5 2.5 37.5 7.5 42.5 12.5 37.5
22.5 7.5 17.5 12.5 22.5 17.5 27.5 12.5
22.5 20 17.5 25 22.5 30 27.5 25
22.5 32.5 17.5 37.5 22.5 42.5 27.5 37.5
32.5 7.5 32.5 17.5 42.5 17.5 42.5 7.5
32.5 20 32.5 30 42.5 30 42.5 20
32.5 32.5 32.5 42.5 42.5 42.5 42.5 32.5
47.5 7.5 47.5 17.5 57.5 17.5 57.5 7.5
47.5 20 47.5 30 57.5 30 57.5 20
47.5 32.5 47.5 42.5 57.5 42.5 57.5 32.5

******************************************
5 5 5 20 7 20 7 5
4 30 4 32 20 32 20 30
15 10 15 12 26 12 26 10
40 8 40 20 50 20 50 8
32 35 32 47 35 47 35 35
36 32 36 34 52 34 52 32

******************************************

9 -0.1 9 41 11 41 11 -0.1
11 39 11 41 51 41 51 39
49 9 49 39 51 39 51 9
19 9 19 11 51 11 51 9
19 9 19 31 21 31 21 9
19 29 19 31 41 31 41 29
39 19 39 31 41 31 41 19
29 19 29 21 41 21 41 19
********************a big narrow obstacle****************
10 3 10 47 15 47 15 3

******************* center big obstacle***************
20 15 20 40 40 40 40 15

********************* room obstacle for robots
15 0 15 20 17 20 17 0
35 0 35 20 37 20 37 0
7 26 7 50 8 50 8 26
27 26 27 50 28 50 28 26
**************************************
0 10 0 12 25 12 25 10
15 24 15 27 45 27 45 24
7 40 7 42 30 42 30 40
*******************
0 10 0 12 25 12 25 10
15 20 15 22 45 22 45 20
0 30 0 32 25 32 25 30
25 40 25 42 45 42 45 40
*
*
10 3 10 47 15 47 15 3
22.5 7.5 17.5 12.5 22.5 17.5 27.5 12.5
22.5 20 17.5 25 22.5 30 27.5 25
22.5 32.5 17.5 37.5 22.5 42.5 27.5 37.5
32.5 7.5 32.5 17.5 42.5 17.5 42.5 7.5
32.5 20 32.5 30 42.5 30 42.5 20
32.5 32.5 32.5 42.5 42.5 42.5 42.5 32.5
47.5 7.5 47.5 17.5 57.5 17.5 57.5 7.5
47.5 20 47.5 30 57.5 30 57.5 20
47.5 32.5 47.5 42.5 57.5 42.5 57.5 32.5
******************************************
46 10 46 18 54 18 54 10
30 0 30 10 35 10 35 0
5 35 5 40 15 40 15 35
5 45 5 50 15 50 15 45
*****************************************
paper locational optimization fig 4
20 10 20 20 45 20 45 10
20 35 20 45 45 45 45 35
******************************************
* city road must across
5 0 5 10 20 10 20 5 25 5 25 10 30 10 30 0
40 2 40 10 41 10 41 2
5 20 5 30 10 30 10 20
16 20 16 30 17 30 17 20
23 10 23 30 35 30 35 25 33 25 25 25 25 10
44 20 44 30 46 30 46 20
53 10 53 25 58 25 58 10 
5 50 5 40 20 40 20 45 32 45 32 40 48 40 48 45 55 45 55 40 60 40 60 50

*********************************************
* city road
5 0 5 10 20 10 20 5 25 5 25 10 30 10 30 0
40 2 40 10 41 10 41 2
5 20 5 30 10 30 10 20
16 20 16 30 17 30 17 20
23 20 23 30 35 30 35 20 33 20 33 25 25 25 25 20
44 20 44 30 46 30 46 20
53 10 53 25 58 25 58 10 
5 50 5 40 20 40 20 45 32 45 32 40 48 40 48 45 55 45 55 40 60 40 60 50
*********************************************/
