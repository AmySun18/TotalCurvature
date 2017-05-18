package curvature_total;


import java.util.*;

public class Robot
{
	

    
    public point2 position; //robot position
    //Sensing model
    //Environment
    Obstacle boundary;
    ArrayList<Obstacle> obstacles;
    
	// Control parameter
	double sensingCutoffRange;    
    public double sensingDecayFactor;

    //Neighbors
    ArrayList<Robot> neighbors = new ArrayList<Robot>();

    
    public Robot(point2 p, Obstacle bndr, ArrayList<Obstacle> gswl)
    {
    	sensingCutoffRange = 8;    
        sensingDecayFactor = 0.03; // Test from 0.02 to 0.4
        position = p;
        boundary = bndr;
        obstacles = gswl;
    }

  		
  
    public boolean is_point_visible(point2 sample_point) //Notice that a point coincide with robot position is NOT visible
    {
    	double distance = point2.Dist(sample_point, position);
        
        if (distance > this.sensingCutoffRange || distance <= 0) //to make sure dist>0 is necessary. SamplePoint might coincide with robot position. dist=0 will cause divide by zero
        {	
        	return false;
        }
    	
        //boundary block all sensing. This test can be disabled if nodes can not stay out of boundary and boundary is always a rectangle.  
        if (!boundary.LineOfSight(position, sample_point)) return false;
       
        if(!isPointInLOS(sample_point))
        {
        	return false;
        }
              
    	return true;
    }
    
    public boolean isPointInLOS(point2 samplePoint) //in line of sight, not considering FOV, only obstacles
    {
    	 for (int i1 = 0; i1 < obstacles.size(); i1++)
         {
             if (!obstacles.get(i1).LineOfSight(position, samplePoint))
             {
                 return false;
             }
         }
    	 return true;
    }
    
 
    
      

    double SensingModelFunction(double distance) //exponentially decreasing with the distance
    {
        //Make sure it is between 0 to 1
    	//if (this.id ==0)
         return Math.exp( -sensingDecayFactor * distance);
    	//else
        //return 1-0.01*distance;
    	//	return Math.exp(-sensingDecayFactor * 5 * distance);
    	//return 0.85;

    }
 
    
    double SensingModelDerivative(double distance)
    {
    	//if (this.id ==0)
    		return -sensingDecayFactor * Math.exp( -sensingDecayFactor * distance);
    	//else
    	//return -0.01;
        //return 1/(distance+1) ;
    	//	return -5*sensingDecayFactor * Math.exp( -sensingDecayFactor * distance);

    }
    
    // If there is no neighbor boost, the NeighborEffect is normal;
    synchronized double NeighborEffects(point2 samplePoint)
    {
        double neighborEffect = 1;

        for (int i1 = 0; i1 < neighbors.size(); i1++)
        {
        	double alpha = 1;
        	
        	        	
        	//TODO should use the version of is_point_visible with estimated Position and sensor_heading        	
        	if(!neighbors.get(i1).is_point_visible(samplePoint))
        	{
        		alpha = 0;
        	}
        	
        	neighborEffect *=  
        		(1 - alpha * neighbors.get(i1).SensingModelFunction(point2.Dist(samplePoint, neighbors.get(i1).position)));
        }

        return neighborEffect; //missing probability by neighbors
    }


}
