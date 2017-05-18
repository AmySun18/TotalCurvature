package curvature_total;
import java.util.ArrayList;

public class TotalCurvatureCoverage {
	
	   public ArrayList<Robot> robotFullList;

	   Obstacle boundary = new Obstacle();
	   ArrayList<Obstacle> obstacles = new ArrayList<Obstacle>();
	
	    public TotalCurvatureCoverage()
	    {
	        
	        
	        
	        boundary.largestX = 60;
	        boundary.largestY = 50;
	        
//	        String obss = "13 15 13 50 20 50 20 15\n"
//	        +"30 7 30 18 33 28 37 28 37 7\n"
//	        +"32 35 32 45 43 45 43 35\n"
//	        +"46 10 46 18 54 18 54 10";
	        
	        String obss = " ";
	        
//	        String obss = "10 3 10 47 15 47 15 3";
	        
//	        String obss = "10 20 10 27 15 27 15 20";
            
            String[] splitLines = obss.split("\n");
            for (int i1 = 0; i1 < splitLines.length; i1++)
            {
                //    splitLines[i1] =  splitLines[i1].replaceAll(","," ");
                splitLines[i1] = splitLines[i1].replaceAll("[\t,;a-zA-Z]", " ");
                splitLines[i1] = splitLines[i1].replaceAll(" +", " ");
                if (splitLines[i1].length() > 0)
                {
                    if (splitLines[i1].charAt(0) == ' ') //eliminate preceding blanks
                    {
                        splitLines[i1] = splitLines[i1].replaceFirst(" ", "0");
                    }
                }
                try
                {
                    String[] splitNumbers = splitLines[i1].split(" ");

                    if ((splitNumbers.length >= 6) && (splitNumbers.length % 2 == 0))
                    {
                        Obstacle newObstacle = new Obstacle();
                        for (int i = 0; i < splitNumbers.length / 2; i++)
                        {
                            newObstacle.vertices.add(new point2(Double.parseDouble(splitNumbers[i * 2]) + Math.random() * 0.00,
                                    Double.parseDouble(splitNumbers[i * 2 + 1]) + Math.random() * 0.00));
                            //add this random 0.01 is because human tends to create different objects using the same vertex. That will prevent that vertex
                            //becoming visible to nodes
                        }
                       newObstacle.updateInteriorPoint();
                        newObstacle.updateBoundingBox();
                        

                        
                       newObstacle.originalVertices = (ArrayList<point2>) newObstacle.vertices.clone();
                        
                        
                        obstacles.add(newObstacle);
                    }
                } catch (NumberFormatException e1)
                {
                	e1.printStackTrace();     
                }
            }
            
            robotFullList = new ArrayList<Robot>();
            robotFullList = getFullList();
            
	    }	        
	        //load obstacles:
	        /*********************************   general obstacle
	        13 15 13 50 20 50 20 15
	        30 7 30 18 33 28 37 28 37 7
	        32 35 32 45 43 45 43 35
	        46 10 46 18 54 18 54 10
	        ***************************/
	        
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
	        
	        ****************************************** Maze obstacle

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
			
			********************Narrow_2 obstacle*****************
			10 20 10 27 15 27 15 20
	        ************************************************/
	public ArrayList<Robot> getFullList()        
	{        
	        
	       
	    	   for(double i=1; i<boundary.largestX;i+=10)
	    	   {
	    		   for(double j=1; j<boundary.largestY;j+=10)
	    		   {
	    			   point2 feasiblePoint= new point2();
	    			   boolean feasible=true;
	    			   feasiblePoint.x = i;
	    			   feasiblePoint.y = j;
	    			   for(int k=0; k<obstacles.size();k++)
	    			   {
	    				   if(obstacles.get(k).IsInteriorPoint(feasiblePoint))
	    				   {
	    					   
	    					   feasible = false;
	    					   break;
	    				   }
	    					   
	    			   }
	    			   if(feasible)
	    			   {   robotFullList.add(new Robot(feasiblePoint, boundary, obstacles));
	    			       
	    			   }
	    			   
	    		   }
	    	   }

	    return robotFullList;	
	    
	    }
	    
		   public double EvaluateObj_Curvature(ArrayList<Robot> List)
		    {
		    	double objective = 0;
		        point2 samplePoint = new point2(-1,-1);
		        double miss_prob;
		         double alpha;
		         double objEvalIncrement = 1;
		         try
		         {
		             for (double horizontalSamplePoint = 0.01; horizontalSamplePoint <= boundary.largestX;
		                                                 horizontalSamplePoint += objEvalIncrement)
		             {
		                 for (double verticalSamplePoint = 0.01; verticalSamplePoint <= boundary.largestY; verticalSamplePoint += objEvalIncrement)
		                 {
		                     miss_prob = 1;
		                     samplePoint.x = horizontalSamplePoint;
		                     samplePoint.y = verticalSamplePoint;
		                   
		                        for (int i5 = 0; i5 < List.size(); i5++)
		                         {
		                             if (point2.Dist(List.get(i5).position, samplePoint) < List.get(i5).sensingCutoffRange) //SYNC
		                             {
		                                 if (boundary.LineOfSight(List.get(i5).position, samplePoint))
		                                 {
		                                     alpha = 1; //sensing ability discount factor
		                                   
		                                     if(!List.get(i5).is_point_visible(samplePoint))
		                                     {
		                                    	 alpha = 0;
		                                     }                                   
		                                     
		                                     if(alpha>0)
		                                     {
		                                    	 miss_prob *= (1 - 
		                                    	 alpha * List.get(i5).SensingModelFunction(point2.Dist(List.get(i5).position, samplePoint)));
		                                     }
		                                 }
		                             }
		                         }

		                     objective += ( (1 - miss_prob));
		                 }
		             }
		         }
		         catch(ArrayIndexOutOfBoundsException e)
		         {
		        	 e.printStackTrace();
		             return -1;
		         }

		         return objective * objEvalIncrement * objEvalIncrement;
		    }
	
}	    
