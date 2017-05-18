package curvature_total;
import java.util.ArrayList;
import java.util.Collections;



public class TotalCurvatureCalculation {
	
    	
	
    public static void main(String args[])
    
    {
    	long startTime = System.currentTimeMillis();
    	
     	TotalCurvatureCoverage e = new TotalCurvatureCoverage();  
  	    double h11, h,hmin, totalCurvature_tem, totalCurvature=0;
  	    point2 pi,pj;
  	    pi = new point2();
  	    pj = new point2();
  	    
  	   // Add neighbors
 	   for(int i=0;i<e.robotFullList.size();i++)
 		 {
 			   for(int j=0;j<e.robotFullList.size();j++)
 			   {
 				   if(j!=i)
 				   {
 					   if (point2.Dist(e.robotFullList.get(i).position, e.robotFullList.get(j).position) < e.robotFullList.get(0).sensingCutoffRange * 2)
 					   {
 						   e.robotFullList.get(i).neighbors.add(e.robotFullList.get(j));
 					   }
 				   }
 			   }
 		 }
 	   
 	   // Total Curvature c;
 	      	    
	   h = e.EvaluateObj_Curvature(e.robotFullList);
	   for (int i=0; i<e.robotFullList.size();i++)
	   {
		   ArrayList<Robot> robotPart11List = new ArrayList<Robot>();
		   ArrayList<Robot> robotMinusList = new ArrayList<Robot>();
		   robotPart11List.add(e.robotFullList.get(i));
		   for(int j=0; j<e.robotFullList.size();j++)
		   {
			   if(j!=i)
			   {
				   robotMinusList.add(e.robotFullList.get(j));
			   }
		   }
//		   Collections.copy(robotMinusList,e.robotFullList);
//		   robotMinusList.remove(robotMinusList.get(i));
		   		   
		   h11 = e.EvaluateObj_Curvature(robotPart11List);
		   hmin = e.EvaluateObj_Curvature(robotMinusList);
		   totalCurvature_tem = 1-(h-hmin)/h11;
//		   System.out.println("h is "+ h + " h11 "+ h11 + " hmin"+ hmin);
		   if(totalCurvature_tem>totalCurvature)
		   {
			   totalCurvature = totalCurvature_tem;
			   pi.x = robotPart11List.get(0).position.x;
			   pi.y = robotPart11List.get(0).position.y;
		   
		   }
		   
		   
	   }
	   
	   System.out.println("The total Curvature is "+  totalCurvature);
	   System.out.println("The point is at "+ pi.x + " " + pi.y + " ");
	   
	   
    long endTime   = System.currentTimeMillis();
	long totalTime = endTime - startTime;
	System.out.println(totalTime);
    }

}
