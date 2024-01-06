/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


import java.io.*;
import java.util.*;
import javax.xml.parsers.*;
import javax.xml.transform.*;
import javax.xml.transform.dom.DOMSource;

import org.w3c.dom.*;
import org.xml.sax.*;
//import org.apache.xpath.XPathAPI.*;

class helpRoutines {

// returns a vector of NodeLists 
// takes as input pieces of a xpath passed as a vector 

 public Vector getNodeLists( Vector vec, Node doc)

{
       System.out.println ( " !!! geting  vector of NodeLists !!!");


  Vector   result_vec = new Vector();


      int vecLength = vec.size();
     System.out.println ( " xpath vector size: " + vecLength );
       String temp =   vec.elementAt(0).toString() ; 

       System.out.println ( " xpath vector [0] : " + vec.elementAt(0) );
     String xpath0 = getXpathExpr(temp );

  

   if( vecLength == 1)
   {    
     
        try {
            NodeList nodelist= org.apache.xpath.XPathAPI.selectNodeList (doc, xpath0);
            int numNodes = nodelist.getLength();

    System.out.println ("numNodes:"+numNodes+ "printing list of items");
    
	         
             result_vec.addElement(  nodelist);

            String[] lhsItem = new String[numNodes];
            lhsItem = xpathItems (nodelist, numNodes);
            printItems (lhsItem, numNodes) ;

         
        }
        catch (Exception e) {
            e.printStackTrace();
        }
     }

     if( vecLength > 1)
     {
       temp =   vec.elementAt(1).toString() ; 

       System.out.println ( " xpath vector [1] : " + vec.elementAt(1) );
     
       String xpath1= getXpathExpr(temp );
     


        try {

            NodeList nodelist0 =org.apache.xpath.XPathAPI.selectNodeList(doc, xpath0);
            int numNodes0 = nodelist0.getLength();
           System.out.println ("nodes0:"+numNodes0);

           for( int i=1; i<= numNodes0; i++)

           {
               String     re_xpath  = xpath0+ "[" + i +"]" + xpath1;
      
           
                 System.out.println ( " xpath [i] : " + re_xpath );

            if( vecLength == 2) {   

       

 NodeList  nodelist = org.apache.xpath.XPathAPI.selectNodeList ( doc, re_xpath); 


  result_vec.addElement(  nodelist);

  int numNodes = nodelist.getLength();
      

      System.out.println ("numNodes: "+numNodes+ "  printing list of items");

            String[] lhsItem = new String[numNodes];
            lhsItem = xpathItems (nodelist, numNodes);
           // printItems (lhsItem, numNodes) ;


           } // else
    
          if( vecLength ==3){


             temp =   vec.elementAt(2).toString() ; 

            System.out.println ( " xpath vector [2] : " + vec.elementAt(2) );
            String xpath2 = getXpathExpr(temp );
    

            NodeList nodelist1 
=org.apache.xpath.XPathAPI.selectNodeList(doc, re_xpath);
            int numNodes1 = nodelist1.getLength();
       
           for( int j=1; j<=numNodes1; j++)
           {

            String     new_xpath  =re_xpath+ "[" + j+"]" + xpath2;
            System.out.println ( " xpath [i] : " + new_xpath );

            NodeList  nodelist = org.apache.xpath.XPathAPI.selectNodeList 
( doc, new_xpath);
            

            int numNodes = nodelist.getLength();
      
    System.out.println ("numNodes:"+numNodes+ "printing list of items");
 
            String[] lhsItem = new String[numNodes];
            lhsItem = xpathItems (nodelist, numNodes);
           // printItems (lhsItem, numNodes) ;
					
             result_vec.addElement(  nodelist);


                 }// end for j

                               
                    } //else
                      if( vecLength >3)
                             System.out.println("*** this was not expected " );   


                   }// end for i

        }
        catch (Exception e) {
            e.printStackTrace();
	}
      }

  return  result_vec;
}


/*
	      Node contextNode =nodelist0.item(i);

	    String  xpath_context = "/"+contextNode.getNodeName()+ xpath1;
	  
    System.out.println( "contextNode name :"+  contextNode.getNodeName() );

// NodeList nodelist = org.apache.xpath.XPathAPI.selectNodeList ( contextNode, xpath_context);

// ! I got wrong all this stuff with context node
// selectNodeList behaves here exactly the same as in the case 
// doc is passes as first argument

NodeList  nodelist = org.apache.xpath.XPathAPI.selectNodeList (contextNode , re_xpath);
*/



  //  need to use a method that returns all the nodes values


	public static String getSingleNodeValue (NodeList nodelist) {

 
		Node node = nodelist.item(0);
        NodeList childNodes = node.getChildNodes();
        Node childNode = childNodes.item(0);
        String nValue = childNode.getNodeValue();
		System.out.println ("nValue: "+nValue);
        return nValue;
    }

	int getOpType (String oper) {

      	if (oper.equals ("=")) return 1;
       	if (oper.equals ("<")) return 2;
       	if (oper.equals (">")) return 3;
       	if (oper.equals ("<=")) return 4;
       	if (oper.equals (">=")) return 5;
       	if (oper.equals ("!=")) return 6;
		return 0;

	}
        
	int getArithOpType (String oper) {

      	if (oper.equals ("+")) return 1;
       	if (oper.equals ("-")) return 2;
       	if (oper.equals ("*")) return 3;
       	if (oper.equals ("/")) return 4;
		return 0;
	}

	String getXpathExpr (String xpathexpr) {

		String xpath_var; 
       	xpath_var = xpathexpr.substring(1, xpathexpr.length()-1);
       	System.out.println (xpath_var);
       	return xpath_var;
	}               

	int checkTheCondition (int nodeVal, int cons, int op) {

        /* 1:=, 2:<, 3:>, 4:<=, 5:>=, 6:!= */
        switch (op) {
            case 1: if (nodeVal == cons) return 1;
                    else return 0;
            case 2: if (nodeVal < cons) return 1;
                    else return 0;
            case 3: if (nodeVal > cons) return 1;
                    else return 0;
            case 4: if (nodeVal <= cons) return 1;
                    else return 0;
            case 5: if (nodeVal >= cons) return 1;
                    else return 0;
		}
		return -1;
	}

	String[] xpathItems (NodeList nodelist, int numNodes) {

		String[] item = new String[numNodes];
		for (int i=0; i<numNodes; i++) {
			Node node = nodelist.item(i);
			NodeList childNodes = node.getChildNodes();
			Node child = childNodes.item(0);
			item[i] = child.getNodeValue();
		}
		return item;
	}

	int[] xpathIntItems (NodeList nodelist, int numNodes) {

		int[] item = new int[numNodes];
		for (int i=0; i<numNodes; i++) {
			Node node = nodelist.item(i);
			NodeList childNodes = node.getChildNodes();
			Node child = childNodes.item(0);
			item[i] = Integer.parseInt(child.getNodeValue());
		}
		return item;
	}
	double[] xpathDoubleItems (NodeList nodelist, int numNodes) {

		double[] item = new double[numNodes];
		for (int i=0; i<numNodes; i++) {
			Node node = nodelist.item(i);
			NodeList childNodes = node.getChildNodes();
			Node child = childNodes.item(0);
			item[i] = Double.parseDouble(child.getNodeValue());
		}
		return item;
	}

	int[] makeRhsSet (int start, int end) {

		int[] item = new int[end-start+1];
		for (int i=start, j=0; i<=end; i++,j++) {
			item[j] = i;
		}
		return item;
	}
	double[] makeDoubleRhsSet (double start, double end) {
	  double i=start;
   
		double[] item = new double[(int)(end-start)+1];
		for ( int j=0; i<=end; i=i+1,j=j+1) {
			item[j] = i;
		}
		return item;
	}

  
	int subsetSets (String[] lhs, String[] rhs) {

		int i, success = 0;
		for (i=0; i<lhs.length; i++) {
			for (int j=0; j<rhs.length; j++) {
		
		
           //System.out.println ("Comparing "+lhs[i]+ "and" + rhs[j]);
				if (lhs[i].equals(rhs[j])) {
					success=1;
					break;
				}
			} 
			if (success == 0) break;
			else success = 0;
		}
		if (i==lhs.length) return 1;
		else return 0;
	}
 
    int equalSets (String[] lhs, String[] rhs) {

        int i, success = 0;
        for (i=0; i<lhs.length; i++) {
            for (int j=0; j<rhs.length; j++) {

                       //System.out.println ("Comparing "+lhs[i] +"& "+ rhs[j]);
                if (lhs[i].equals(rhs[j])) {
                    success=1;
				    break;
                }
            }
		    if (success == 0) break;
		    else success = 0;
        }
        if (i==lhs.length) return 1;
        else return 0;
    }

// this is order n^2 . can sort first=> O(n log n)

    int equalSets_asDouble (String[] lhs, String[] rhs) {

        int i, success = 0;
        for (i=0; i<lhs.length; i++) {
            for (int j=0; j<rhs.length; j++) {
             //			System.out.println ("Comparing "+lhs[i] +"& "+ rhs[j]);

		double val1 =Double.parseDouble(lhs[i]);
		double val2 =Double.parseDouble(rhs[j]);

                if ( val1 == val2) {
                    success=1;
				    break;
                }
            }
		    if (success == 0) break;
		    else success = 0;
        }
        if (i==lhs.length) return 1;
        else return 0;
    }

    int equalDoubleSets (double[] lhs, double[] rhs) {

        int i, success = 0;
        for (i=0; i<lhs.length; i++) {
            for (int j=0; j<rhs.length; j++) {

	  //	System.out.println ("Comparing "+lhs[i] +"& "+ rhs[j]);
                if (lhs[i] == rhs[j]) {
                    success=1;
				    break;
                }
            }
		    if (success == 0) break;
		    else success = 0;
        }
        if (i==lhs.length) return 1;
        else return 0;
    }
  int equalIntSets (int[] lhs, int[] rhs) {

        int i, success = 0;
        for (i=0; i<lhs.length; i++) {
            for (int j=0; j<rhs.length; j++) {

			//	System.out.println ("Comparing "+lhs[i] +"& "+ rhs[j]);
                if (lhs[i] == rhs[j]) {
                    success=1;
				    break;
                }
            }
		    if (success == 0) break;
		    else success = 0;
        }
        if (i==lhs.length) return 1;
        else return 0;
    }

    int uniqueSets (String[] items) {

		int i, j, success = 1;
		for (i=0; i<items.length; i++) 
		    for (j=i+1; j<items.length; j++) {

			//	System.out.println ("Comparing "+items[i] +"& "+ items[j]);
				if (items[i].equals(items[j])) 
				     return 0;
		    }				
		return success;
	}

    float  minOfSet (String[] items) {
	
		int i;
		float minValue = Float.parseFloat(items[0]);
		for (i=1; i<items.length; i++) {
		//	System.out.println ("Comparing "+Float.parseFloat(items[i]) +"& "+ minValue);
		    if (Float.parseFloat(items[i]) < minValue)
				minValue = Float.parseFloat(items[i]);
		}
		return minValue;
    }

    float  maxOfSet (String[] items) {
		int i;
		float maxValue = Float.parseFloat(items[0]);
		for (i=1; i<items.length; i++) {
			//System.out.println ("Comparing "+Float.parseFloat(items[i]) +"& "+ maxValue);
		    if (Float.parseFloat(items[i]) > maxValue)
				maxValue = Float.parseFloat(items[i]);
		}
		return maxValue;
    }

    float  sumOfSet (String[] items) {
		int i;
		float sumValue = 0;
		for (i=0; i<items.length; i++) {
			System.out.println ("Adding "+Float.parseFloat(items[i])); 
				sumValue = sumValue + Float.parseFloat(items[i]);
		}
		return sumValue;
    }

	void printItems (String[] items, int numItems) {

		int i;
		for (i=0;i<numItems;i++) 
			System.out.println (items[i]);
	}
}

public class checker {

	static helpRoutines help = new helpRoutines();
	static Document doc = null;

  // default constructor should be used instead of init


  public checker() {
  //	doc = parseXmlFile("200.inp ", false);
  //System.out.println (" parsing default file 200.inp" ); 
 }
  public checker(String filename) {
  	doc = parseXmlFile( filename, false);
 System.out.println (" parsing file " + filename);

  }
   
	// checks for conditions of the type Xpath Op Constant, Op=arithmetic op
	public static boolean checkDefined (String xpathexpr) {

		String xpath = help.getXpathExpr(xpathexpr);

		try  {

            NodeList nodelist = org.apache.xpath.XPathAPI.selectNodeList
					(doc, xpath);
           
           if(nodelist.getLength() >0) 
                return true;
	   }
        catch (Exception e) {
            e.printStackTrace();
        }
           return false;
          }
         

	// checks for conditions of the type Xpath Op Constant, Op=arithmetic op
	public static int checkTypeA (String xpathexpr, String opr, int cons) {

		String xpath = help.getXpathExpr(xpathexpr);
		int op = help.getOpType(opr);
		int nodeVal = 0, success = 0;
		try  {

            NodeList nodelist = org.apache.xpath.XPathAPI.selectNodeList
														(doc, xpath);
            String nodeValue = help.getSingleNodeValue (nodelist);
			nodeVal = Integer.parseInt (nodeValue);
		}
		catch (Exception e) {
			e.printStackTrace();
		}

		success = help.checkTheCondition (nodeVal, cons, op);
		return success;
	}

    // checks for conditions of the type Xpath Op Xpath, Op:=
    public static int checkTypeB (String xpathexpr1, String xpathexpr2) {

		int success = 0;
        String xpathLHS = help.getXpathExpr(xpathexpr1);
        String xpathRHS = help.getXpathExpr(xpathexpr2);

        try  {

            NodeList nodelistLHS = org.apache.xpath.XPathAPI.selectNodeList
                                                        (doc, xpathLHS);
            NodeList nodelistRHS = org.apache.xpath.XPathAPI.selectNodeList
                                                        (doc, xpathRHS);
			int numNodesLHS = nodelistLHS.getLength();
			int numNodesRHS = nodelistRHS.getLength();
			System.out.println ("numnodes lhs:"+ numNodesLHS + " rhs:"+ numNodesRHS);

			if (numNodesLHS > numNodesRHS) return success;

			String[] lhsItem = new String[numNodesLHS];
			lhsItem = help.xpathItems (nodelistLHS, numNodesLHS);

            String[] rhsItem = new String[numNodesRHS];
			rhsItem = help.xpathItems (nodelistRHS, numNodesRHS);

			success = help.subsetSets (lhsItem, rhsItem);
		}
        catch (Exception e) {
            e.printStackTrace();
        }

		return success;
	}

    // Checks for conditions of the type Xpath OP { oprnd1, .., oprndN }
	// where OP is = or SUBSET_OF
	public static int checkTypeC (String[] rhsItem, int n, String xpathExpr, 
										String op) {
	
		int success = 0;
		String xpathLHS = help.getXpathExpr(xpathExpr);
		try {
            NodeList nodelistLHS = org.apache.xpath.XPathAPI.selectNodeList (doc, xpathLHS);
		    int numNodesLHS = nodelistLHS.getLength();
			int rhsLength = n;

			System.out.println ("lhs: "+numNodesLHS+ " rhs: "+rhsLength);
			if (numNodesLHS != rhsLength && op.equals("="))
                          {
                          System.out.println (" fail: different sizes" );    
                          return success;
                          }
			String[] lhsItem = new String[numNodesLHS];
			lhsItem = help.xpathItems (nodelistLHS, numNodesLHS);

			if (op.equals("="))
				success = help.equalSets (lhsItem, rhsItem);
			else {
				System.out.println ("checking subsets");
				success = help.subsetSets (lhsItem, rhsItem);
			}
		}
		catch (Exception e) {
		    e.printStackTrace();
		}
		return success;
	}

// should be rewritten to use calls to one method 
// of kind checkTypeD to avoid code redundancy 


    public static int checkCount1 (Vector vec,  String xpath_RHS,String op){
  int success = 0;
  
   try{

  Vector list_vec = help.getNodeLists( vec, doc );

     String xpathRHS = help.getXpathExpr(xpath_RHS);
     NodeList nodelistRHS = org.apache.xpath.XPathAPI.selectNodeList (doc, xpathRHS);
	int numNodesRHS = nodelistRHS.getLength();


  for(int i=0; i<list_vec.size(); i++)   
  {
       NodeList nodelistLHS=(  NodeList) list_vec.elementAt(i) ;
       int numNodesLHS = nodelistLHS.getLength();
      System.out.println ("nodes-lhs:"+numNodesLHS+ " nodes-rhs:"+numNodesRHS);
			switch (help.getOpType(op))
			{
			case 1: if (numNodesLHS == numNodesRHS)
				 		success = 1;
					break;
			case 2: if  (numNodesLHS < numNodesRHS)	
                                           success = 1;
					break;
			case 3: if  (numNodesLHS > numNodesRHS)	
                                           success = 1;
					break;
			case 4: if  (numNodesLHS <= numNodesRHS)	
                                           success = 1;
					break;
          		case 5: if  (numNodesLHS >=  numNodesRHS)	
                                           success = 1;
					break;
			}
       
  }
 }   
        catch (Exception e) {
            e.printStackTrace();
        }

  return success;

              
      
     }

   

    // Checks for conditions of the type COUNT (Xpath) [= <= >= < >] Count (Xpath)


    public static int checkTypeD (String xpath1, String xpath2, String op) {

        int success = 0;
        String xpathLHS = help.getXpathExpr(xpath1);
        String xpathRHS = help.getXpathExpr(xpath2);
        try {
            NodeList nodelistLHS = org.apache.xpath.XPathAPI.selectNodeList (doc, xpathLHS);
            NodeList nodelistRHS = org.apache.xpath.XPathAPI.selectNodeList (doc, xpathRHS);
            int numNodesLHS = nodelistLHS.getLength();
			int numNodesRHS = nodelistRHS.getLength();

            System.out.println ("nodes-lhs:"+numNodesLHS+ " nodes-rhs:"+numNodesRHS);
			switch (help.getOpType(op))
			{
			case 1: if (nodelistRHS.getLength() == nodelistLHS.getLength()) 
				 		success = 1;
					break;
			case 2: if (nodelistRHS.getLength() < nodelistLHS.getLength()) 
				 		success = 1;
					break;
			case 3: if (nodelistRHS.getLength() > nodelistLHS.getLength()) 
				 		success = 1;
					break;
			case 4: if (nodelistRHS.getLength() <= nodelistLHS.getLength()) 
				 		success = 1;
					break;
			case 5: if (nodelistRHS.getLength() >= nodelistLHS.getLength()) 
				 		success = 1;
					break;
			}
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return success;
    }
 


// of kind checkTypeG 

   public static int checkCount2 (Vector vec, int rhsConst, String op) {   

     int success =0;
 try{

  Vector list_vec = help.getNodeLists( vec, doc );

     

  for(int i=0; i<list_vec.size(); i++)   
  {
       NodeList nodelistLHS=(  NodeList) list_vec.elementAt(i) ;
       int numNodesLHS = nodelistLHS.getLength();
     System.out.println ("nodes-lhs:"+numNodesLHS);
            	switch (help.getOpType(op))
			{
			case 1: if (numNodesLHS == rhsConst) 
				 		success = 1;
					break;
			case 2: if (numNodesLHS < rhsConst) 
				 		success = 1;
					break;
			case 3: if (numNodesLHS > rhsConst) 
				 		success = 1;
					break;
			case 4: if (numNodesLHS <= rhsConst) 
				 		success = 1;
					break;
			case 5: if (numNodesLHS >= rhsConst) 
				 		success = 1;
					break;
			}
}
  
 }   
        catch (Exception e) {
            e.printStackTrace();
        }

  return success;

              
      
     }


  


 public static int checkUnique ( Vector vec) {
 
 int success = 0;

   try{

  Vector list_vec = help.getNodeLists( vec, doc );

  for(int i=0; i<list_vec.size(); i++)   
  {
       NodeList nodelist=(  NodeList) list_vec.elementAt(i) ;
       int numNodes = nodelist.getLength();

           System.out.println ("numNodes:"+numNodes);
       String[] lhsItem = new String[numNodes];
            lhsItem = help.xpathItems (nodelist, numNodes);

            success = help.uniqueSets (lhsItem);
  }
 }   
        catch (Exception e) {
            e.printStackTrace();
        }

  return success;
}

 
    // Checks for conditions of the type COUNT (Xpath) [= < > <= >=] IntegerConstant
    public static int checkTypeG (String xpath1, int rhsConst, String op) {

        int success = 0;
        String xpathLHS = help.getXpathExpr(xpath1);
        try {
            NodeList nodelistLHS = org.apache.xpath.XPathAPI.selectNodeList (doc, xpathLHS);
            int numNodesLHS = nodelistLHS.getLength();
			System.out.println ("operator recvd:"+op);
			System.out.println ("operator: "+help.getOpType(op)+" lhs:"+numNodesLHS);
			switch (help.getOpType(op))
			{
			case 1: if (numNodesLHS == rhsConst) 
				 		success = 1;
					break;
			case 2: if (numNodesLHS < rhsConst) 
				 		success = 1;
					break;
			case 3: if (numNodesLHS > rhsConst) 
				 		success = 1;
					break;
			case 4: if (numNodesLHS <= rhsConst) 
				 		success = 1;
					break;
			case 5: if (numNodesLHS >= rhsConst) 
				 		success = 1;
					break;
			}
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return success;
    }


    // Checks for conditions of the type UNIQUE (Xpath)
    public static int checkTypeE (String xpathExpr) {

        int success = 0;
        String xpath = help.getXpathExpr(xpathExpr);
        try {
            NodeList nodelist = org.apache.xpath.XPathAPI.selectNodeList (doc, xpath);
            int numNodes = nodelist.getLength();

            System.out.println ("lhs: "+numNodes);

            String[] lhsItem = new String[numNodes];
            lhsItem = help.xpathItems (nodelist, numNodes);

            success = help.uniqueSets (lhsItem);
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return success;

    }


 // a min or a max value is returned for every node
 // at the nested level

 public static Vector getMinMax( Vector vec , String function)

{

    help.getNodeLists( vec, doc );


  Vector   result_vec = new Vector();

   try{

  Vector list_vec = help.getNodeLists( vec, doc );

  for(int i=0; i<list_vec.size(); i++)   
  {
       NodeList nodelist=(  NodeList) list_vec.elementAt(i) ;
       int numNodes = nodelist.getLength();

           System.out.println ("numNodes:"+numNodes);
       String[] lhsItem = new String[numNodes];
            lhsItem = help.xpathItems (nodelist, numNodes);

                   if (function.equals("MIN") )
               result_vec.addElement ( new Double(help.minOfSet(lhsItem) ));
		   else if (function.equals("MAX"))
		    {
		     // help.printItems( lhsItem, lhsItem.length);
		      System.out.println(" lhsItem.length " +lhsItem.length);
		      Double elem = new Double(help.maxOfSet(lhsItem));
                         result_vec.addElement(  elem);
		    }
        
      
     }   
     }   catch (Exception e) {
            e.printStackTrace();
        }

  return  result_vec;
}

  //  sum has to be computed/checked only at
  // the nested level, not at the global one/ 



    public static float checkSum (  Vector vec, String function, 
double constant) {

 float result = 0;


   try{

  Vector list_vec = help.getNodeLists( vec, doc );

  for(int i=0; i<list_vec.size(); i++)   
  {
       NodeList nodelist=(  NodeList) list_vec.elementAt(i) ;
       int numNodes = nodelist.getLength();

           System.out.println ("numNodes:"+numNodes);
       String[] lhsItem = new String[numNodes];
            lhsItem = help.xpathItems (nodelist, numNodes);
		  if (function.equals("SUM")) {
					result = help.sumOfSet (lhsItem);					
					if (result != constant) return 0;
						else return 1;
					
			     } 
           
  }
 }   
        catch (Exception e) {
            e.printStackTrace();
        }

   return  result;

  }


    // Computes xpath values of the type FUNCTION (Xpath)
    public static float checkTypeF (String xpath1, String function, double constant) {

        float result = 0;
        String xpath = help.getXpathExpr(xpath1);
        try {
            NodeList nodelist = org.apache.xpath.XPathAPI.selectNodeList (doc, xpath);
            int numNodes = nodelist.getLength();

            System.out.println ("lhs: "+numNodes);

            String[] lhsItem = new String[numNodes];
            lhsItem = help.xpathItems (nodelist, numNodes);

		    if (function.equals("MIN"))
				result = help.minOfSet (lhsItem);
		    else if (function.equals("MAX"))
				result = help.maxOfSet (lhsItem);
		    else if (function.equals("SUM")) {
					result = help.sumOfSet (lhsItem);
					if (constant != -1.0) {
						if (result != constant) return 0;
						else return 1;
					}
			     }
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return result;

    }

    public static int checkIntRange (String xpath1, int start, int end) {

		int success = 0;
        String xpath = help.getXpathExpr(xpath1);
        try {
            NodeList nodelist = org.apache.xpath.XPathAPI.selectNodeList (doc, xpath);
			int numNodesLHS = nodelist.getLength();
			int[] lhsItems = new int[numNodesLHS];
			lhsItems = help.xpathIntItems (nodelist, numNodesLHS);

			int[] rhsItems = new int[end-start+1];
			rhsItems = help.makeRhsSet (start, end);
			
			System.out.println ("lhs:"+lhsItems.length+" rhs:"+rhsItems.length);
			success = help.equalIntSets (lhsItems, rhsItems);
			return success;
		}
        catch (Exception e) {
            e.printStackTrace();
        }
		return success;
	}
	
    public static int checkFloatRange (String xpath1, double start, double end) {
	int success = 0;
        String xpath = help.getXpathExpr(xpath1);
        try {
            NodeList nodelist = org.apache.xpath.XPathAPI.selectNodeList (doc, xpath);
			int numNodesLHS = nodelist.getLength();
			if (numNodesLHS != (int )(end-start+1)) 
				return 0;
			double[] lhsItems = new double[numNodesLHS];
			lhsItems = help.xpathDoubleItems (nodelist, numNodesLHS);

			double[] rhsItems = new double[(int)(end-start)+1];
			rhsItems = help.makeDoubleRhsSet (start, end);
			
			success = help.equalDoubleSets (lhsItems, rhsItems);
			return success;
		}
        catch (Exception e) {
            e.printStackTrace();
        }
		return success;

	}
 public static String[] performUnionPathElement ( Vector xpathVector, 
 double elem){
            String [ ] union = new String [2];
            union[0] = String.valueOf(elem);

 return union;
 }

 public static String[] unionSetElem (NodeList nodelist  , Double elem) {

     
          int   numNodesLHS = nodelist.getLength();
           
            String[] lhsItems= help.xpathItems (nodelist, numNodesLHS);
	    String [ ] union = new String [numNodesLHS +1];
	    for (int i=0; i<numNodesLHS ; i++)
	      union[i]=   lhsItems[i] ;
 
            union[numNodesLHS] = String.valueOf(elem);


            return union;
    }


  // to be implemented; not used yet
 public static String[] performUnion ( Vector vec1,
 Vector  vec2){
            String [ ] union = new String [2];

 return union;
 }



 public static int  checkUnion ( Vector vecLHS,
 Vector vecRHS, Vector vec_doubleLHS, Vector vec_doubleRHS ){

  Vector list_vecLHS=   help.getNodeLists( vecLHS, doc );
   Vector list_vecRHS=  help.getNodeLists( vecRHS, doc );

 
     String [] union1 =null;
     String [] union2 =null;
     Double elem =null;

     int success = 0;
    
   try{

int nr_listsLHS =list_vecLHS.size();
int nr_listsRHS =list_vecRHS.size();

              if( nr_listsLHS !=  nr_listsRHS )
	      { System.out.println("  inconsistent [i] ");
	         return 0;
              } 

	     if( nr_listsLHS != vec_doubleLHS.size() )  
	      { 

 System.out.println(" nr_listsLHS =" + nr_listsLHS);
System.out.println("  inconsistent [i] with  vec_doubleLHS  " +  vec_doubleLHS.size());               return 0;
              }

        if(  nr_listsLHS != vec_doubleRHS.size() )  
	      { System.out.println("  inconsistent [i] with  vec_doubleLH  "+vec_doubleRHS.size());
	         return 0;
              } 






  for(int i=0; i<list_vecLHS.size(); i++)   
  {
     
  NodeList nodelistLHS=(  NodeList) list_vecLHS.elementAt(i) ;
       int numNodesLHS = nodelistLHS.getLength();

       NodeList nodelistRHS=(  NodeList) list_vecRHS.elementAt(i) ;
       int numNodesRHS = nodelistRHS.getLength();



          elem =(Double) vec_doubleLHS.elementAt(i); 
          union1 =  unionSetElem (nodelistLHS, elem);

        // System.out.println (" union1 printing list of items");
       //   help.printItems (union1, numNodesLHS+1) ;



         elem =(Double) vec_doubleRHS.elementAt(i); 
         union2 =  unionSetElem (nodelistRHS, elem);


       //    System.out.println (" union2 printing list of items");
        //   help.printItems (union2, numNodesRHS+1) ;

            success = help.equalSets_asDouble (union1, union2) ;
            if(success ==0) return 0;

              
  } 


        }
        catch (Exception e) {
            e.printStackTrace();
	}
      
        return success;

          
 
 }

    public static String[] performUnionOfXpath (String xpath_expr1, String xpath_expr2) {

        int success = 0;
        String xpath1 = help.getXpathExpr(xpath_expr1);
        String xpath2 = help.getXpathExpr(xpath_expr2);
		int numNodesLHS=0, numNodesRHS=0;
		NodeList nodelist1=null, nodelist2=null;
        try {
              nodelist1 = org.apache.xpath.XPathAPI.selectNodeList (doc, xpath1);
              nodelist2 = org.apache.xpath.XPathAPI.selectNodeList (doc, xpath2);
		}
        catch (Exception e) {
            e.printStackTrace();
        }
            numNodesLHS = nodelist1.getLength();
            numNodesRHS = nodelist2.getLength();
            String[] lhsItems = new String[numNodesLHS];
            lhsItems = help.xpathItems (nodelist1, numNodesLHS);

            String[] rhsItems = new String[numNodesRHS];
            rhsItems = help.xpathItems (nodelist2, numNodesRHS);

			/* Perform union of the xpath sets */

            return rhsItems; /* Return union */
    }

 	public static double  arith_op(double value1,double value2, String op) {

		double result = 0;;
		switch (help.getArithOpType(op)) {

		case 1: 
			System.out.println ("adding");
	   		result = value1 + value2; break;
		case 2: 
			System.out.println ("minusing");
	   		result = value1 - value2; break;
		case 3: 
			System.out.println ("multing");
	   		result = value1 * value2; break;
		case 4: 
			System.out.println ("dividing");
	   		result = value1 / value2; break;
		}
   		return result;
  	}

	public static double evaluateXpath (String xpath_expr) {

        String xpath = help.getXpathExpr(xpath_expr);
		System.out.println ("xpath:" +xpath);
        try {
            NodeList nodelist = org.apache.xpath.XPathAPI.selectNodeList
														(doc, xpath);
			System.out.println ("numnodes: "+nodelist.getLength());
			if (nodelist.getLength() != 1) {
				System.out.println ("xpath expression doesnot evaluate to a constant");
			}

			double value = Double.parseDouble (help.getSingleNodeValue(nodelist));
			return value;
        }
        catch (Exception e) {
            e.printStackTrace();
        }
		return 0.0;
	}

    public static int evaluateXpathtoInt (String xpath_expr) {

        String xpath = help.getXpathExpr(xpath_expr);
        System.out.println ("xpath:" +xpath);
        try {
            NodeList nodelist = org.apache.xpath.XPathAPI.selectNodeList
                                                        (doc, xpath);
            System.out.println ("numnodes: "+nodelist.getLength());
            if (nodelist.getLength() != 1) {
                System.out.println ("xpath expression doesnot evaluate to a constant");
                 return 0;
                }
            
            int value = Integer.parseInt (help.getSingleNodeValue(nodelist));
            return value;
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return 0;
    }

    // Parses an XML file and System.out.printlns a DOM document.
    // If validating is true, the contents is validated against the DTD
    // specified in the file.
    public static Document parseXmlFile(String filename, boolean validating) {
    	try {
        	// Create a builder factory
            DocumentBuilderFactory factory = DocumentBuilderFactory.
															newInstance();
            factory.setValidating(validating);
            // Create the builder and parse the file
            Document doc = factory.newDocumentBuilder().
													parse(new File(filename));
            return doc;
        } 
		catch (SAXException e) {
        	// A parsing error occurred; the xml input is not valid
        } 
		catch (ParserConfigurationException e) {
        } 
		catch (IOException e) {
        }
        return null;
	}

}
