/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


import java.io.*;
import javax.swing.*;
import javax.swing.event.*;
import java.awt.*;
import java.awt.event.*;
import java.util.*;  //needed for vectors
import org.w3c.dom.*;
import org.xml.sax.*;
import javax.xml.parsers.*;

class AngleSet {

    private int id, grpSetId;
    private String idString;
    private Vector angIdsInSet=new Vector();
    private Integer tempAngId1, tempAngId2;
    private boolean done;

    public static Document doc = frame1.doc;

    public AngleSet() {};
  
    public AngleSet(int i, int gsId, Vector a) {
	id = i;
	grpSetId = gsId;
	idString = Integer.toString(i);
	angIdsInSet = a;
    }
  
    public AngleSet(int i, int gsId) {
	id = i;
	grpSetId = gsId;
	idString = Integer.toString(i);
    }
  
    public void removeAngle(Angle x) {
	int angId = x.getId();
	int i = 0;
	done = false;
	do {
	    Integer idInteger = (Integer)angIdsInSet.get(i);
	    if(idInteger.intValue() == angId) {
		angIdsInSet.remove(i);
		x.setSelYN(false);
		sort();
		done = true;
	    }
	    i++;
	} while(!done);
    }
  
    public void addAngle(Angle x) {
	angIdsInSet.add(new Integer(x.getId()));
	x.setSelYN(true);
	sort();
    }
  
    public Vector getAngIdsInSetVec() {return angIdsInSet;}
    
    public int getGrpSetId() {return grpSetId;}
  
    public int getId() {return id;}
  
    void sort() {
	do {
	    done = false;
	    for(int i=angIdsInSet.size()-1; i>0; i--) {
		tempAngId1 = (Integer)angIdsInSet.get(i);
		tempAngId2 = (Integer)angIdsInSet.get(i-1);
		if(tempAngId1.intValue()<tempAngId2.intValue()) {
		    done = true;
		    angIdsInSet.set(i,tempAngId2);
		    angIdsInSet.set(i-1,tempAngId1);
		}
	    }      
	} while(done);
    }
}
