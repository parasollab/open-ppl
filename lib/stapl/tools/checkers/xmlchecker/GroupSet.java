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

class GroupSet {

    private boolean done = false;
    private Group tempGrp;
    private int id = 0, order;
    private Integer tempGrpId1, tempGrpId2;
    private String idString, quadType, orderString, numAngSets;
    private Vector grpIdsInSet, angleSets;
    private AngleSet tempAngleSet;
    
    public GroupSet() {
	idString = "";
	quadType = "";
	order = 0;
	orderString = "";
	numAngSets = "";
	grpIdsInSet = new Vector();
	angleSets = new Vector();
    }
  
    public GroupSet(int i) {
	id = i;
	quadType = "";
	order = 0;
	orderString = "";
	numAngSets = "";
	idString = Integer.toString(i);
	grpIdsInSet = new Vector();
	angleSets = new Vector();
    }
  
    public GroupSet(int i, Vector grps) {
	grpIdsInSet = new Vector();
	angleSets = new Vector();
	quadType = "";
	order = 0;
	orderString = "";
	numAngSets = "";
	id = i;
	idString = Integer.toString(i);
	for(int j=0; j<grps.size(); j++) {
	    tempGrp = (Group)grps.get(j);
	    grpIdsInSet.add(new Integer(tempGrp.getId()));
	}
    }
  
    public String getIdString() {return idString;}
  
    public void addGrpId(Group g) {
	grpIdsInSet.add(new Integer(g.getId()));
	g.setIdStringSelYN(true);
	sort();
	return;
    }
    
    public void removeGrpId(int grpIndex, Vector grps) {
	Integer x = (Integer)grpIdsInSet.get(grpIndex);
	tempGrp = (Group)grps.get(x.intValue());
	tempGrp.setIdStringSelYN(false);
	grpIdsInSet.remove(grpIndex);
	sort();
    }
  
    public void fixNumAngleSets(int x) {
	for(int i=0; i<x; i++) {
	    tempAngleSet = new AngleSet(i,id);
	    angleSets.add(tempAngleSet);
	}
    }
  
    public int getSize() {return grpIdsInSet.size();}
  
    public Vector getAngSetsVec() {return angleSets;}
  
    public Vector getGrpIdsInSetVec() {
	return grpIdsInSet;
    }
  
    public void setQuadType(String s) {
	quadType = s;
    }
  
    public void setOrder(int x) {
	orderString = Integer.toString(x);
	order = x;
    }
  
    public String getOrderString() {return orderString;}
  
    public int getOrder() {return order;}
  
    public String getQuadType() {return quadType;}
  
    public void setNumAngSets(int x) {
	numAngSets = Integer.toString(x);
	angleSets.removeAllElements();
	for(int i=0; i<x; i++) {
	    tempAngleSet = new AngleSet(i,id);
	    angleSets.add(tempAngleSet);
	}
    }
  
    public String getNumAngSets() {return numAngSets;}
  
    void sort() {
	do {
	    done = false;
	    for(int i=grpIdsInSet.size()-1; i>0; i--) {
		tempGrpId1 = (Integer)grpIdsInSet.get(i);
		tempGrpId2 = (Integer)grpIdsInSet.get(i-1);
		if(tempGrpId1.intValue()<tempGrpId2.intValue()) {
		    done = true;
		    grpIdsInSet.set(i,tempGrpId2);
		    grpIdsInSet.set(i-1,tempGrpId1);
		}
	    }
	}while(done);
	//return();
    }
}
