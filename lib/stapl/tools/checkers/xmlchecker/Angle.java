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

class Angle {

    private int id, grpSetId;
    private String selYNString, idString;
    private boolean selYN;

    public Angle() {};
  
    public Angle(int i, int j) {
	id = i;
	grpSetId = j;
	idString = Integer.toString(i);
	selYNString = idString + "*";
	selYN = false;
    }
  
    public void setSelYN(boolean sel) {
	selYN = sel;
	if(sel) selYNString = idString;
	else selYNString = idString + "*";
    }
  
    public String getSelYNString() {return selYNString;}
  
    public boolean getSelYN() {return selYN;}
  
    public int getId() {return id;}
  
    public String getIdString() {return idString;}
  
    public int getGrpSetId() {return grpSetId;}
  
}
