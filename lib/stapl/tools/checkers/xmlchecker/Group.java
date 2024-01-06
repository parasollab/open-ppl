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

class Group {

    private int id;
    private double levelMax, levelMin;
    private String idString, idStringSelYN;
    private boolean selYN; 

    public Group( ) {}
    Document doc;
       
        // make sure that modifications to the doc are seen outside 
        // the methods of this class
        
    public Group(int a, double max, double min, Document doc) {
	id = a;
	idString = Integer.toString(a);
	idStringSelYN = idString + "*";
	levelMax = max;
	levelMin = min;
	this.doc = doc; 
    }
  
    public void setId(int a) {
	id = a;
    }
  
    public void setIdString(int a) {
	idString = Integer.toString(a);
    }
  
    public void setIdStringSelYN(boolean sel) {
	selYN = sel;
	if(sel)
	    idStringSelYN = idString;
	else
	    idStringSelYN = idString + "*";
    }
  
    public void setMax(double max) {
	levelMax = max;
    }
  
    public void setMin(double min) {
	levelMin = min;
    }
  
    public int getId() {return id;}
  
    public String getIdString() {return idString;}
  
    public String getIdStringSelYN() {return idStringSelYN;}
  
    public boolean getSelYN() {return selYN;}
  
    public double getMax() {return levelMax;}
  
    public double getMin() {return levelMin;}
}
