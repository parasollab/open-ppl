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

class MaxMinSet extends JDialog implements ListSelectionListener,ActionListener {

    Document doc;
    private JTextField maxEnTF, minEnTF;
    private JList listOGrps;
    private JFrame frm;
    private Vector v, idStringForList;
    private JButton set;
    private Group selectedGroup, temp;
    private JLabel grpLabel, minLabel, maxLabel, reminderLabel;
    private ErrorDialog error;
   
    public MaxMinSet(JFrame parent, Vector c,  Document  doc) {
	super(parent, "Set Energy Levels", true);
       
        // make sure that modifications to the doc are seen outside 
        // the methods of this class
        
	this.doc =doc;
    
   
	frm = parent;
    
	v = new Vector();
	v = c;
	selectedGroup = new Group();
	idStringForList = new Vector();
    
	for(int i=0; i<v.size(); i++) {
	    temp = (Group)v.get(i);
	    idStringForList.add(temp.getIdString());
	}
	listOGrps = new JList(idStringForList);
	listOGrps.setSelectionMode(1);
	listOGrps.setFixedCellWidth(40);
	listOGrps.setFixedCellHeight(20);
	listOGrps.setVisibleRowCount(5);
	listOGrps.addListSelectionListener(this);
	JScrollPane scrollPane = new JScrollPane(listOGrps);
 
	set = new JButton("Set New Values");
	set.addActionListener(this);
	maxEnTF = new JTextField("",6);
	minEnTF = new JTextField("",6);
	grpLabel = new JLabel("Groups");
	minLabel = new JLabel("Minimum Energy Level");
	maxLabel = new JLabel("Maximum Energy Level");
	reminderLabel = new JLabel("Reminder: Assign energy values to ALL groups.");
        
	Container contentPane = getContentPane();
	GridBagLayout gbl = new GridBagLayout();
	contentPane.setLayout(gbl);
    
	GridBagConstraints gbc = new GridBagConstraints();
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(grpLabel,gbc,0,0,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(scrollPane,gbc,0,1,1,5);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(minLabel,gbc,2,0,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(minEnTF,gbc,2,1,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.SOUTH;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(maxLabel,gbc,2,3,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(maxEnTF,gbc,2,4,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(set,gbc,2,6,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(reminderLabel,gbc,0,6,2,1);
	setSize(520,200);
    }
  
    public void add(Component c, GridBagConstraints gbc, int x,
		    int y, int w, int h) {
	gbc.gridx = x;
	gbc.gridy = y;
	gbc.gridwidth = w;
	gbc.gridheight = h;
	getContentPane().add(c,gbc);
    }
  
    public void valueChanged(ListSelectionEvent evt) {
	JList src = (JList)evt.getSource();
	int index = src.getSelectedIndex();
	src.setSelectedIndex(index);
	selectedGroup = (Group)v.get(index);
	maxEnTF.setText(Double.toString(selectedGroup.getMax()));
	minEnTF.setText(Double.toString(selectedGroup.getMin()));   
    }
  
    public void actionPerformed(ActionEvent evt) {
	Object src2 = evt.getSource();
	if(src2==set) {
	    try {
		double a = Double.parseDouble(maxEnTF.getText());
		selectedGroup.setMax(a);
		double b = Double.parseDouble(minEnTF.getText());
		selectedGroup.setMin(b);
        
        
		String b_str = minEnTF.getText();
        
       
		NodeList list = doc.getElementsByTagName("energy_group");
         
                 
		Node node = list.item(selectedGroup.getId());
		Node child =  doc.createElement("min_energy_level") ;
         
		node.appendChild( child );
          
		child.appendChild( doc.createTextNode( b_str) );
		NodeList childNodes = node.getChildNodes();
	    }
	    catch(NumberFormatException exc) {
		error = new ErrorDialog(frm,4);
		error.show();
	    }
	}
    }
}
