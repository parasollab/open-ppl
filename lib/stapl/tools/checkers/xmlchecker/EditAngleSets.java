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


class EditAngleSets extends JDialog implements ListSelectionListener,ActionListener {

    private Vector angSets, grpSets, grpSetStringIds, angs, tempVec, tempVec2;
    private Integer tempAngIdFromVec;
    private JButton addBtn, removeBtn;
    private JFrame frm;
    private JLabel grpSetsLabel, angSetsLabel, angsLabel, angsInAngSetLabel;
    private GroupSet tempGrpSet;
    private Angle tempAng;
    private AngleSet tempAngSet;
    private String selAngString, tempAngString, tempAngSetString;
    private int grpSetIndex, angSetIndex, angIndex;
    private JList listOGrpSets, listOAngs, listOAngsInAngSet, listOAngSets;
    private DefaultListModel angStringIds, angsInAngSet, angSetsInGrpSet;
    private boolean foundAng, foundAngSet, caught;
    private ErrorDialog error;
    private JScrollPane scrollPaneAngs, scrollPaneAngsInAngSet, scrollPaneGrpSets, scrollPaneAngSets;
    
    public EditAngleSets(JFrame parent, Vector gs, Vector a) {
	super(parent, "Angle Sets", true);
	frm = parent;
	setSize(675,350);
	grpSets = new Vector();
	tempVec = new Vector();
	tempVec2 = new Vector();
	angs = new Vector();
	angs = a;
	grpSets = gs;
	grpSetStringIds = new Vector();
	angSetsInGrpSet = new DefaultListModel();
	angStringIds = new DefaultListModel();
	angsInAngSet = new DefaultListModel();
	
	listOAngs = new JList(angStringIds);
	listOAngs.setSelectionMode(1);
	listOAngs.setFixedCellWidth(40);
	listOAngs.setFixedCellHeight(20);
	listOAngs.setVisibleRowCount(5);
	scrollPaneAngs = new JScrollPane(listOAngs);
    
	listOAngsInAngSet = new JList(angsInAngSet);
	listOAngsInAngSet.setSelectionMode(1);
	listOAngsInAngSet.setFixedCellWidth(40);
	listOAngsInAngSet.setFixedCellHeight(20);
	listOAngsInAngSet.setVisibleRowCount(5);
	scrollPaneAngsInAngSet = new JScrollPane(listOAngsInAngSet);
	
	listOGrpSets = new JList(grpSetStringIds);
	listOGrpSets.setSelectionMode(1);
	listOGrpSets.setFixedCellWidth(40);
	listOGrpSets.setFixedCellHeight(20);
	listOGrpSets.setVisibleRowCount(5);
	listOGrpSets.addListSelectionListener(this);
	scrollPaneGrpSets = new JScrollPane(listOGrpSets);
	
	listOAngSets = new JList(angSetsInGrpSet);
	listOAngSets.setSelectionMode(1);
	listOAngSets.setFixedCellWidth(40);
	listOAngSets.setFixedCellHeight(20);
	listOAngSets.setVisibleRowCount(5);
	listOAngSets.addListSelectionListener(this);
	scrollPaneAngSets = new JScrollPane(listOAngSets);
	
	grpSetsLabel = new JLabel("Group Sets");
	angSetsLabel = new JLabel("Angle Sets In Group Set");
	angsLabel = new JLabel("Angles");
	angsInAngSetLabel = new JLabel("Angles In Angle Set");
	
	addBtn = new JButton("Add Angle to Angle Set");
	removeBtn = new JButton("Remove Angle from Angle Set");
	addBtn.addActionListener(this);
	removeBtn.addActionListener(this);
	
	for(int i=0; i<grpSets.size(); i++) {
	    tempGrpSet = (GroupSet)grpSets.get(i);
	    grpSetStringIds.add(tempGrpSet.getIdString());
	}
	
	Container contentPane = getContentPane();
	GridBagLayout gbl = new GridBagLayout();
	contentPane.setLayout(gbl);
	
	GridBagConstraints gbc = new GridBagConstraints();
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(grpSetsLabel,gbc,0,0,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(scrollPaneGrpSets,gbc,0,1,1,5);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(angSetsLabel,gbc,1,0,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(scrollPaneAngSets,gbc,1,1,1,5);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(angsLabel,gbc,3,1,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(scrollPaneAngs,gbc,3,2,1,5);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(addBtn,gbc,2,4,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(removeBtn,gbc,2,7,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(angsInAngSetLabel,gbc,1,6,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(scrollPaneAngsInAngSet,gbc,1,7,1,5);
	
    }
    
    public void add(Component c, GridBagConstraints gbc, int x,
		    int y, int w, int h) {
	gbc.gridx = x;
	gbc.gridy = y;
	gbc.gridwidth = w;
	gbc.gridheight = h;
	getContentPane().add(c,gbc);
    }
  
    void clearAngsListModel() {
	angStringIds.removeAllElements();
    }
  
    void setAngsListModel(int grpSetIndex) {
	for(int i=0; i<angs.size(); i++) {
	    tempAng = (Angle)angs.get(i);
	    if(tempAng.getGrpSetId() == grpSetIndex) {
		angStringIds.addElement(tempAng.getSelYNString());
	    }
	}
    }
    
    void clearAngSetsListModel() {
	angSetsInGrpSet.removeAllElements();
    }
    
    void setAngSetsListModel(int grpSetIndex) {
	tempGrpSet = (GroupSet)grpSets.get(grpSetIndex);
	tempVec = tempGrpSet.getAngSetsVec();
	for(int i=0; i<tempVec.size(); i++) {
	    angSetsInGrpSet.addElement(Integer.toString(i));
	}
    }
    
    void clearAngsInAngSetListModel() {
	angsInAngSet.removeAllElements();
    }
    
    void setAngsInAngSetListModel(int angSetIndex, int grpSetIndex) {
	tempGrpSet = (GroupSet)grpSets.get(grpSetIndex);
	tempVec = tempGrpSet.getAngSetsVec();
	int i = 0;
	foundAngSet = false;
	do {
	    tempAngSet = (AngleSet)tempVec.get(i);
	    if(tempAngSet.getId() == angSetIndex && tempAngSet.getGrpSetId() == grpSetIndex) {
		foundAngSet = true;
		tempVec2 = tempAngSet.getAngIdsInSetVec();
		int j = 0;
		do {
		    if(tempVec2.size() != 0) {
			tempAngIdFromVec = (Integer)tempVec2.get(j);
			int k = 0;
			do {
			    foundAng = false;
			    tempAng = (Angle)angs.get(k);
			    if(tempAng.getId() == tempAngIdFromVec.intValue() && tempAng.getGrpSetId() == grpSetIndex) {
				angsInAngSet.addElement(tempAng.getIdString());
				foundAng = true;
			    }
			    k++;
			} while(!foundAng);
			j++;
		    }
		} while(j<tempVec2.size());
	    } i++;
	} while(!foundAngSet && i<tempVec.size());
    }
    
    void addAng(int angSetIndex, int grpSetIndex) {
	angIndex = listOAngs.getSelectedIndex();
	tempAngString = (String)angStringIds.get(angIndex);
	foundAng = false;
	int i = 0;
	do {
	    tempAng = (Angle)angs.get(i);
	    if(tempAng.getSelYNString() == tempAngString && tempAng.getGrpSetId() == grpSetIndex) {
		if(tempAng.getSelYN()) {
		    error = new ErrorDialog(frm,5);
		    error.show();
		}
		else {
		    if(angSetIndex!=-1) {
			tempGrpSet = (GroupSet)grpSets.get(grpSetIndex);
			tempVec = tempGrpSet.getAngSetsVec();
			tempAngSet = (AngleSet)tempVec.get(angSetIndex);
			tempAngSet.addAngle(tempAng);
		    }
		}
		foundAng = true;
	    } i++;
	} while(!foundAng);
    }
    
    void removeAng(int angSetIndex, int grpSetIndex) {
	angIndex = listOAngsInAngSet.getSelectedIndex();
	selAngString = (String)angsInAngSet.get(angIndex);
	tempGrpSet = (GroupSet)grpSets.get(grpSetIndex);
	tempVec = tempGrpSet.getAngSetsVec();
	tempAngSet = (AngleSet)tempVec.get(angSetIndex);
	foundAng = false;
	int i = 0;
	do {
	    tempAng = (Angle)angs.get(i);
	    if(tempAng.getIdString() == selAngString && tempAng.getGrpSetId() == grpSetIndex) {
		tempAngSet.removeAngle(tempAng);
		foundAng = true;
	    }i++;
	} while(!foundAng);
    }
    
    public void actionPerformed(ActionEvent evt) {
	Object src = evt.getSource();
	if(src == addBtn) {
	    grpSetIndex = listOGrpSets.getSelectedIndex();
	    angSetIndex = listOAngSets.getSelectedIndex();
	    addAng(angSetIndex,grpSetIndex);
	    clearAngsListModel();
	    clearAngsInAngSetListModel();
	    setAngsListModel(grpSetIndex);
	    setAngsInAngSetListModel(angSetIndex,grpSetIndex);
	}
	else if(src == removeBtn) {
	    grpSetIndex = listOGrpSets.getSelectedIndex();
	    angSetIndex = listOAngSets.getSelectedIndex();
	    removeAng(angSetIndex,grpSetIndex);
	    clearAngsListModel();
	    clearAngsInAngSetListModel();
	    setAngsListModel(grpSetIndex);
	    setAngsInAngSetListModel(angSetIndex,grpSetIndex);
	}
    }
    
    public void valueChanged(ListSelectionEvent e) {
	
	JList src = (JList)e.getSource();
	clearAngsListModel();
	clearAngsInAngSetListModel();
	setAngsListModel(grpSetIndex);
	if(src == listOGrpSets) {
	    grpSetIndex = listOGrpSets.getSelectedIndex();
	    //listOGrpSets.setSelectedIndex(grpSetIndex);
	    clearAngSetsListModel();
	    setAngSetsListModel(grpSetIndex);
	}
	else if(src == listOAngSets) {
	    grpSetIndex = listOGrpSets.getSelectedIndex();
	    angSetIndex = listOAngSets.getSelectedIndex();
	    setAngsInAngSetListModel(angSetIndex,grpSetIndex);
	    //repaint();
	    //listOAngSets.setSelectedIndex(angSetIndex);
	}
    }
}
