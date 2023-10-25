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


class EditQuadTypeAndOrder extends JDialog implements ListSelectionListener,ActionListener {

    private JTextField quadTypeTF, orderTF, numAngSetsTF;
    private JLabel grpSetLabel, quadTypeLabel, comboLabel, orderLabel, numAngSetsLabel, reminderLabel;
    private JButton angBtn, setBtn;
    private ErrorDialog error;
    private JScrollPane scrollPane;
    private JList listOGrpSets;
    private JComboBox quadTypeCombo;
    private boolean caught, caught2, rangeError;
    private GroupSet tempGrpSet;
    private JFrame frm;
    private Vector grpSets, grpSetStringIds, angs;
    private EditAngleSets ang=null;
  
    public EditQuadTypeAndOrder(JFrame parent, Vector gs) {
	super(parent, "Quadrature Type and Order", true);
	frm = parent;
	grpSets = new Vector();
	angs = new Vector();
	grpSetStringIds = new Vector();
	grpSets = gs;
    
	for(int i=0; i<grpSets.size(); i++) {
	    tempGrpSet = (GroupSet)grpSets.get(i);
	    grpSetStringIds.add(tempGrpSet.getIdString());
	}
    
	listOGrpSets = new JList(grpSetStringIds);
	listOGrpSets.setSelectionMode(1);
	listOGrpSets.setFixedCellWidth(40);
	listOGrpSets.setFixedCellHeight(20);
	listOGrpSets.setVisibleRowCount(5);
	listOGrpSets.addListSelectionListener(this);
	scrollPane = new JScrollPane(listOGrpSets);
    
	quadTypeTF = new JTextField("Pick type -->",10);
	quadTypeTF.setEditable(false);
    
	quadTypeCombo = new JComboBox();
	quadTypeCombo.setEditable(false);
	quadTypeCombo.addItem("LevelSym");
	quadTypeCombo.addItem("something1");
	quadTypeCombo.addItem("something2");
	quadTypeCombo.addActionListener(this);
    
	orderTF = new JTextField("",6);
	numAngSetsTF = new JTextField("",6);
    
	angBtn = new JButton("Edit Angle Sets");
	angBtn.addActionListener(this);
	setBtn = new JButton("Set Quad Order & Angle Sets");
	setBtn.addActionListener(this);
	numAngSetsLabel = new JLabel("Num. of Angle Sets");
	grpSetLabel = new JLabel("Group Sets");
	quadTypeLabel = new JLabel("Quadrature Type");
	comboLabel = new JLabel("Choose a Type");
	orderLabel = new JLabel("Order");
	reminderLabel = new JLabel("Reminder: ALL Group Sets must have Type, Order, and Angle Sets defined.");
    
	Container contentPane = getContentPane();
	GridBagLayout gbl = new GridBagLayout();
	contentPane.setLayout(gbl);
    
	GridBagConstraints gbc = new GridBagConstraints();
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.SOUTH;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(grpSetLabel,gbc,0,0,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(scrollPane,gbc,0,1,1,5);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.SOUTH;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(quadTypeLabel,gbc,1,0,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(quadTypeTF,gbc,1,1,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.SOUTH;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(comboLabel,gbc,2,0,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(quadTypeCombo,gbc,2,1,1,3);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.SOUTH;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(orderLabel,gbc,1,2,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(orderTF,gbc,1,3,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(numAngSetsLabel,gbc,1,4,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(numAngSetsTF,gbc,1,5,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(setBtn,gbc,1,6,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(angBtn,gbc,2,5,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(reminderLabel,gbc,1,7,2,1);
	setSize(580,300);
    
    }
  
    public void add(Component c, GridBagConstraints gbc, int x,
		    int y, int w, int h) {
	gbc.gridx = x;
	gbc.gridy = y;
	gbc.gridwidth = w;
	gbc.gridheight = h;
	getContentPane().add(c,gbc);
    }
  
    void initializeAngsVec(GroupSet tempGrpSet) {
	angs.removeAllElements();
	for(int i=0; i<grpSets.size(); i++) {
	    tempGrpSet = (GroupSet)grpSets.get(i);
	    int x = tempGrpSet.getOrder();
	    for(int j=0; j<x*(x+2); j++) {
		angs.add(new Angle(j,i));
	    }
	}
    }
  
    public void actionPerformed(ActionEvent e) {
	Object src = e.getSource();
	if(src == quadTypeCombo) {
	    JComboBox src2 = (JComboBox)src;
	    int x = listOGrpSets.getSelectedIndex();
	    if(x!=-1) {
		tempGrpSet = (GroupSet)grpSets.get(x);
		tempGrpSet.setQuadType((String)src2.getSelectedItem());
		quadTypeTF.setText(tempGrpSet.getQuadType());
	    }
	    else {
		error = new ErrorDialog(frm,2);
		error.show();
	    }
	}
	else if(src == setBtn) {
	    caught = false;
	    rangeError = false;
	    int x = listOGrpSets.getSelectedIndex();
	    tempGrpSet = (GroupSet)grpSets.get(x);
	    try {
		int a = Integer.parseInt(orderTF.getText());
		int b = Integer.parseInt(numAngSetsTF.getText());
		if(b<a || b>a*(a+2)) {
		    error = new ErrorDialog(frm,6);
		    error.show();
		    rangeError = true;
		}
		else {
		    tempGrpSet.setOrder(a);
		    tempGrpSet.setNumAngSets(b);
		}
	    }
	    catch(NumberFormatException exc) {
		error = new ErrorDialog(frm,3);
		error.show();
		caught = true;
	    }
	    if(!caught && !rangeError) initializeAngsVec(tempGrpSet);
	}
	else if(src == angBtn) {
	    //if(ang == null)
	    ang = new EditAngleSets(frm,grpSets,angs);
	    this.hide();
	    ang.show();
	}
    }
    
    public void valueChanged(ListSelectionEvent evt) {
	int grpSetIndex = listOGrpSets.getSelectedIndex();
	tempGrpSet = (GroupSet)grpSets.get(grpSetIndex);
	quadTypeTF.setText(tempGrpSet.getQuadType());
	orderTF.setText(tempGrpSet.getOrderString());
	numAngSetsTF.setText(tempGrpSet.getNumAngSets());
    }
}
