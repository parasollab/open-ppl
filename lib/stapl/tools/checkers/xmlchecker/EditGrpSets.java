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


class EditGrpSets extends JDialog implements ListSelectionListener,ActionListener {

    private Vector grpSets, grps, grpSetStringIds, tempVec;
    private DefaultListModel grpStringIds, grpsInSetStringIds;
    private JList listOGrpSets, listOGrps, listOGrpsInSet;
    private JScrollPane scrollPane1, scrollPane2, scrollPane3;
    private JButton addBtn, removeBtn, doneBtn;
    private GroupSet tempGrpSet;
    private Group tempGrp;
    private ErrorDialog addDialog;
    private JFrame frm;
    private JLabel grpLabel, grpSetLabel, grpInSetLabel, noteLabel1, noteLabel2, reminderLabel;

    public EditGrpSets(JFrame parent, Vector gS, Vector g) {
	super(parent, "Set Energy Group Sets", true);
	frm = parent;
	grpSets = new Vector();
	grps = new Vector();
	tempVec = new Vector();
	grpStringIds = new DefaultListModel();
	grpsInSetStringIds = new DefaultListModel();
	grpSetStringIds = new Vector();
	grpLabel = new JLabel("Groups");
	grpSetLabel = new JLabel("Group Sets");
	grpInSetLabel = new JLabel("Groups In Selected Set");
	reminderLabel = new JLabel("Reminder: ALL Groups must be assigned to Group Sets.");
	noteLabel1 = new JLabel("* indicates group not yet");
	noteLabel2 = new JLabel("  included in a group set");
	grps = g;
	grpSets = gS;
	setSize(580,320);
	for(int i=0; i<grpSets.size(); i++) {
	    tempGrpSet = (GroupSet)grpSets.get(i);
	    grpSetStringIds.add(tempGrpSet.getIdString());
	}
	setGrpsListModel();
	setGrpsInSetListModel(0);
	listOGrpSets = new JList(grpSetStringIds);
	listOGrpSets.setSelectionMode(1);
	listOGrpSets.setFixedCellWidth(40);
	listOGrpSets.setFixedCellHeight(20);
	listOGrpSets.setVisibleRowCount(5);
	listOGrpSets.addListSelectionListener(this);
	scrollPane1 = new JScrollPane(listOGrpSets);
	listOGrps = new JList(grpStringIds);
	listOGrps.setSelectionMode(1);
	listOGrps.setFixedCellWidth(40);
	listOGrps.setFixedCellHeight(20);
	listOGrps.setVisibleRowCount(5);
	scrollPane2 = new JScrollPane(listOGrps);
	grpsInSetStringIds.addElement("");
	listOGrpsInSet = new JList(grpsInSetStringIds);
	listOGrpsInSet.setSelectionMode(1);
	listOGrpsInSet.setFixedCellWidth(40);
	listOGrpsInSet.setFixedCellHeight(20);
	listOGrpsInSet.setVisibleRowCount(5);
	scrollPane3 = new JScrollPane(listOGrpsInSet);
	addBtn = new JButton("Add Group to Selected Set");
	addBtn.addActionListener(this);
	removeBtn = new JButton("Remove Group from Set");
	removeBtn.addActionListener(this);
    
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
	add(scrollPane1,gbc,0,1,1,5);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(reminderLabel,gbc,1,1,2,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(addBtn,gbc,1,4,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.SOUTH;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(grpLabel,gbc,2,3,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(scrollPane2,gbc,2,4,1,5);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.SOUTH;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(noteLabel1,gbc,2,9,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.NORTH;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(noteLabel2,gbc,2,10,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.SOUTH;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(grpInSetLabel,gbc,0,7,1,1);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(scrollPane3,gbc,0,8,1,5);
	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	add(removeBtn,gbc,1,9,1,1); 
    }
  
    public void add(Component c, GridBagConstraints gbc, int x,
		    int y, int w, int h) {
	gbc.gridx = x;
	gbc.gridy = y;
	gbc.gridwidth = w;
	gbc.gridheight = h;
	getContentPane().add(c,gbc);
    }
  
    void setGrpsListModel() {
	for(int i=0; i<grps.size(); i++) {
	    tempGrp = (Group)grps.get(i);
	    grpStringIds.addElement(tempGrp.getIdStringSelYN());
	}
    }
  
    void clearGrpsListModel() {
	grpStringIds.removeAllElements();
    }
  
    void setGrpsInSetListModel(int index) {
	tempGrpSet = (GroupSet)grpSets.get(index);
	tempVec = tempGrpSet.getGrpIdsInSetVec();
	for(int i=0; i<tempVec.size(); i++) {
	    Integer x = (Integer)tempVec.get(i);
	    tempGrp = (Group)grps.get(x.intValue());
	    grpsInSetStringIds.addElement(tempGrp.getIdString());
	}
    }
  
    void clearGrpsInSetListModel() {
	grpsInSetStringIds.removeAllElements();
    }
  
    public void actionPerformed(ActionEvent e) {
	Object src = e.getSource();
	if(src == addBtn) {
	    int grpIndex = listOGrps.getSelectedIndex();
	    int grpSetIndex = listOGrpSets.getSelectedIndex();
	    tempGrpSet = (GroupSet)grpSets.get(grpSetIndex);
	    tempGrp = (Group)grps.get(grpIndex);
	    if(tempGrp.getSelYN()) {
		if(addDialog == null)
		    addDialog = new ErrorDialog(frm,1);
		addDialog.show();
	    }
	    else {
		tempGrpSet.addGrpId(tempGrp);
		clearGrpsListModel();
		setGrpsListModel();
		clearGrpsInSetListModel();
		setGrpsInSetListModel(grpSetIndex);
	    }
	}
	else if(src == removeBtn) {
	    int selGrpIndex = listOGrpsInSet.getSelectedIndex();
	    int grpSetIndex = listOGrpSets.getSelectedIndex();
	    tempGrpSet = (GroupSet)grpSets.get(grpSetIndex);
	    tempGrpSet.removeGrpId(selGrpIndex,grps);
	    clearGrpsListModel();
	    setGrpsListModel();
	    clearGrpsInSetListModel();
	    setGrpsInSetListModel(grpSetIndex);
	}
    }
    
    public void valueChanged(ListSelectionEvent evt) {
	int grpSetIndex = listOGrpSets.getSelectedIndex();
	clearGrpsInSetListModel();
	setGrpsInSetListModel(grpSetIndex);
    }
}
