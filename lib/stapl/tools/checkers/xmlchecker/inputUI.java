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

  


class frame1 extends JFrame implements ActionListener {



 static  tt xml_tree =new tt("input.xml" );
 static Document doc = xml_tree.getDoc();
 
  public frame1() {
    
      //xml_tree = new tt("input.xml" );
      //doc = xml_tree.getDoc();
      
    
    setTitle("Energy Specifications");
    addWindowListener(new WindowAdapter() {
      public void windowClosing(WindowEvent e) {
        System.exit(0);
      }     
    } );
    
    
    grpsTF = new JTextField("",6);
    grpSetsTF = new JTextField("",6);
    editEnValuesBtn = new JButton("Edit Energy Range Values");
    editGrpSetsBtn = new JButton("Edit Energy Group Settings");
    editQuadTypeBtn = new JButton("Edit Quad Type and Order");
    doneButton = new JButton("Done");
    
    grps = new Vector(10,50);
    grpSets = new Vector(10,50);
    maxEnTF = new JTextField("",6);
    minEnTF = new JTextField("",6);
    editEnValuesBtn.addActionListener(this);
    editGrpSetsBtn.addActionListener(this);
    editQuadTypeBtn.addActionListener(this);
    doneButton.addActionListener(this);
    setSize(550,450);
    
    warning = new JLabel("**Click corresponding button after every change to text field values**");
    grpsLabel = new JLabel("Number of Energy Groups");
    grpSetsLabel = new JLabel("Number of Group Sets");
     
    Container contentPane = getContentPane();
    GridBagLayout gbl = new GridBagLayout();
    contentPane.setLayout(gbl);
    
    GridBagConstraints gbc = new GridBagConstraints();
    gbc.fill = GridBagConstraints.NONE;
    gbc.anchor = GridBagConstraints.CENTER;
    gbc.weightx = 100;
    gbc.weighty = 100;
    addIt(warning,gbc,0,0,3,1);
    gbc.fill = GridBagConstraints.NONE;
    gbc.anchor = GridBagConstraints.CENTER;
    gbc.weightx = 100;
    gbc.weighty = 100;
    addIt(grpsLabel,gbc,0,1,1,1);
    gbc.fill = GridBagConstraints.NONE;
    gbc.anchor = GridBagConstraints.CENTER;
    gbc.weightx = 100;
    gbc.weighty = 100;
    addIt(grpSetsLabel,gbc,0,2,1,1);
    gbc.fill = GridBagConstraints.NONE;
    gbc.anchor = GridBagConstraints.CENTER;
    gbc.weightx = 100;
    gbc.weighty = 100;
    addIt(grpsTF,gbc,1,1,1,1);
    gbc.fill = GridBagConstraints.NONE;
    gbc.anchor = GridBagConstraints.CENTER;
    gbc.weightx = 100;
    gbc.weighty = 100;
    addIt(grpSetsTF,gbc,1,2,1,1);
    gbc.fill = GridBagConstraints.NONE;
    gbc.anchor = GridBagConstraints.CENTER;
    gbc.weightx = 100;
    gbc.weighty = 100;
    addIt(editEnValuesBtn,gbc,2,1,1,1);
    gbc.fill = GridBagConstraints.NONE;
    gbc.anchor = GridBagConstraints.CENTER;
    gbc.weightx = 100;
    gbc.weighty = 100;
    addIt(editGrpSetsBtn,gbc,2,2,1,1);
    gbc.fill = GridBagConstraints.NONE;
    gbc.anchor = GridBagConstraints.CENTER;
    gbc.weightx = 100;
    gbc.weighty = 100;
    addIt(editQuadTypeBtn,gbc,2,3,1,1);
   
    gbc.fill = GridBagConstraints.NONE;
    gbc.anchor = GridBagConstraints.CENTER;
    gbc.weightx = 100;
    gbc.weighty = 100;
    addIt(doneButton,gbc,0,4,1,1);
    
  }
  
  public void addIt(Component c, GridBagConstraints gbc, int x,
    int y, int w, int h) {
    gbc.gridx = x;
    gbc.gridy = y;
    gbc.gridwidth = w;
    gbc.gridheight = h;
    getContentPane().add(c,gbc);
  }
  
  public void actionPerformed(ActionEvent evt) {
    Object src = evt.getSource();
    NodeList list = null;
    Node node = null;
    caught = false;
    if( src == doneButton) {
      
       xml_tree.writeXML("output.xml");
       System.exit(0);
        
    }


   else if(src == editEnValuesBtn) {
  try {

         numGrps = Integer.parseInt(grpsTF.getText());
         String ngroups_value = grpsTF.getText();
         if (doc== null) System.out.println( "doc is null" );
         else
            list = doc.getElementsByTagName("ngroups.int");
     
         if (list.getLength()!=1)  {  
              System.out.println(" ngroup.int tag missing ");  
              return ;         
              }
           node = list.item(0);
          node.appendChild( doc.createTextNode(ngroups_value) );
  

   }
  catch(NumberFormatException exc) {
    caught = true;
    error = new ErrorDialog(this,3);
    error.show();
  }
  if(!caught) {
    if(numGrps!=numGrpsOld) {
      grps.removeAllElements();
      for(int i=0; i<numGrps; i++) {
        temp = new Group(i,0,0,doc);
        grps.add(temp);
      }
      maxMin = new MaxMinSet(this,grps,doc);
      numGrpsOld=numGrps;
    }
    maxMin.show();
  }
 }     

    else if(src == editGrpSetsBtn) {
      try {numGrpSets = Integer.parseInt(grpSetsTF.getText());}
      catch(NumberFormatException exc) {
        caught = true;
        error = new ErrorDialog(this,3);
        error.show();
      }
      if(!caught) {
        if(numGrpSets!=numGrpSetsOld) {
          grpSets.removeAllElements();
          for(int j=0; j<numGrpSets; j++) {
            tempGrpSet = new GroupSet(j);
            grpSets.add(tempGrpSet);
          }
          grpSetsFrm = new EditGrpSets(this,grpSets,grps);
          numGrpSetsOld=numGrpSets;
        }
        grpSetsFrm.show();
      }
     
     }
    else if(src == editQuadTypeBtn) {
      
     
      try {numGrpSetsII = Integer.parseInt(grpSetsTF.getText());}
      catch(NumberFormatException exc) {
        caught = true;
        error = new ErrorDialog(this,3);
        error.show();
      }
      if(!caught) {
        if(numGrpSetsII!=numGrpSetsOldII) {
          grpSets.removeAllElements();
          for(int j=0; j<numGrpSets; j++) {
            tempGrpSet = new GroupSet(j);
            grpSets.add(tempGrpSet);
          }
          quadFrm = new EditQuadTypeAndOrder(this,grpSets);
          numGrpSetsOldII = numGrpSetsII;
        }
        quadFrm.show();
      }
     }
 
    
     
 
    
}
  
  private JButton doneButton;
  private JTextField grpsTF, maxEnTF, minEnTF, grpSetsTF;
  private JButton editEnValuesBtn, editGrpSetsBtn, editQuadTypeBtn;
  private JLabel warning, grpsLabel, grpSetsLabel;
  private Vector grps, grpsString, grpsInSet, grpSets;
  private Group temp;
  private GroupSet tempGrpSet;
  private MaxMinSet maxMin=null;
  private frame1 frm;
  private EditGrpSets grpSetsFrm=null;
  private EditQuadTypeAndOrder quadFrm=null;
  private ErrorDialog error;
  private boolean caught;
  private int numGrps, numGrpSets, numGrpsOld=0, numGrpSetsOld=0, numGrpSetsII=0, numGrpSetsOldII=0;
  
}

class EditAngleSets extends JDialog implements ListSelectionListener,ActionListener {
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
}

class EditQuadTypeAndOrder extends JDialog implements ListSelectionListener,ActionListener {
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
}

class Angle {
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
  
  private int id, grpSetId;
  private String selYNString, idString;
  private boolean selYN;
}

class AngleSet {
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
  
  private int id, grpSetId;
  private String idString;
  private Vector angIdsInSet=new Vector();
  private Integer tempAngId1, tempAngId2;
  private boolean done;
}

class EditGrpSets extends JDialog implements ListSelectionListener,ActionListener {
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
}


class MaxMinSet extends JDialog implements ListSelectionListener,ActionListener {

 Document doc;

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
  
  private JTextField maxEnTF, minEnTF;
  private JList listOGrps;
  private JFrame frm;
  private Vector v, idStringForList;
  private JButton set;
  private Group selectedGroup, temp;
  private JLabel grpLabel, minLabel, maxLabel, reminderLabel;
  private ErrorDialog error;
}

class ErrorDialog extends JDialog {
  public ErrorDialog(JFrame parent, int x) {
    super(parent, "Error", true);
    
    Box b = Box.createVerticalBox();
    b.add(Box.createGlue());
    switch(x) {
      case 1: b.add(new JLabel("The selected Group is already in a Group Set.")); break;
      case 2: b.add(new JLabel("Select a Group Set before choosing Quadrature Type.")); break;
      case 3: b.add(new JLabel("Enter only integers.")); break;
      case 4: b.add(new JLabel("Enter only numbers.")); break;
      case 5: b.add(new JLabel("The selected Angle is already in an Angle Set.")); break;
      case 6: b.add(new JLabel("Number of Angle Sets must be >= Order and <= Order*(Order+2).")); break;
    }
    b.add(Box.createGlue());
    getContentPane().add(b, "Center");
    
    JPanel p2 = new JPanel();
    JButton ok = new JButton("Ok");
    p2.add(ok);
    getContentPane().add(p2, "South");
    
    ok.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent evt) {
        setVisible(false);
      }
    } );
    
    setSize(375,150);
  }
}

class Group {  //pg 657
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
  
  private int id;
  private double levelMax, levelMin;
  private String idString, idStringSelYN;
  private boolean selYN;
}

class GroupSet {
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
  private boolean done = false;
  private Group tempGrp;
  private int id = 0, order;
  private Integer tempGrpId1, tempGrpId2;
  private String idString, quadType, orderString, numAngSets;
  private Vector grpIdsInSet, angleSets;
  private AngleSet tempAngleSet;
}
/*
class frame1 extends JFrame {
  public frame1() {
    setSize(450,400);
    setTitle("Energy Specifications");
    addWindowListener(new WindowAdapter() {
      public void windowClosing(WindowEvent e) {
        System.exit(0);
      }
    } );
    Container contentPane = getContentPane();
    contentPane.add(p = new panel1(this), "Center");
  }
  
  private MaxMinSet maxMin;
  private JPanel p;
  private Vector v;
}*/

public class inputUI {
  public static void main(String[] args) {
    JFrame f = new frame1();
    f.show();
  }
}
