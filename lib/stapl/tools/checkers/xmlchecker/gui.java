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



public class gui {
  public static void main(String[] args) {
    JFrame f = new Gui();
    f.setVisible(true);
  }
}

class Gui extends JFrame implements ActionListener {

    private JButton doneButton, checkButton;
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
    private int numGrps, numGrpSets, numGrpsOld=0, numGrpSetsOld=0, 
numGrpSetsII=0, numGrpSetsOldII=0;


    static  tt xml_tree =new tt("input.xml" );
    static Document doc = xml_tree.getDoc();

    public Gui() {

      //xml_tree = new tt("input.xml" );
      //doc = xml_tree.getDoc();


	setTitle("Energy Specifications");
	addWindowListener(new WindowAdapter() {
		public void windowClosing(WindowEvent e) {
		    System.exit(0);
		}
	    } );


	grpsTF = new JTextField( " ",6);
       	setFieldValue( grpsTF ,"ngroups.int");

	grpSetsTF = new JTextField("",6);
        setFieldValueWithXpath(	grpSetsTF  
,"/prototype/groupsets/energy_set");
	editEnValuesBtn = new JButton("Edit Energy Range Values");
	editGrpSetsBtn = new JButton("Edit Energy Group Settings");
	editQuadTypeBtn = new JButton("Edit Quad Type and Order");
	doneButton = new JButton("Done");
        checkButton = new JButton("Check correctness");

	grps = new Vector(10,50);
	grpSets = new Vector(10,50);
	maxEnTF = new JTextField("",6);
	minEnTF = new JTextField("",6);
	editEnValuesBtn.addActionListener(this);
	editGrpSetsBtn.addActionListener(this);
	editQuadTypeBtn.addActionListener(this);
	doneButton.addActionListener(this);
        checkButton.addActionListener(this);
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

	gbc.fill = GridBagConstraints.NONE;
	gbc.anchor = GridBagConstraints.CENTER;
	gbc.weightx = 100;
	gbc.weighty = 100;
	addIt(checkButton,gbc,1,4,1,1);


    }

    public void addIt(Component c, GridBagConstraints gbc, int x,
		      int y, int w, int h) {
	gbc.gridx = x;
	gbc.gridy = y;
	gbc.gridwidth = w;
	gbc.gridheight = h;
	getContentPane().add(c,gbc);
    }

public  NodeList  getNodeList (String xpath) {
            NodeList nodelist= null;
		try  {
              nodelist = org.apache.xpath.XPathAPI.selectNodeList(doc, 
xpath);
		 		}
		catch (Exception e) {
		  e.printStackTrace();
		}
    		return nodelist;
	}

  public void  setFieldValueWithXpath( JTextField field  ,String xpath)
    {
        NodeList list = null;
        Node node = null;

        list = getNodeList ( xpath);
        int size = list.getLength();


       	field.setText(String.valueOf(size) );



    }
     public void  setFieldValue( JTextField field  ,String tagName)
    {
        NodeList list = null;
        Node node = null;



         if (doc== null) System.out.println( "doc is null" );
         else
            list = doc.getElementsByTagName(tagName);

         if (list.getLength()==0)  {
              System.out.println( tagName +" tag missing ");
              return  ;
              }
        node = list.item(0);

          NodeList childNodes = node.getChildNodes();
	  Node childNode = childNodes.item(0);
	  String nValue = childNode.getNodeValue();
	  // return  node.getNodeValue();

       	field.setText(nValue );



    }
public void  setNodeValue(JTextField field  ,String tagName)
    {
        NodeList list = null;
        Node node = null;

         int num_val = Integer.parseInt(field.getText());
         String field_value = field.getText();

         if (doc== null) System.out.println( "doc is null" );
         else
            list = doc.getElementsByTagName(tagName);

         if (list.getLength()==0)  {
              System.out.println( tagName +" tag missing ");
              return ;
              }
           node = list.item(0);

          node.replaceChild( doc.createTextNode(field_value),
                                          node.getFirstChild());

    }

    public Node getLastNode(String tagName)
    {
	NodeList list = null;
	Node node = null;

	if (doc == null) System.out.println("doc is null");
	else
	    list = doc.getElementsByTagName(tagName);

	if(list.getLength()==0) {
	    System.out.println( tagName + "tag missing");
	    return null;
	}
	int x = list.getLength()-1;
	node = list.item(x);
	return node;
    }

    public void actionPerformed(ActionEvent evt) {
	Object src = evt.getSource();
	NodeList list = null, energy_groups_tagList, energy_group_tagList, 
childList;
	Node node = null, energy_groups_node, child;
	caught = false;

	if( src == doneButton) {

	    xml_tree.writeXML("output.xml");
	    System.exit(0);

	}

       if( src == checkButton) {

	 /*      BufferedReader input_file=null;
         try{
            input_file = new 	BufferedReader( new FileReader("rules.txt"));
         } catch (FileNotFoundException ex) { }

	    new xpath (input_file);
	 */

	}


	else if(src == editEnValuesBtn) {
	    try {

		numGrps = Integer.parseInt(grpsTF.getText());
        	setNodeValue( grpsTF ,"ngroups.int");

	    }
	    catch(NumberFormatException exc) {
		caught = true;
		error = new ErrorDialog(this,3);
		error.show();
	    }
	    if(!caught) {
		if(numGrps!=numGrpsOld) {
		    grps.removeAllElements();
		    energy_group_tagList = 
getNodeList("/prototype/energy_groups/energy_group");  //node list of all the energy_group tags
		    System.out.println("number of energy_group tags is " + energy_group_tagList.getLength());
		    // for(int i=0; i<numGrps; i++) {    //creating one Group object for the # of en gps specified
		    //temp = new Group(i,0,0,doc);
		    //grps.add(temp);
		    //}
		    //if the # of energy groups desired is greater than the number already in the input file...
		    System.out.println("number of energy groups requested is " + numGrps);
		    if((numGrps>energy_group_tagList.getLength())) {
			energy_groups_tagList = getNodeList("/prototype/energy_groups");  //node list of energy_groups tags
			energy_groups_node = energy_groups_tagList.item(0);  //select the first energy_groups tag
			System.out.println(numGrps-energy_group_tagList.getLength());
			int numOfInitEnergyGroupTags = energy_group_tagList.getLength();
			for(int i=0; i<(numGrps-numOfInitEnergyGroupTags); i++) {  //1 loop for each additional group needed
			    System.out.println("loop" + i + " " + 
                   energy_group_tagList.getLength()+" "+numOfInitEnergyGroupTags);
			    energy_groups_node.appendChild(doc.createElement("energy_group"));  
//append an element node to the first energy_group tag
			    node = getLastNode("energy_group");
			    node.appendChild(doc.createElement("en_group_id"));  //append an element node to the last energy_group tag
			    node.appendChild(doc.createElement("min_en.fp"));
			    node.appendChild(doc.createElement("max_en.fp"));
			    node = getLastNode("en_group_id");  //select the last
			    String index = Integer.toString(numOfInitEnergyGroupTags + i);
			    System.out.println("new index is " + index);
			    node.appendChild(doc.createTextNode(index));  //append a text node to the last en_group_id tag
			    node = getLastNode("min_en.fp");
			    node.appendChild(doc.createTextNode("1"));
			    node = getLastNode("max_en.fp");
			    node.appendChild(doc.createTextNode("2"));
			}
			 System.out.println("out of loop");
		    }

		    else if(numGrps<energy_group_tagList.getLength()) {
			//add this code to get rid of extra energy groups
			int numOfInitEnergyGroupTags = energy_group_tagList.getLength();
			for(int i=numOfInitEnergyGroupTags-1; i>=numGrps; i--) {
			    node = getLastNode("energy_group");
			    //  for(int j=0; j<3; j++) {
			    childList = node.getChildNodes();
			    int numOfInitChildren = childList.getLength();
			    //for(int j=0; j<numOfInitChildren; j++) {
			    //	child = node.getLastChild();
			    //	node.removeChild(child);
			    // }
			    node = node.getParentNode();
			    child = node.getLastChild();
			    node.removeChild(child);
				// }
			    System.out.println(node.getNodeValue());
			}
		    }

		    for(int i=0; i<numGrps; i++) {    //creating one Group object for the # of en gps specified
			temp = new Group(i,0,0,doc);
			grps.add(temp);
		    }
	   // if(!caught) {
	//	if(numGrps!=numGrpsOld) {
	//	    grps.removeAllElements();
	//	    for(int i=0; i<numGrps; i++) {
	//		temp = new Group(i,0,0,doc);
	//		grps.add(temp);
		    //	    }
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
}

