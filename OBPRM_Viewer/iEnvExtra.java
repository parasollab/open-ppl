/**
 *This file contains class iEnvironment which contains
 *options to configure environment for OBPRM.
 *For example, the file name for input environment,
 *file name for output path....
 *
 *@author Jyh-Ming Lien
 *@date 1/07/2001
 */
 
////////////////////////////////////////////////////////////////////////
//Package Name


////////////////////////////////////////////////////////////////////////
//Imports
import java.io.File;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;
import java.text.*;

final public class 
iEnvExtra extends iOptionPanel implements CaretListener, ActionListener, FocusListener
{
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Class data member
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        final private String m_str_env_Iconname="images/env_icon.gif";
        final private String m_str_Default_TextFieldValue="<not specified>";
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //      Class data member : for obprm command
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        //for defaul file name
        //final private String m_strobprm_DefaultName_CTag="-f";	//command tag
        //final private String m_strobprm_DefaultName_Des="Default filename";
        //private String m_strobprm_DefaultName;
        //private JTextField m_tf_obprmDefaultName;
        //private JButton m_bt_obprmDefaulOpen;   //invoke file dialog
        
	// private boolean m_bobprmDefault;
        //private JCheckBox m_cb_obprmDefault;    //Check this to used default filename
        
         //for environmental data directory name. (where models are!!)
	// final private String m_strDir_EnvName_CTag="-descDir";	//command tag
        //final private String m_strDir_EnvName_Des="Geometric Data Directory";
        //private String m_strDir_EnvName;
        //private JButton m_bt_EnvDirOpen;        //invoke file dialog
        //private JTextField m_tf_EnvDirName;
                
         //for environment file name
	// final private String m_strFile_EnvName_CTag="-envFile";//command tag
        //final private String m_strFile_EnvName_Des="Input Environment filename";
        //private String m_strFile_EnvName;
        //private JButton m_bt_EnvOpen;   //invoke file dialog
        //private JTextField m_tf_EnvName;
        
         //for map file name
	// final private String m_strFile_MapName_CTag="-outmapFil";//command tag
        //final private String m_strFile_MapName_Des="Ouput RoadMap filename";
        //private String m_strFile_MapName;
        //private JButton m_bt_MapOpen;   //invoke file dialog
        //private JTextField m_tf_MapName;
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //      Class data member : for advanced obprm command
        //
        /////////////////////////////////////////////////////////////////////////////////////////   
        iEnvironmentalBoundingBox m_iEBB;
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //      Class data member : for query command
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        //for defaul file name
        //final private String m_strquery_DefaultName_CTag="-f";	//command tag
        //final private String m_strquery_DefaultName_Des="Default filename";
	// private String  m_strquery_DefaultName;
        //private JTextField m_tf_queryDefaultName;
        //private JButton m_bt_queryDefaulOpen;   //invoke file dialog
        
	// private boolean m_bqueryDefault;
        //private JCheckBox m_cb_queryDefault;    //Check this to used default filename
        
        //for inmap file name
        //final private String m_strFile_inMapName_CTag="-inmapFile";//command tag
        //final private String m_strFile_inMapName_Des="Input RoadMap filename";
        //private String  m_strFile_inMapName;
        //private JButton m_bt_inMapOpen; //invoke file dialog
        //private JTextField m_tf_inMapName;
        
        //for query file name
	// final private String m_strFile_QryName_CTag="-queryFile";//command tag
        //final private String m_strFile_QryName_Des="Input Query filename";
        //private String  m_strFile_QryName;
        //private JButton m_bt_QryOpen;   //invoke file dialog
        //private JTextField m_tf_QryName;
        
        //for path file name
	// final private String m_strFile_PathName_CTag="-pathFile";//command tag
        //final private String m_strFile_PathName_Des="Output Path filename";
        //private String  m_strFile_PathName;
        //private JButton m_bt_PathOpen;  //invoke file dialog
        //private JTextField m_tf_PathName;
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //      Class data member : for advanced obprm command
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        //for input map file name
        final private String m_strobprm_InMap_CTag="-inmapFile";//command tag
        final private String m_strobprm_InMap_Des="Input RoadMap File";
        private String  m_strobprm_InMap;
        private JTextField m_tf_obprmInMapName;
        private JButton m_bt_obprmInMapOpen;   //invoke file dialog
        
        //for position resolution
        final private String m_strobprm_PR_CTag="-posres";//command tag
        final private String m_strobprm_PR_Des="Position Resolution";
        private double  m_dobprm_PR;
        private JTextField m_tf_obprm_PR;
        private JToggleButton m_bt_obprm_PR;
        final private double m_defaultPR=0.005;	//defined in Input.cpp
            
        //for resolution resolution
        final private String m_strobprm_RR_CTag="-orires";//command tag
        final private String m_strobprm_RR_Des="Resolution Resolution";
        private double  m_dobprm_RR;
        private JTextField m_tf_obprm_RR;
        private JToggleButton m_bt_obprm_RR;
        final private double m_defaultRR=0.005;	//defined in OBPRM.h
            
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Constructor
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        public iEnvExtra()
        {

                loadIcon(m_str_env_Iconname);
                
                setDefaultValues();	//set data members to default values
                updateValues();		//update components' value to current data values
                add(getComponent(),BorderLayout.CENTER); //add to this container
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Override iOptionPanel's methods
        //		- toString
        //		- toCommand
        //		- validateOptions
        //		- getComponent
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *Get description of this class.
         *Override Object#toString.
         */
        public String toString()
        {
                return "Configure Advanced Environment related options for OBPRM";
        }
        
        /**
	 *Set Options in this class to default values.
 	 */
	public void setDefaultValues()
	{
	    //       	setDefaultValues_for_obprm();
	    //	setDefaultValues_for_query();

        	setDefaultValues_for_advobprm();
        	
		if( m_iEBB==null )			
                	m_iEBB=new iEnvironmentalBoundingBox();
		m_iEBB.setDefaultValues();
	}
        
        
        public void updateValues()
        {
	    // 	updateValues_for_obprm();	
	    //	updateValues_for_query();
        	updateValues_for_advobprm();
        	
		if( m_iEBB!=null )
			m_iEBB.updateValues();
        }
        
        /**
         *Read Environmental related data from given file.
         *@note assume file exist
         */
         public boolean parseValues(String [] strValues) throws Exception
         {
         	if( strValues==null )
        		return false;
        	
        	//call sub options' parseValues
        	
        	/*if( strValues[0].endsWith("qry") )
        	{
        		if( parseValues_for_query(strValues)==false )
	        		return false;
	        }
		*/


	        else //mkmp
	        {
		    // 	if( parseValues_for_obprm(strValues)==false )
		    // 		return false;
	        	if( parseValues_for_advobprm(strValues)==false )
	        		return false;
	        	if( m_iEBB!=null )
				if( m_iEBB.parseValues(strValues)==false )
					return false;
		}
				
		//update components to current values
        	updateValues();
        	return true;
         }
         
        /**
         *Get command for OBPRM's Environemtal options
         *accoding to the state of components in this instance.
         */


        public String toCommand()
        {
	       return toMkmpCommand() + m_iEBB.toCommand();
        }



        /*
        public String toQryCommand()
        {
        	String Command="";
        	
        	//if use default
        	if( m_bqueryDefault )
        	{
       			Command+=(m_strquery_DefaultName_CTag+" "+m_strquery_DefaultName+" ");
        	}
        	else
        	{
        		Command+=(m_strFile_inMapName_CTag+" "+m_strFile_inMapName+" ");
        		Command+=(m_strFile_QryName_CTag+" "+m_strFile_QryName+" ");
        		Command+=(m_strFile_PathName_CTag+" "+m_strFile_PathName+" ");
        	}
        	
        	return Command;
	}
	*/


	public String toMkmpCommand()
	{
        	String Command="";
        	updateBB();
        	/* 
		   //if use default
        	if( m_bobprmDefault )
        	{
       			Command+=(m_strobprm_DefaultName_CTag+" "+m_strobprm_DefaultName+" ");
        	}
        	else
        	{
        		Command+=(m_strFile_EnvName_CTag+" "+m_strFile_EnvName+" ");
        		Command+=(m_strFile_MapName_CTag+" "+m_bt_MapOpen+" ");
        	}
        	*/

        	File current=iOBPRMUtility.getCurrentProgramDirectory();
        	// if( current!=null && current.getPath().compareTo(m_strDir_EnvName)!=0 ) 
		//Command+=(m_strFile_inMapName_CTag+" "+m_strDir_EnvName+" ");
        		
        	if(m_strobprm_InMap.compareTo(m_str_Default_TextFieldValue)!=0)
        		Command+=(m_strobprm_InMap_CTag+" "+m_strobprm_InMap+" ");
        		
        	if(m_bt_obprm_PR.isSelected())
        		Command+=(m_strobprm_PR_CTag+" "+m_dobprm_PR+" ");
        	        		
        	if(m_bt_obprm_RR.isSelected())
        		Command+=(m_strobprm_RR_CTag+" "+m_dobprm_RR+" ");
        		

		
        	return Command;
	}
        
        /**
         *This method checks if user input/selected option
         *is "correct".
         *@return true if specified files exist.
         *        false otherwise.
         */
        public boolean validateOptions()
        {
     		//if use default
	    /*
	      /*    if( m_bqueryDefault )
        	{
        		m_strquery_DefaultName=m_tf_queryDefaultName.getText();
        		if( m_strquery_DefaultName.compareTo(m_str_Default_TextFieldValue)==0 )
        			return false;
        	}
        	else
        	{
	    
        		m_strFile_inMapName=m_tf_inMapName.getText();
        		if( m_strFile_inMapName.compareTo(m_str_Default_TextFieldValue)==0 )
        			return false;
        			
        		m_strFile_QryName=m_tf_QryName.getText();
        		if( m_strFile_QryName.compareTo(m_str_Default_TextFieldValue)==0 )
        			return false;
        			        			
        		m_strFile_PathName=m_tf_PathName.getText();
        		if( m_strFile_PathName.compareTo(m_str_Default_TextFieldValue)==0 )
        			return false;
        	}
	    */
        	return true;
        }
        
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         */
        protected Component getComponent()
        {
        	//overall container
        	JPanel env_inter_Panel=new JPanel();
        	env_inter_Panel.setLayout(new BorderLayout());
                
                //basic Panel
                JPanel env_basic_Panel=new JPanel();
                env_basic_Panel.setLayout(new BoxLayout(env_basic_Panel, BoxLayout.X_AXIS));
                
                //add component to basic Panel
		//    env_basic_Panel.add(getComponent_for_obprm());
                env_basic_Panel.add(getComponent_for_obprm_bb());  //for bounding box options
                //env_basic_Panel.add(getComponent_for_query());
                
                setEventHandler();	//set listeners
                
                //add component to overall panel
                env_inter_Panel.add(env_basic_Panel, BorderLayout.CENTER);
                env_inter_Panel.add(getComponent_for_advobprm(),BorderLayout.SOUTH); //advanced options
                
                //return
                return env_inter_Panel;
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //	public
        //      Event Listener :
        //
        //		actionPerformed (for Action Listener)
        //		caretUpdate	(for Caret Listener)
        //		focusGained	(for Focus Listener)
        //		focusLost	(for Focus Listener)
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *When user typed in default fields for obprm or query.
         *the file name for other files, such as env, map, path, query files
         *should be updated.
         *@see update_obprm_Filename and update_obprm_Filename
         */
        public void caretUpdate(CaretEvent e) {
	    //if( e.getSource()==m_tf_obprmDefaultName && m_cb_obprmDefault.isSelected())
	    //                  update_obprm_Filename(m_tf_obprmDefaultName.getText());
	    //  else if( e.getSource()==m_tf_queryDefaultName && m_cb_queryDefault.isSelected())
	    //          update_query_Filename(m_tf_queryDefaultName.getText());
        }

        
        /**
         *Handle events form buttons, most of them are open a file chooser
         *and choose a file or dir.
         *@see iOBPRMUtility.getFilename2Open and iOBPRMUtility.getDirname
         */
        public void actionPerformed(ActionEvent e) 
        {
        	//obprm fefault file
	    /*
	    if( e.getSource()==m_bt_obprmDefaulOpen)
                {
                	String strDefaultName=
                	iOBPRMUtility.getFilename2Open(this,"Open Default FileName",null);
                        if( strDefaultName!=null )
                        {
                        	//truncate extension of filename
                                int index=strDefaultName.lastIndexOf('.');
                                if( index>0 )
                                	m_tf_obprmDefaultName.setText(strDefaultName.substring( 0, index) );
                                else
                                	m_tf_obprmDefaultName.setText(strDefaultName);
                                update_obprm_Filename(m_tf_obprmDefaultName.getText());
                        }
                 }
	    
                 //data directory
                 else if( e.getSource()==m_bt_EnvDirOpen)
                 {
 	                String strDirName=
 	                iOBPRMUtility.getDirname(this,"Open Geometric Data File(s) Directory");
                        if( strDirName!=null )
                        {
         	                m_tf_EnvDirName.setText(strDirName);
                        }
                 }
                 //query default file
                 else if( e.getSource()==m_bt_queryDefaulOpen)
                 {
                 	String strDefaultName=
                 	iOBPRMUtility.getFilename2Open(this,"Open Default FileName",null);
                        if( strDefaultName!=null )
                        {
                        	//truncate extension of filename
                                int index=strDefaultName.lastIndexOf('.');
                                if( index>0 )
                                	m_tf_queryDefaultName.setText(strDefaultName.substring( 0, index) );
                                else
                                        m_tf_queryDefaultName.setText(strDefaultName);
                                update_query_Filename(m_tf_queryDefaultName.getText());
                        }
                 }
                 else if( e.getSource()==m_bt_EnvOpen )
                 {
                        String [] filter=new String[1]; filter[0]="env";
                        String strEnvName=
                        iOBPRMUtility.getFilename2Open(this,"Open Environemnt FileName",filter);
                        if( strEnvName!=null )
                        	m_tf_EnvName.setText(strEnvName);
                 }

	    */


                 //map file open
                 if(e.getSource()==m_bt_obprmInMapOpen )
                 {
                 	String [] filter=new String[1]; filter[0]="map";
                        String strMapName=
                        iOBPRMUtility.getFilename2Open(this,"Open Map FileName",filter);
                        if( strMapName==null ) return;
                        //if( e.getSource()==m_bt_MapOpen ) m_tf_MapName.setText(strMapName);
                        //if( e.getSource()==m_bt_inMapOpen ) m_tf_inMapName.setText(strMapName);
                        if( e.getSource()==m_bt_obprmInMapOpen ) {
			    m_tf_obprmInMapName.setText(strMapName);
			    m_strobprm_InMap=strMapName;
			}
                 }


		 /*
                 else if( e.getSource()==m_bt_QryOpen )
                 {
                 	String [] filter=new String[1]; filter[0]="query";
                        String strQryName=
                        iOBPRMUtility.getFilename2Open(this,"Open Query FileName",filter);
                        if( strQryName!=null )
                        	m_tf_QryName.setText(strQryName);
                 }else if( e.getSource()==m_bt_PathOpen )
                 {
                 	String [] filter=new String[1]; filter[0]="path";
                        String strPathName=
                        iOBPRMUtility.getFilename2Open(this,"Open Path FileName",filter);
                        if( strPathName!=null )
                        	m_tf_PathName.setText(strPathName);
                 }
	    */
	}

        
        /**
         *do nothing currently
         */
        public void focusGained(FocusEvent e) 
        {/*do nothing currently*/}
        
        /**
         *This method checks if input in text field is a number.
         */
        public void focusLost(FocusEvent e) 
        { 	
        	//check if input is number only for m_tf_obprm_PR and m_tf_obprm_RR
	            	if( e.getSource()!=m_tf_obprm_PR && e.getSource()!=m_tf_obprm_RR)
	    	return;
        	
        	float fValue;
		//get value
        	if( e.getSource()==m_tf_obprm_PR ) //for position
		fValue=(float)m_defaultPR;
        	else //sp this is for orientation
        		fValue=(float)m_defaultRR;
        		
		///////////////////////////////////////////////////////////////
       		//Now,try to parse text string
                try 
                {
                	NumberFormat Num_Format=NumberFormat.getNumberInstance();
        		fValue=Num_Format.parse( ((JTextField)(e.getSource())).getText() ).floatValue();
                	((JTextField)(e.getSource())).setText(""+fValue);
                }
                catch( ParseException ex )
                {
                	String strvalue=((JTextField)(e.getSource())).getText();
                	((JTextField)(e.getSource())).setText(""+fValue);
                        Toolkit.getDefaultToolkit().beep();
                        //output error message
                	JOptionPane.
                        showMessageDialog(this,((JTextField)(e.getSource())).getToolTipText()+
                        		       " = "+strvalue+
                                               " is not a number!!");
                }
	}

        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Private member methods :
        //
        //	For overloading:
        //              getComponent_for_obprm
        //		getComponent_for_advobprm
        //		getComponent_for_obprm_bb

        //              getComponent_for_query
        //
        //		setDefaultValues_for_obprm
        //		setDefaultValues_for_query
        //		setDefaultValues_for_advobprm
        //
        //		updateValues_for_obprm
        //		updateValues_for_query
        //		updateValues_for_advobprm
        //
        //		parseValues_for_obprm
        //		parseValues_for_query
        //		parseValues_for_advobprm
        //
        //	Private Helper:
        //		isComponentCreated_for_obprm
        //		isComponentCreated_for_query
        //		isComponentCreated_for_advobprm
        //
        //		create_Components_for_obprm
        //		create_Components_for_query
        //		create_Components_for_advobprm
        //
        //		packFileComponent
        //              setEventHandler
        //	Update:
        //              update_obprm_Filename (when user type in default file field)
        //              update_obprm_DefaultState
        //              update_query_DefaultState
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //		obprm command
        //
        //              getComponent_for_obprm
        //		setDefaultValues_for_obprm
        //		updateValues_for_obprm
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        /**
         *Set default values for obprm env components
         */

	/*
        private void setDefaultValues_for_obprm()
        {
        	m_strobprm_DefaultName=m_str_Default_TextFieldValue;
        	m_bobprmDefault=true;
        	
		//get current directory and set data dir as current dir
                File currentDir=iOBPRMUtility.getCurrentDirectory();
                                
                //if cann't set value just forget it...
                m_strDir_EnvName=
                (currentDir==null)?m_str_Default_TextFieldValue:currentDir.getPath();
                
                m_strFile_EnvName=m_str_Default_TextFieldValue;
                m_strFile_MapName=m_str_Default_TextFieldValue;
	}


	*/


	/*
        private void updateValues_for_obprm()
        {
        	//create component if necessary        
        	if( !isComponentCreated_for_obprm() )
        		create_Components_for_obprm();
        		
        	//default
                m_tf_obprmDefaultName.setText(m_strobprm_DefaultName);
                m_tf_obprmDefaultName.setEnabled(m_bobprmDefault);              
                m_bt_obprmDefaulOpen.setEnabled(m_bobprmDefault);        
		
		//switch
                m_cb_obprmDefault.setSelected(m_bobprmDefault);
                
                //env dir
                m_tf_EnvDirName.setText(m_strDir_EnvName);           
                
                //env file
                m_tf_EnvName.setText(m_strFile_EnvName);
                m_tf_EnvName.setEnabled(!m_bobprmDefault);
                m_bt_EnvOpen.setEnabled(!m_bobprmDefault);

		//road map
                m_tf_MapName.setEnabled(!m_bobprmDefault);
                m_tf_MapName.setText(m_strFile_MapName);    
                m_bt_MapOpen.setEnabled(!m_bobprmDefault);
	}
	*/


	/*
   	private boolean parseValues_for_obprm(String [] strValues) throws Exception
   	{
   		//view strValues as a file name
        	String [] values;
        	
        	//dir
        	values=iOBPRMUtility.getOptionValue(strValues[0], m_strDir_EnvName_CTag);
        	if( values!=null && values.length==2 && 
        	    values[0].compareTo(m_strDir_EnvName_CTag)==0 )
        	{
        		m_strDir_EnvName=values[1];
        	}
        	
        	//default
        	values=iOBPRMUtility.getOptionValue(strValues[0], m_strobprm_DefaultName_CTag);
        	if( values!=null && values.length==2 && 
        	    values[0].compareTo(m_strobprm_DefaultName_CTag)==0 )
        	{
        		m_strobprm_DefaultName=values[1];
        		m_strFile_EnvName=m_strobprm_DefaultName+".env";
        		m_strFile_MapName=m_strobprm_DefaultName+".map";
        		m_bobprmDefault=true;
        		return true;
        	}
        	
        	//default file is not used
        	m_bobprmDefault=false;
        	
        	//env
        	values=iOBPRMUtility.getOptionValue(strValues[0], m_strFile_EnvName_CTag);
        	if( values!=null && values.length==2 && 
        	    values[0].compareTo(m_strFile_EnvName_CTag)==0 )
        	{
        		m_strFile_EnvName=values[1];

        	}
        	else throw new Exception("No default and Environment file defined in "+strValues);
        	
        	//map
        	values=iOBPRMUtility.getOptionValue(strValues[0], m_strFile_MapName_CTag);
        	if( values!=null && values.length==2 && 
        	    values[0].compareTo(m_strFile_MapName_CTag)==0 )
        	{
        		m_strFile_MapName=values[1];
        	}
        	else throw new Exception("No default and Map file defined in "+strValues);
        	
        	return true;
   	}
        	

	*/





        /**
          *Get component for obpem command.
          *They are "defaultFile", "Envfile", and "roadmapfile".
          *@note called by getComponent.
          */



	/*
        private Component getComponent_for_obprm()
        {
                //init Panel
                JPanel env_obprm_Panel=new JPanel();
                env_obprm_Panel.setLayout(new BoxLayout(env_obprm_Panel, BoxLayout.Y_AXIS));
                env_obprm_Panel.setBorder(BorderFactory.createTitledBorder("<obprm> environment"));
                
                //add event hadlere
                m_cb_obprmDefault.addActionListener(new ActionListener(){ 
                public void actionPerformed(ActionEvent e){update_obprm_DefaultState();} });
                
                env_obprm_Panel.add(	
                packFileComponent(true,m_strobprm_DefaultName_Des,m_tf_obprmDefaultName,m_bt_obprmDefaulOpen));
                
                env_obprm_Panel.add(
                packFileComponent(true,m_strDir_EnvName_Des,m_tf_EnvDirName,m_bt_EnvDirOpen));
                
                env_obprm_Panel.add(m_cb_obprmDefault);
                     
                env_obprm_Panel.add(
                packFileComponent(true,m_strFile_EnvName_Des,m_tf_EnvName,m_bt_EnvOpen));
                
                env_obprm_Panel.add(
                packFileComponent(false,m_strFile_MapName_Des,m_tf_MapName,m_bt_MapOpen));
   
                //return
                return env_obprm_Panel;
        }

	*/
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //		obprm boudning box command 
        //		(there is only one because it is made to class)
        //
        //              getComponent_for_obprm_bb
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
             
        private Component getComponent_for_obprm_bb()
        {
        	//init Panel
                JPanel obprm_bb_Panel=(JPanel)m_iEBB.getComponent();
                obprm_bb_Panel.setBorder
                (BorderFactory.createTitledBorder("<obprm> bounding box"));
                
                //return panel
                return obprm_bb_Panel;
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //		query command
        //
        //              getComponent_for_query
        //		setDefaultValues_for_query
        //		updateValues_for_query
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *Set default values for obprm env components
         */

	/*
        private void setDefaultValues_for_query()
        {
          	m_strquery_DefaultName=m_str_Default_TextFieldValue;
        	m_bqueryDefault=true;
                m_strFile_inMapName=m_str_Default_TextFieldValue;
                m_strFile_QryName=m_str_Default_TextFieldValue;
                m_strFile_PathName=m_str_Default_TextFieldValue;
	}
	

	*/


	/*
        private void updateValues_for_query()
        {
        	//create component if necessary        
        	if( !isComponentCreated_for_query() )
        		create_Components_for_query();
        		
        	//default
                m_tf_queryDefaultName.setText(m_strquery_DefaultName); 
                m_tf_queryDefaultName.setEnabled(m_bqueryDefault);
                m_bt_queryDefaulOpen.setEnabled(m_bqueryDefault);
                
                //switch
        	m_cb_queryDefault.setSelected(m_bqueryDefault);
        	
        	//in map
                m_tf_inMapName.setText(m_strFile_inMapName); 
                m_tf_inMapName.setEnabled(!m_bqueryDefault);
                m_bt_inMapOpen.setEnabled(!m_bqueryDefault);
                
                //query file
       		m_tf_QryName.setText(m_strFile_QryName);
                m_tf_QryName.setEnabled(!m_bqueryDefault);
                m_bt_QryOpen.setEnabled(!m_bqueryDefault);
                
                //path file
                m_tf_PathName.setEnabled(!m_bqueryDefault);
                m_tf_PathName.setText(m_strFile_PathName); 
                m_bt_PathOpen.setEnabled(!m_bqueryDefault);   
	}
		
	*/

	/*
	private boolean parseValues_for_query(String [] strValues) throws Exception
        {
        	//view strValues as a file name
        	String [] values;
        	//default
        	values=iOBPRMUtility.getOptionValue(strValues[0], m_strquery_DefaultName_CTag);
        	if( values!=null && values.length==2 
        	    && values[0].compareTo(m_strquery_DefaultName_CTag)==0 )
        	{
        		m_strquery_DefaultName=values[1];
        		m_bqueryDefault=true;
        		return true;
        	}
        	
        	//default file is not used
        	m_bqueryDefault=false;
        	
        	//in map
        	values=iOBPRMUtility.getOptionValue(strValues[0], m_strFile_inMapName_CTag);
        	if( values!=null && values.length==2 
        	    && values[0].compareTo(m_strFile_inMapName_CTag)==0 )
        	{
        		m_strFile_inMapName=values[1];
        	}else throw new Exception("No default and Map file defined in "+strValues);
        	        	
        	//query
        	values=iOBPRMUtility.getOptionValue(strValues[0], m_strFile_QryName_CTag);
        	if( values!=null && values.length==2 
        	    && values[0].compareTo(m_strFile_QryName_CTag)==0 )
        	{
        		m_strFile_QryName=values[1];
        	}else throw new Exception("No default and Query file defined in "+strValues);
        	    	
        	//path
        	values=iOBPRMUtility.getOptionValue(strValues[0], m_strFile_PathName_CTag);
        	if( values!=null && values.length==2  
        	    && values[0].compareTo(m_strFile_PathName_CTag)==0 )
        	{
        		m_strFile_PathName=values[1];
        	}else throw new Exception("No default and Path file defined in "+strValues);
        	
        	return true;
        }
        

	*/
        /**
          *Get component for query command.
          *@note called by getComponent.
          */
	


/*


      private Component getComponent_for_query()
        {
                 //init Panel
                JPanel env_query_Panel=new JPanel();
                env_query_Panel.setLayout(new BoxLayout(env_query_Panel, BoxLayout.Y_AXIS));
                env_query_Panel.setBorder(BorderFactory.createTitledBorder("<query> environment"));
                
                //add event hadlere
                m_cb_queryDefault.addActionListener(new ActionListener(){ 
                public void actionPerformed(ActionEvent e){update_query_DefaultState();} });
                
                //Add to panel
                env_query_Panel.add(
                packFileComponent(true,m_strquery_DefaultName_Des,m_tf_queryDefaultName,m_bt_queryDefaulOpen));
                
                env_query_Panel.add(m_cb_queryDefault);
                     
                env_query_Panel.add(
                packFileComponent(true,m_strFile_inMapName_Des,m_tf_inMapName,m_bt_inMapOpen));
                     
                env_query_Panel.add(
                packFileComponent(true,m_strFile_QryName_Des,m_tf_QryName,m_bt_QryOpen));
                     
                env_query_Panel.add(
                packFileComponent(false,m_strFile_PathName_Des,m_tf_PathName,m_bt_PathOpen));
                     
                //return
                return env_query_Panel;
        }
*/



        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //		adv obprm command
        //
        //              getComponent_for_advobprm
        //		setDefaultValues_for_advobprm
        //		updateValues_for_advobprm
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
	/**
         *Set default values for obprm env components
         */
        private void setDefaultValues_for_advobprm()
        {
          	m_strobprm_InMap=m_str_Default_TextFieldValue;
        	m_dobprm_PR=m_defaultPR;
        	m_dobprm_RR=m_defaultRR;
	}
	
        private void updateValues_for_advobprm()
        {
        	//create component if necessary
        	if( !isComponentCreated_for_advobprm() )
			create_Components_for_advobprm();
			
		//in map
                m_tf_obprmInMapName.setText(m_strobprm_InMap);
                
                //RR
                m_bt_obprm_RR.setSelected(m_dobprm_RR!=m_defaultRR);
                m_tf_obprm_RR.setText(""+m_dobprm_RR); 
                m_tf_obprm_RR.setEnabled(m_bt_obprm_RR.isSelected());
                
                //PR
        	m_bt_obprm_PR.setSelected(m_dobprm_PR!=m_defaultPR);
        	m_tf_obprm_PR.setText(""+m_dobprm_PR); 
                m_tf_obprm_PR.setEnabled(m_bt_obprm_PR.isSelected());
	}
	
        private boolean parseValues_for_advobprm(String [] strValues) throws Exception
        {
        	//see strValues as a file name
        	String [] values;
        	//input map
        	values=iOBPRMUtility.getOptionValue(strValues[0], m_strobprm_InMap_CTag);
        	if( values!=null && values.length==2 
        	    && values[0].compareTo(m_strobprm_InMap_CTag)==0 )
        	{
        		m_strobprm_InMap=values[1];
        	}
        	
        	values=iOBPRMUtility.getOptionValue(strValues[0], m_strobprm_RR_CTag);
        	if( values!=null && values.length==2 
        	    && values[0].compareTo(m_strobprm_RR_CTag)==0 )
        	{
        		m_dobprm_RR=Double.parseDouble(values[1]);
        	}
        	
             	values=iOBPRMUtility.getOptionValue(strValues[0], m_strobprm_PR_CTag);
        	if( values!=null && values.length==2 
        	    && values[0].compareTo(m_strobprm_PR_CTag)==0 )
        	{
        		m_dobprm_PR=Double.parseDouble(values[1]);
        	}
        	
        	return true;	
        }
        
        /**
         *Get Componets for advanced obprm options..
         */
        private Component getComponent_for_advobprm()
        {
        	//init Panel
                JPanel adv_obprm_Panel=new JPanel();
                adv_obprm_Panel.setLayout(new BoxLayout(adv_obprm_Panel, BoxLayout.X_AXIS));
                adv_obprm_Panel.setBorder
                (BorderFactory.createTitledBorder("<obprm> advanced environment options"));
                
                
		/////////////////////////////////////////////////////////////////////////////////
                //for position resolution
                m_bt_obprm_PR.addActionListener(new ActionListener() 
        	{
            		public void actionPerformed(ActionEvent e) 
            		{
                		m_tf_obprm_PR.setEnabled
                		(((JToggleButton)(e.getSource())).isSelected());
            		}
        	});
		
		/////////////////////////////////////////////////////////////////////////////////
		//for resolution resolution
                m_bt_obprm_RR.addActionListener(new ActionListener() 
        	{
            		public void actionPerformed(ActionEvent e) 
            		{
                		m_tf_obprm_RR.setEnabled
                		(((JToggleButton)(e.getSource())).isSelected());
            		}
        	});
		
		/////////////////////////////////////////////////////////////////////////////////
		// Add all
		
		adv_obprm_Panel.add(Box.createHorizontalGlue());
		
		adv_obprm_Panel.add(
		packFileComponent(true, m_strobprm_InMap_Des,m_tf_obprmInMapName,m_bt_obprmInMapOpen));
		
		adv_obprm_Panel.add(
		packFileComponent(true, m_strobprm_PR_Des, m_tf_obprm_PR, m_bt_obprm_PR));
		
		adv_obprm_Panel.add(
		packFileComponent(true, m_strobprm_RR_Des, m_tf_obprm_RR, m_bt_obprm_RR));		
		
		adv_obprm_Panel.add(Box.createHorizontalGlue());
		
		///packFileComponent set these button's tool tip text to "browse .."
		m_bt_obprm_PR.setToolTipText("use position resolution");
		m_bt_obprm_RR.setToolTipText("use orientation resolution");
		
		m_tf_obprm_PR.setToolTipText("Position Resolution");
		m_tf_obprm_RR.setToolTipText("Orientation Resolution");
		
		//return
		return adv_obprm_Panel;
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //		Helper
        //
        //		isComponentCreated_for_obprm
        //		create_Components_for_obprm
        //		isComponentCreated_for_query
        //		create_Components_for_query
        //		isComponentCreated_for_advobprm
        //		create_Components_for_advobprm
        //              packFileComponent
        //		setEventHandler
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////


	/*
        private boolean isComponentCreated_for_obprm()
        {
        	if( m_tf_obprmDefaultName==null || m_bt_MapOpen==null )
        		return false;
        	return true;
        }
        

	*/

	/*
        private void
        create_Components_for_obprm()
        {
        	ImageIcon open_Icon=new ImageIcon("images/Open16.gif");
                
     		//for defaul file name
                m_tf_obprmDefaultName=new JTextField(15);
               	m_bt_obprmDefaulOpen=new JButton(open_Icon);
               	
               	//for switch
                m_cb_obprmDefault=new JCheckBox("use default");
                m_cb_obprmDefault.setFont(new Font("times", Font.BOLD,10));
                
		//for data diectory
                m_tf_EnvDirName=new JTextField(15);
                m_bt_EnvDirOpen=new JButton(open_Icon);

                //for defaul env name
                m_tf_EnvName=new JTextField(15);
                m_bt_EnvOpen=new JButton(open_Icon);
                
                //for defaul map name
	        m_tf_MapName=new JTextField(15);
        	m_bt_MapOpen=new JButton(open_Icon);
        }

	*/


	/*
        
        private boolean isComponentCreated_for_query()
        {
        	if( m_tf_queryDefaultName==null || m_bt_PathOpen==null )
        		return false;
        	return true;
        }
        

	*/




	/*
        private void
        create_Components_for_query()
        {
        	ImageIcon open_Icon=new ImageIcon("images/Open16.gif"); //icon for file open button.
                
                //for defaul file name
                m_tf_queryDefaultName=new JTextField(15);
                m_bt_queryDefaulOpen=new JButton(open_Icon);

                //switch defaul file name
                m_cb_queryDefault=new JCheckBox("use default");
                m_cb_queryDefault.setFont(new Font("times", Font.BOLD,10));

		//input map file
		m_tf_inMapName=new JTextField(15);
                m_bt_inMapOpen=new JButton(open_Icon);
                
                //input query file
                m_tf_QryName=new JTextField(15);
                m_bt_QryOpen=new JButton(open_Icon);
                     
                //output path file
                m_tf_PathName=new JTextField(15);
                m_bt_PathOpen=new JButton(open_Icon);
        }
       



	*/



        private boolean isComponentCreated_for_advobprm()
        {
        	if( m_bt_obprmInMapOpen==null || m_tf_obprm_PR==null )
        		return false;
        	return true;
        }
         
        private void
        create_Components_for_advobprm()
        {
        	//image for button
		ImageIcon open_Icon=new ImageIcon("images/Open16.gif"); //icon for file open button.
                
		/////////////////////////////////////////////////////////////////////////////////
                //for input map file
                m_bt_obprmInMapOpen=new JButton(open_Icon);
              	m_tf_obprmInMapName=new JTextField(15);
              	
              	//RR
                m_bt_obprm_RR=new JToggleButton(new ImageIcon("images/Stop16.gif"));
                m_tf_obprm_RR=new JTextField(10);
                
                //PR
                m_bt_obprm_PR=new JToggleButton(new ImageIcon("images/Stop16.gif"));
        	m_tf_obprm_PR=new JTextField(10);
        }
        
        /**
         *Get a container for packing given JComponent (textfield and button).
         *For getComponent_for_obprm, getComponent_for_query, 
         *and getComponent_for_advobprm.
         */
        private Component 
        packFileComponent
        (boolean bIN,String strLabel, JTextField tfText, AbstractButton btOpen)
        {
        	//create images
        	ImageIcon iconIN_OUT=null;
        	if( bIN ) //if this is a parameter for input (file or dir)
        		iconIN_OUT = new ImageIcon("images/Import16.gif");
        	else	//output file/dir
        		iconIN_OUT = new ImageIcon("images/Export16.gif");

             	//Description label
                JLabel D_label=new JLabel(strLabel,iconIN_OUT,SwingConstants.LEFT);
                D_label.setFont(new Font("times", Font.BOLD,10));
             	 
        	JPanel return_Panel=new JPanel(); //return this panel.       
                return_Panel.setLayout(new BoxLayout(return_Panel, BoxLayout.Y_AXIS));
                
       		JPanel op_Panel=new JPanel(); //use default layout manager. for filename.
       		op_Panel.setLayout(new FlowLayout(FlowLayout.LEFT));
       		
       		//text field
                tfText.setFont(new Font("times", Font.BOLD,10));
                
                //button
                btOpen.setPreferredSize(new Dimension(20,20));
                btOpen.setToolTipText("Browse File System");
                
                op_Panel.add(tfText);
                op_Panel.add(btOpen);
                
                //put every thing to return panel
                D_label.setAlignmentX(JComponent.LEFT_ALIGNMENT);
                op_Panel.setAlignmentX(JComponent.LEFT_ALIGNMENT);
                
                return_Panel.add(D_label);
                return_Panel.add(op_Panel);
                
                //return
                return return_Panel;
        }
        
        /**
         *Set Listeners for components.
         */
        private void setEventHandler()
        {
	    //               if( m_tf_obprmDefaultName!=null )
	    //          m_tf_obprmDefaultName.addCaretListener(this);
                      
	    //   if( m_tf_queryDefaultName!=null )
	    //          m_tf_queryDefaultName.addCaretListener(this);
        
	    //  if( m_bt_obprmDefaulOpen!=null )
	    //          m_bt_obprmDefaulOpen.addActionListener(this);
                
	    //   if( m_bt_EnvDirOpen!=null )
	    //          m_bt_EnvDirOpen.addActionListener(this);
                        
	    //  if( m_bt_queryDefaulOpen!=null )
	    //          m_bt_queryDefaulOpen.addActionListener(this);
                        
	    //  if( m_bt_EnvOpen!=null )
	    //          m_bt_EnvOpen.addActionListener(this);
                                
	    //  if( m_bt_MapOpen!=null )
	    //          m_bt_MapOpen.addActionListener(this);
                
               	if( m_tf_obprm_PR!=null )
			m_tf_obprm_PR.addFocusListener(this);
			
		if( m_tf_obprm_RR!=null )
			m_tf_obprm_RR.addFocusListener(this);
			
		//	if( m_bt_inMapOpen!=null )
		//m_bt_inMapOpen.addActionListener(this);
			
		//	if( m_bt_QryOpen!=null )
		//m_bt_QryOpen.addActionListener(this);
			
		//	if( m_bt_PathOpen!=null )
		//m_bt_PathOpen.addActionListener(this);
		
		if( m_bt_obprmInMapOpen!=null )
			m_bt_obprmInMapOpen.addActionListener(this);
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //		Update 
        //
        //              update_obprm_Filename
        //		update_query_Filename
        //		update_obprm_DefaultState
        //		update_query_DefaultState
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *Update map/env filename for obprm command by given name.
         */
	/*
        private void update_obprm_Filename(String defaultName){
                String appendixMap=".map";
                String appendixEnv=".env";
                        
                if( defaultName.length()==0 )
                {
                        appendixMap="";
                        appendixEnv="";
                }
                        
                m_tf_EnvName.setText(defaultName+appendixEnv);
                m_tf_MapName.setText(defaultName+appendixMap);  
        }
        
        */




        /**
         *Update map/path/query filename for query command by given name.
         */

	/*
        private void update_query_Filename(String defaultName){
                String appendixMap=".map";
                String appendixQuery=".query";
                String appendixPath=".path";
                        
                if( defaultName.length()==0 )
                {
                        appendixMap="";
                        appendixQuery="";
                        appendixPath="";
                }
                        
                m_tf_inMapName.setText(defaultName+appendixMap);
                m_tf_QryName.setText(defaultName+appendixQuery);
                m_tf_PathName.setText(defaultName+appendixPath);        
        }
        */





        /**
         *Switch between deafult mode and file specified mode for obprm command.
         */

	/*
        private void update_obprm_DefaultState()
        {
                if( m_cb_obprmDefault.isSelected() ) //Use default
                  	m_bobprmDefault=true;
                else    //not use deafult
			m_bobprmDefault=false;
                
                m_tf_obprmDefaultName.setEnabled(m_bobprmDefault);
                m_bt_obprmDefaulOpen.setEnabled(m_bobprmDefault);
                m_tf_EnvName.setEnabled(!m_bobprmDefault);
                m_bt_EnvOpen.setEnabled(!m_bobprmDefault);
                m_tf_MapName.setEnabled(!m_bobprmDefault);
                m_bt_MapOpen.setEnabled(!m_bobprmDefault);
        }
        


	*/



        /**
         *Switch between deafult mode and file specified mode for query command.
         */

	/*
        private void update_query_DefaultState()
        {
                if( m_cb_queryDefault.isSelected() ) //Use default
			m_bqueryDefault=true;
                else    //not use deafult
			m_bqueryDefault=false;
        
                m_tf_queryDefaultName.setEnabled(m_bqueryDefault);
                m_bt_queryDefaulOpen.setEnabled(m_bqueryDefault);
                m_tf_inMapName.setEnabled(!m_bqueryDefault);        
                m_bt_inMapOpen.setEnabled(!m_bqueryDefault);
                m_tf_QryName.setEnabled(!m_bqueryDefault);
                m_bt_QryOpen.setEnabled(!m_bqueryDefault);
                m_tf_PathName.setEnabled(!m_bqueryDefault);
                m_bt_PathOpen.setEnabled(!m_bqueryDefault);
        }


	*/

 public void updateBB() 
        {
				
		NumberFormat Num_Format=NumberFormat.getNumberInstance();
        	//	fValue=Num_Format.parse( ((JTextField)(e.getSource())).getText() ).floatValue();

		try{
		    /*m_f_XMIN=Num_Format.parse(m_tf_XMIN.getText()).floatValue();
		m_f_XMAX=Num_Format.parse(m_tf_XMAX.getText()).floatValue();
		m_f_YMIN=Num_Format.parse(m_tf_YMIN.getText()).floatValue();
		m_f_YMAX=Num_Format.parse(m_tf_YMAX.getText()).floatValue();
		m_f_ZMIN=Num_Format.parse(m_tf_ZMIN.getText()).floatValue();
		m_f_ZMAX=Num_Format.parse(m_tf_ZMAX.getText()).floatValue();
		    */
		    m_dobprm_PR=Num_Format.parse(m_tf_obprm_PR.getText()).doubleValue();
		    m_dobprm_RR=Num_Format.parse(m_tf_obprm_RR.getText()).doubleValue();
		}
		catch(ParseException ex)
		    {}
	}





 }
 
 /**
  *A class represents the options for environmental bounding box
  */
 class 
 iEnvironmentalBoundingBox 
 extends iOptionPanel implements ChangeListener, FocusListener
 {
 	/////////////////////////////////////////////////////////////////////////////////////////
        //
        //      Class data member
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
 	//for position resolution
 	final private String m_strobprm_BB_CTag="-bbox";//command tag
        final private String m_strobprm_BB_Des="define a box";
        final private String m_strobprm_BBScale_CTag="-bbox_scale";//command tag
        final private String m_strobprm_BBScale_Des="box scale";
        
        private boolean bUseUserDefinedBB;
        private boolean bUseBBScale;
        
        private JToggleButton m_bt_UserBB;
        private JToggleButton m_bt_UserBBScale;
        
        final private float m_fDefault_XMIN=-10f;	
        final private float m_fDefault_XMAX=10f;	
        final private float m_fDefault_YMIN=-10f;
        final private float m_fDefault_YMAX=10f;
        final private float m_fDefault_ZMIN=-10f;
        final private float m_fDefault_ZMAX=10f;
        
        private float m_f_XMIN;	
        private float m_f_XMAX;	
        private float m_f_YMIN;
        private float m_f_YMAX;
        private float m_f_ZMIN;
        private float m_f_ZMAX;
        
        private JTextField m_tf_XMIN;	
        private JTextField m_tf_XMAX;	
        private JTextField m_tf_YMIN;	
        private JTextField m_tf_YMAX;	
        private JTextField m_tf_ZMIN;	
        private JTextField m_tf_ZMAX;
        
        private JSlider m_sl_BBScale;
        final private float m_d_DefaultBBScale_Value=2;
        private float m_f_BBScale_Value;
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Constructor
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
 	public iEnvironmentalBoundingBox()
 	{       
 		setEventHandler();
 		setToolTips();
 	}
 	
 	/////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Override iOptionPanel's methods
        //		- toString
        //		- toCommand
        //		- validateOptions
        //		- getComponent
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *Get description of this class.
         *Override Object#toString.
         */
        public String toString()
        {
                return "<obprm> bounding box";
        }
        
        /**
	 *Set Options in this class to default values.
 	 */
	public void setDefaultValues()
	{
		m_f_XMIN=m_fDefault_XMIN;	
	        m_f_XMAX=m_fDefault_XMAX;	
	        m_f_YMIN=m_fDefault_YMIN;
	        m_f_YMAX=m_fDefault_YMAX;
	        m_f_ZMIN=m_fDefault_ZMIN;
	        m_f_ZMAX=m_fDefault_ZMAX;
	        
 		m_f_BBScale_Value=m_d_DefaultBBScale_Value;
	        
	        bUseUserDefinedBB=false;
		bUseBBScale=false;
	}
	
	public void updateValues()
	{
	        updateValues_for_UserBB();
		updateValues_for_BBScale();
	}

        public boolean parseValues(String [] strValues) throws Exception
        {	
        	if( parseValues_for_UserBB(strValues)==false )
        		return false;
        	if( parseValues_for_BBScale(strValues)==false )
        		return false;
        	
	        updateValues_for_UserBB();
		updateValues_for_BBScale();
		return true;
        }
        
        /**
         *Get command for OBPRM's Environemtal options
         *accoding to the state of components in this instance.
         */
        public String toCommand()
        {
	    
	    String Command=" ";
	    if(m_bt_UserBB.isSelected())
		{
		    Command+=m_strobprm_BB_CTag + " [" + m_f_XMIN + "," + m_f_XMAX + "," + m_f_YMIN + "," + m_f_YMAX + "," + m_f_ZMIN + "," + m_f_ZMAX + "] ";

		 }
	    if(m_bt_UserBBScale.isSelected())
		{
		    Command+=m_strobprm_BBScale_CTag + " " + m_f_BBScale_Value + " ";
		}

	    
                return Command;
        }
        
        /**
         *This method checks if user input/selected option
         *is "correct".
         *@return true if specified files exist.
         *        false otherwise.
         */
        public boolean validateOptions()
        {
                //not yet
                return true;
        }
        
         /**
          *Get components which will be added to this panel.
          *@return a container which contains all components.
          */
        protected Component getComponent()
        {
        	//overall container
         	setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
         	
                //add component to bb_inter panel
                JPanel BB=(JPanel)getComponentforUserBB();
                JPanel BBScale=(JPanel)getBBScale();
                
                BB.setAlignmentX(JComponent.CENTER_ALIGNMENT);
                BBScale.setAlignmentX(JComponent.CENTER_ALIGNMENT);
                
                add(BB);	add(BBScale);
                  
                //return
                return this;
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //	public
        //      Event Listener :
        //
        //		actionPerformed (for Action Listener)
        //		caretUpdate	( for Caret Listener)
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *do nothing currently
         */
        public void focusGained(FocusEvent e) 
        {/*do nothing currently*/}
        
        /**NumberFormat Num_Format=NumberFormat.getNumberInstance();
        		fValue=Num_Format.parse( ((JTextField)(e.getSource())).getText() ).floatValue();
         *This method checks if input in text field is a number.
         */
        public void focusLost(FocusEvent e) 
        { 	
        	//I am not going to check non text filed components
       		if( !(e.getSource() instanceof JTextField) )
       			return;
       		
		///////////////////////////////////////////////////////////////
       		//find out what fielld this is?!
       		float fValue=0;
       		if( e.getSource() == m_tf_XMIN )
       			fValue=m_f_XMIN;
       		else if( e.getSource() == m_tf_XMAX )
       			fValue=m_f_XMAX;
       		else if( e.getSource() == m_tf_YMIN )
       			fValue=m_f_YMIN;
       		else if( e.getSource() == m_tf_YMAX )
       			fValue=m_f_YMAX;
       		else if( e.getSource() == m_tf_ZMIN )
       			fValue=m_f_ZMIN;
       		else if( e.getSource() == m_tf_ZMAX )
       			fValue=m_f_ZMAX;
       		
		///////////////////////////////////////////////////////////////
       		//Now,try to parse text string
                try 
                {
                	NumberFormat Num_Format=NumberFormat.getNumberInstance();
        		fValue=Num_Format.parse( ((JTextField)(e.getSource())).getText() ).floatValue();
                	((JTextField)(e.getSource())).setText(""+fValue);
                }
                catch( ParseException ex )
                {
                	String strvalue=((JTextField)(e.getSource())).getText();
                	((JTextField)(e.getSource())).setText(""+fValue);
                        Toolkit.getDefaultToolkit().beep();
                        //output error message
                	JOptionPane.
                        showMessageDialog(this,((JTextField)(e.getSource())).getToolTipText()+
                        		       " = "+strvalue+
                                               " is not a number!!");
                }
        }

        
        public void stateChanged(ChangeEvent e) 
        {
		m_f_BBScale_Value=((float)(m_sl_BBScale.getValue()))/10;
		m_bt_UserBBScale.setText(m_strobprm_BBScale_Des+"="+m_f_BBScale_Value);
		
		NumberFormat Num_Format=NumberFormat.getNumberInstance();
        	//	fValue=Num_Format.parse( ((JTextField)(e.getSource())).getText() ).floatValue();

		try{
		m_f_XMIN=Num_Format.parse(m_tf_XMIN.getText()).floatValue();
		m_f_XMAX=Num_Format.parse(m_tf_XMAX.getText()).floatValue();
		m_f_YMIN=Num_Format.parse(m_tf_YMIN.getText()).floatValue();
		m_f_YMAX=Num_Format.parse(m_tf_YMAX.getText()).floatValue();
		m_f_ZMIN=Num_Format.parse(m_tf_ZMIN.getText()).floatValue();
		m_f_ZMAX=Num_Format.parse(m_tf_ZMAX.getText()).floatValue();
		
		}
		catch(ParseException ex)
		    {}
	}
	
	/////////////////////////////////////////////////////////////////////////////////////////
        //
        //	Private Methods
        //
        //		getComponentforUserBB
        //		getBBScale
        //		updateValues_for_UserBB
        //		updateValues_for_BBScale
        //
        //		parseValues_for_UserBB
        //		parseValues_for_BBScale
        //		
        //		packBBTextFields
        //		BBTextFiled_SetEnabled (distable all text field)
        //		setEventHandler
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        
        	
	/////////////////////////////////////////////////////////////////////////////////

	//
	//
	//
	//	UserBB Methods
	//
	//
	//
	/////////////////////////////////////////////////////////////////////////////////
	
        /**
         *This method sets componnets and values back to default state.
         *If necessarty components are created here
         */
        private void updateValues_for_UserBB()
        {
       		if( m_tf_XMIN==null ) //or check all of them are null
	        {        	   
	                //create fields
	                m_tf_XMIN=new JTextField(6); m_tf_XMAX=new JTextField(6); 
	                m_tf_YMIN=new JTextField(6); m_tf_YMAX=new JTextField(6); 
	                m_tf_ZMIN=new JTextField(6); m_tf_ZMAX=new JTextField(6); 
		}
	
	        //set values
	        m_tf_XMIN.setText(m_f_XMIN+""); m_tf_XMAX.setText(m_f_XMAX+"");
	        m_tf_YMIN.setText(m_f_YMIN+""); m_tf_YMAX.setText(m_f_YMAX+"");
	        m_tf_ZMIN.setText(m_f_ZMIN+""); m_tf_ZMAX.setText(m_f_ZMAX+"");
	        BBTextFiled_SetEnabled(bUseUserDefinedBB);
	         
	        if( m_bt_UserBB==null )//create toggle button
	        {
                	m_bt_UserBB
                	=new JToggleButton(m_strobprm_BB_Des,new ImageIcon("images/Stop16.gif"));
                }
                m_bt_UserBB.setSelected(bUseUserDefinedBB);
	}
	
        private boolean parseValues_for_UserBB(String [] strValues) throws Exception
        {
        	//see strValues as a file name
        	String [] values;
        	//input map
        	values=iOBPRMUtility.getOptionValue(strValues[0], m_strobprm_BB_CTag);
        	if( values!=null && values.length==7
        	    && values[0].compareTo(m_strobprm_BB_CTag)==0 )
        	{
        		m_f_XMIN=Float.parseFloat(values[1]);
        		m_f_XMAX=Float.parseFloat(values[2]);
        		m_f_YMIN=Float.parseFloat(values[3]);
        		m_f_YMAX=Float.parseFloat(values[4]);
        		m_f_ZMIN=Float.parseFloat(values[5]);
        		m_f_ZMAX=Float.parseFloat(values[6]);
        		
        		bUseUserDefinedBB=true;
        	}
        	
        	return true;
        }
        
        /**
          *This method creates button and fields for user deinfed bounding box
          */
        private Component getComponentforUserBB()
        {
        	//overall container
        	JPanel bb_Panel=new JPanel();
         	bb_Panel.setLayout(new BoxLayout(bb_Panel, BoxLayout.Y_AXIS));
                
		m_bt_UserBB.setFont(new Font("times", Font.BOLD,10));
                m_bt_UserBB.setPreferredSize(new Dimension(90,20));
                m_bt_UserBB.addActionListener(new ActionListener() 
        	{
            		public void actionPerformed(ActionEvent e) 
            		{
                		BBTextFiled_SetEnabled
                		(((JToggleButton)(e.getSource())).isSelected());
            		}
        	});

                //set fonts
                m_tf_XMIN.setFont(new Font("times", Font.BOLD,10));
                m_tf_XMAX.setFont(new Font("times", Font.BOLD,10));
                m_tf_YMIN.setFont(new Font("times", Font.BOLD,10));
                m_tf_YMAX.setFont(new Font("times", Font.BOLD,10));
                m_tf_ZMIN.setFont(new Font("times", Font.BOLD,10));
                m_tf_ZMAX.setFont(new Font("times", Font.BOLD,10));


                //add
                bb_Panel.add(m_bt_UserBB);
                bb_Panel.add(packBBTextFields());

                //return
                return bb_Panel;
	}
	
	/////////////////////////////////////////////////////////////////////////////////
	//
	//
	//
	//	BBScale Methods
	//
	//
	//
	/////////////////////////////////////////////////////////////////////////////////
        /**
         *This method sets componnets and values back to default state.
         *If necessarty components are created here
         */
        private void updateValues_for_BBScale()
        {
                if( m_bt_UserBBScale==null )//create toggle button
                {

                	m_bt_UserBBScale
                	=new JToggleButton(m_strobprm_BBScale_Des,new ImageIcon("images/Stop16.gif"));
                }	
                m_bt_UserBBScale.setSelected(bUseBBScale);

                if( m_sl_BBScale==null )
	        {	
	                /*0~10, each step is 0.1*/
	                m_sl_BBScale=
	                new JSlider(JSlider.HORIZONTAL,1,499,(int)(m_f_BBScale_Value*10));
                }

                m_sl_BBScale.setValue((int)(m_f_BBScale_Value*10));
                m_sl_BBScale.setEnabled(bUseBBScale);
	}
	
	private boolean parseValues_for_BBScale(String [] strValues) throws Exception
        {
        	//see strValues as a file name
        	String [] values;
        	
        	values=iOBPRMUtility.getOptionValue(strValues[0], m_strobprm_BBScale_CTag);
        	if( values!=null && values.length==2
        	    && values[0].compareTo(m_strobprm_BBScale_CTag)==0 )
        	{
        		m_f_BBScale_Value=Float.parseFloat(values[1]);
        		bUseBBScale=true;
        	}
        	
        	return true;	
        }

	/**
	 *Create components for bounding box scale, such as slider..
	 */
	private Component getBBScale()
	{
		/////////////////////////////////////
                //create toggle button for bb scale
                m_bt_UserBBScale=
                new JToggleButton
                (m_strobprm_BBScale_Des+" = "+m_f_BBScale_Value
                ,new ImageIcon("images/Stop16.gif"));
                m_bt_UserBBScale.setFont(new Font("times", Font.BOLD,10));
                m_bt_UserBBScale.setPreferredSize(new Dimension(90,20));
                m_bt_UserBBScale.addActionListener(new ActionListener()
        	{
            		public void actionPerformed(ActionEvent e)
            		{
                		m_sl_BBScale.setEnabled
                		(((JToggleButton)(e.getSource())).isSelected());
            		}
        	});

                //scale slider
                m_sl_BBScale.addChangeListener(this);
                m_sl_BBScale.setPreferredSize(new Dimension(150,20));

                //add scale components
		JPanel slider_Panel=new JPanel();
		slider_Panel.setLayout(new GridLayout(2,1));
		slider_Panel.add(m_bt_UserBBScale);
		slider_Panel.add(m_sl_BBScale);
		
		JPanel scale_Panel=new JPanel();
		scale_Panel.add(slider_Panel);

                return scale_Panel;
	}
	
		
	/////////////////////////////////////////////////////////////////////////////////
	//
	//
	//
	//	Help Methods
	//
	//
	//
	/////////////////////////////////////////////////////////////////////////////////
	
	/**
	 *This method creates labels and make labels looked "GOOD"
	 */
	private Component packBBTextFields()
	{
		//create labels
                JLabel lb_empty = new JLabel();	lb_empty.setFont(new Font("times", Font.BOLD,10));
                JLabel lb_MIN = new JLabel("min"); lb_MIN.setFont(new Font("times", Font.BOLD,10));
                JLabel lb_MAX = new JLabel("max"); lb_MAX.setFont(new Font("times", Font.BOLD,10));
                JLabel lb_X = new JLabel("x");	lb_X.setFont(new Font("times", Font.BOLD,10));
                JLabel lb_Y = new JLabel("y");	lb_Y.setFont(new Font("times", Font.BOLD,10));
                JLabel lb_Z = new JLabel("z");	lb_Z.setFont(new Font("times", Font.BOLD,10));
                
                //Panel for lables
                JPanel bbLabel_Panel=new JPanel();
                bbLabel_Panel.setLayout(new GridLayout(4,1));
                bbLabel_Panel.add(lb_empty); bbLabel_Panel.add(lb_X); 
                bbLabel_Panel.add(lb_Y); bbLabel_Panel.add(lb_Z); 
                
                //Panel for bb text fileds
                JPanel bbtext_Panel=new JPanel();
                bbtext_Panel.setLayout(new GridLayout(4,2));
                bbtext_Panel.add(lb_MIN); bbtext_Panel.add(lb_MAX); 
                bbtext_Panel.add(m_tf_XMIN); bbtext_Panel.add(m_tf_XMAX);
                bbtext_Panel.add(m_tf_YMIN); bbtext_Panel.add(m_tf_YMAX);
                bbtext_Panel.add(m_tf_ZMIN); bbtext_Panel.add(m_tf_ZMAX);
                
                //Panel for bb text and label 
                JPanel bbtext_label_Panel=new JPanel();
                bbtext_label_Panel.add(bbLabel_Panel);
                bbtext_label_Panel.add(bbtext_Panel);
                
                return bbtext_label_Panel;
	}

        /**
         *Set Listeners for components.
         */
        private void setEventHandler()
        {
		if( m_tf_XMIN!=null )
			m_tf_XMIN.addFocusListener(this);
		if( m_tf_XMAX!=null )
			m_tf_XMAX.addFocusListener(this);
		if( m_tf_YMIN!=null )
			m_tf_YMIN.addFocusListener(this);
		if( m_tf_YMAX!=null )
			m_tf_YMAX.addFocusListener(this);
		if( m_tf_ZMIN!=null )
			m_tf_ZMIN.addFocusListener(this);
		if( m_tf_ZMAX!=null )
			m_tf_ZMAX.addFocusListener(this);	
        }
        
        /**
         *Set tool tips of components in this class
         */
        private void setToolTips()
	{
		if( m_tf_XMIN!=null )
			m_tf_XMIN.setToolTipText("Maxmum X Value");
		if( m_tf_XMAX!=null )
			m_tf_XMAX.setToolTipText("Minimum X Value");
		if( m_tf_YMIN!=null )
			m_tf_YMIN.setToolTipText("Maxmum Y Value");
		if( m_tf_YMAX!=null )
			m_tf_YMAX.setToolTipText("Minimum Y Value");
		if( m_tf_ZMIN!=null )
			m_tf_ZMIN.setToolTipText("Maxmum Z Value");
		if( m_tf_ZMAX!=null )
			m_tf_ZMAX.setToolTipText("Minimum Z Value");
		if( m_bt_UserBB!=null )
			m_bt_UserBB.setToolTipText("Create an Environment Boudning Box");
		if( m_bt_UserBBScale!=null )
			m_bt_UserBBScale.setToolTipText("Scale default/user created Boudning Box");
		if( m_sl_BBScale!=null )
			m_sl_BBScale.setToolTipText("Scale from 0.1~9.9");
	}
        
	private void BBTextFiled_SetEnabled(boolean bEnabled)
	{
		if( m_tf_XMIN!=null )
			m_tf_XMIN.setEnabled(bEnabled);
		if( m_tf_XMAX!=null )
			m_tf_XMAX.setEnabled(bEnabled);
		if( m_tf_YMIN!=null )
			m_tf_YMIN.setEnabled(bEnabled);
		if( m_tf_YMAX!=null )
			m_tf_YMAX.setEnabled(bEnabled);
		if( m_tf_ZMIN!=null )
			m_tf_ZMIN.setEnabled(bEnabled);
		if( m_tf_ZMAX!=null )
			m_tf_ZMAX.setEnabled(bEnabled);
	}
 }
 
