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
//package dsmft.OBPRM.iOBPRM;

////////////////////////////////////////////////////////////////////////
//Imports
import java.io.File;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;
import java.text.*;

final public class 
iEnvironment extends iOptionPanel
{
                      
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //      Class data member
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        final private String m_str_env_Iconname=iOBPRMUtility.getImageDirectory()+"env_icon.gif";
        iEvnOBPRM m_envOBPRM;
        iEvnQUERY m_envQUERY;
            
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Constructor
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        public iEnvironment()
        {
                loadIcon(m_str_env_Iconname);
                m_envOBPRM = new iEvnOBPRM(this);
                m_envQUERY = new iEvnQUERY(this);
                
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
                return "Configure Environment related options for OBPRM";
        }
        
        /**
	 *Set Options in this class to default values.
 	 */
	public void setDefaultValues()
	{
		m_envOBPRM.setDefaultValues();
		m_envQUERY.setDefaultValues();
	}
        
        
        public void updateValues()
        {
		m_envOBPRM.updateValues();
		m_envQUERY.updateValues();
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
        	if( strValues[0].endsWith("qry") )
        	{
        		if( m_envQUERY.parseValues(strValues)==false )
	        		return false;
	        }
	        else //mkmp
	        {
	   	       	if( m_envOBPRM.parseValues(strValues)==false )
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
         	return m_envOBPRM.toCommand();       
        }
        
        public String toQryCommand()
        {
        	String Command=m_envQUERY.toCommand()+" ";
        	String strDir_EnvName=m_envOBPRM.m_tf_EnvDirName.getText();
        	File current=iOBPRMUtility.getCurrentProgramDirectory();
        	if( current!=null && current.getPath().compareTo(strDir_EnvName)!=0 ) 
        		Command+=(m_envOBPRM.m_strDir_EnvName_CTag+" "+strDir_EnvName+ " ");
        	
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
		if( m_envOBPRM.validateOptions()==false )
			return false;
		return true;
        }
        
        public boolean validateQryOptions()
        {
		if( m_envQUERY.validateOptions()==false )
			return false;
		return true;
        }
        
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         */
        protected Component getComponent()
        {               
                //basic Panel
                JPanel env_basic_Panel=new JPanel();
                env_basic_Panel.setLayout(new BoxLayout(env_basic_Panel, BoxLayout.X_AXIS));
                
                //add component to basic Panel
                env_basic_Panel.add(m_envOBPRM.getComponent());
                env_basic_Panel.add(m_envQUERY.getComponent());
                
                //return
                return env_basic_Panel;
        }
}
 
final class iEvnOBPRM implements CaretListener, ActionListener
{
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Class data member
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        final private String m_str_Default_TextFieldValue="<not specified>";
        private Component m_Parent;
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //      Class data member : for obprm command
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        //for defaul command name
        final private String m_strcmd_Des="Command";
        private String  m_strcmd_Name;
        private JTextField m_tf_cmdName;
        private JButton m_bt_cmdOpen;   //invoke file dialog
        
        //for defaul file name
        final private String m_strobprm_DefaultName_CTag="-f";	//command tag
        final private String m_strobprm_DefaultName_Des="Default filename";
        private String m_strobprm_DefaultName;
        private JTextField m_tf_obprmDefaultName;
        private JButton m_bt_obprmDefaulOpen;   //invoke file dialog
        
        //use default or not
        private boolean m_bobprmDefault;
        private JCheckBox m_cb_obprmDefault;    //Check this to used default filename
                       
        //for environment file name
        final private String m_strFile_EnvName_CTag="-envFile";//command tag
        final private String m_strFile_EnvName_Des="Input Environment filename";
        private String m_strFile_EnvName;
        private JButton m_bt_EnvOpen;   //invoke file dialog
        private JTextField m_tf_EnvName;
        
        //for map file name
        final private String m_strFile_MapName_CTag="-outmapFil";//command tag
        final private String m_strFile_MapName_Des="Ouput RoadMap filename";
        private String m_strFile_MapName;
        private JButton m_bt_MapOpen;   //invoke file dialog
        private JTextField m_tf_MapName;
        
        //for environmental data directory name. (where models are!!)
        final public String m_strDir_EnvName_CTag="-descDir";	//command tag
        final public String m_strDir_EnvName_Des="Geometric Data Directory";
        public String m_strDir_EnvName;
        private JButton m_bt_EnvDirOpen;        //invoke file dialog
        public JTextField m_tf_EnvDirName;
        
        //for additional command name
        final private String m_strOption_Des="Additional Options";
        private String  m_strOption_Name;
        private JTextField m_tf_OptionName;
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //	iEvnOBPRM
        /////////////////////////////////////////////////////////////////////////////////////////         
        iEvnOBPRM( Component parent )
        {
        	m_Parent=parent;
	}

        /////////////////////////////////////////////////////////////////////////////////////////
        //		public methods (most of them are following same rules of iOptionalPanel
        //
        //              - setDefaultValues
        //              - updateValues
        //              - parseValues
        //              - validateOptions
        //              - toCommand
        //              - getComponent
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
	 *Set Options in this class to default values.
 	 */
	public void setDefaultValues()
	{
		m_strcmd_Name="obprm";
		m_strOption_Name="";
		
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
        
        ///update gui according to internal values
        public void updateValues()
        {
        	//create component if necessary        
        	if( !isComponentCreated() )
        		create_Components();
        	
        	//cmd	
        	m_tf_cmdName.setText(m_strcmd_Name);
        	
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
                
                //options
                m_tf_OptionName.setText(m_strOption_Name);
        }
        
        /**
         *Read OBPRM Environmental related data from given file.
         *@note assume file exist
         */
        public boolean parseValues(String [] strValues) throws Exception
        {
   		//view strValues as a file name
        	String [] values;
		String [] temp;

		// System.out.println("\n\n\n Values[] before " + values);


        	
        	//dir
		temp=iOBPRMUtility.getOptionValue(strValues[0],"");
        	
		System.out.println("\ntemp[0] = " + temp[0]);// + "\ntemp[1] = " + temp[1] + "\ntemp[2] = " + temp[2]);
		

		values=iOBPRMUtility.getOptionValue(strValues[0], m_strDir_EnvName_CTag);

		//System.out.println("\n\n temp[0] " + temp[0]+ "\n\n");

        	//System.out.println("\n\n\n Values[0] after " + values[0]);
		//System.out.println("\nValues[1] after " + values[1]);



		if( values!=null && values.length==2 && 
        	    values[0].compareTo(m_strDir_EnvName_CTag)==0 )
        	{
        		m_strDir_EnvName=values[1];
			m_strcmd_Name=temp[0];
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

        /**
	 * validate user input value
         */
        public boolean validateOptions()
        {

        	m_strcmd_Name = m_tf_cmdName.getText();
        	if( m_strcmd_Name.length()==0 )
        		return false;
        	
        	if( m_bobprmDefault ){
	        	m_strobprm_DefaultName=m_tf_obprmDefaultName.getText();
			if( m_strobprm_DefaultName.compareTo(m_str_Default_TextFieldValue)==0 )
				return false;
        	}
        	else{
  	        	m_strFile_EnvName=m_tf_EnvName.getText();
			if( m_strFile_EnvName.compareTo(m_str_Default_TextFieldValue)==0 )
				return false;
        
	        	m_strFile_MapName=m_tf_MapName.getText();
			if( m_strFile_MapName.compareTo(m_str_Default_TextFieldValue)==0 )
				return false;      	
        	}

        	m_strDir_EnvName=m_tf_EnvDirName.getText();
        	m_strOption_Name=m_tf_OptionName.getText();     
		return true;
	}

	///convert user input values to obprm commands
        public String toCommand()
        {
        	String Command="";

        	//if use default
        	if( m_bobprmDefault ){
			Command+=(m_strobprm_DefaultName_CTag+" "+m_strobprm_DefaultName+" ");
		}
        	else
        	{
        		Command+=(m_strFile_EnvName_CTag+" "+m_strFile_EnvName+" ");
			Command+=(m_strFile_MapName_CTag+" "+m_strFile_MapName+" ");
		}		
        	
        	File current=iOBPRMUtility.getCurrentProgramDirectory();
        	if( current!=null && current.getPath().compareTo(m_strDir_EnvName)!=0 ) 
        		Command+=(m_strDir_EnvName_CTag+" "+m_strDir_EnvName+ " ");
        		
   		Command+=(" "+m_strOption_Name);
        	return m_strcmd_Name+" "+Command;
        }
        
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         */
        protected Component getComponent()
        {
                //init Panel
                JPanel env_obprm_Panel=new JPanel();
                env_obprm_Panel.setLayout(new BoxLayout(env_obprm_Panel, BoxLayout.Y_AXIS));
                env_obprm_Panel.setBorder(BorderFactory.createTitledBorder("<obprm> environment"));
                
                //add event hadlere
                m_cb_obprmDefault.addActionListener(new ActionListener(){ 
                public void actionPerformed(ActionEvent e){update_DefaultState();} });
                
                env_obprm_Panel.add(iOBPRMUtility.packFileComponent
                (true,m_strcmd_Des,m_tf_cmdName,m_bt_cmdOpen));
                
                env_obprm_Panel.add(iOBPRMUtility.packFileComponent	
                (true,m_strobprm_DefaultName_Des,m_tf_obprmDefaultName,m_bt_obprmDefaulOpen));
                
                env_obprm_Panel.add(iOBPRMUtility.packFileComponent
                (true,m_strDir_EnvName_Des,m_tf_EnvDirName,m_bt_EnvDirOpen));
                
                env_obprm_Panel.add(m_cb_obprmDefault);
                     
                env_obprm_Panel.add(iOBPRMUtility.packFileComponent
                (true,m_strFile_EnvName_Des,m_tf_EnvName,m_bt_EnvOpen));
                
                env_obprm_Panel.add(iOBPRMUtility.packFileComponent
                (false,m_strFile_MapName_Des,m_tf_MapName,m_bt_MapOpen));

                env_obprm_Panel.add(iOBPRMUtility.packFileComponent
                (true,m_strOption_Des,m_tf_OptionName,null));
                
                setEventHandler();	//set listeners
   
                //return
                return env_obprm_Panel;
        }
        
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //	public
        //      Event Listener :
        //
        //		actionPerformed (for Action Listener)
        //		caretUpdate	(for Caret Listener)
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *When user typed in default fields for obprm or query.
         *the file name for other files, such as env, map, path, query files
         *should be updated.
         *@see update_Filename
         */
        public void caretUpdate(CaretEvent e) {
        	if( e.getSource()==m_tf_obprmDefaultName && m_cb_obprmDefault.isSelected())
                	update_Filename(m_tf_obprmDefaultName.getText());
        }

        /**
         *Handle events form buttons, most of them are open a file chooser
         *and choose a file or dir.
         *@see iOBPRMUtility.getFilename2Open and iOBPRMUtility.getDirname
         */
        public void actionPerformed(ActionEvent e) 
        {
        	//obprm fefault file
        	if( e.getSource()==m_bt_obprmDefaulOpen)
                {
                	String strDefaultName=
                	iOBPRMUtility.getFilename2Open(m_Parent,"Open Default FileName",null);
                        if( strDefaultName!=null )
                        {
                        	//truncate extension of filename
                                int index=strDefaultName.lastIndexOf('.');
                                if( index>0 )
                                	m_tf_obprmDefaultName.setText(strDefaultName.substring( 0, index) );
                                else
                                	m_tf_obprmDefaultName.setText(strDefaultName);
                                update_Filename(m_tf_obprmDefaultName.getText());
                        }
                 }
                 //data directory
                 else if( e.getSource()==m_bt_EnvDirOpen)
                 {
 	                String strDirName=
 	                iOBPRMUtility.getDirname(m_Parent,"Open Geometric Data File(s) Directory");
                        if( strDirName!=null )
                        {
         	                m_tf_EnvDirName.setText(strDirName);
                        }
                 }
                 //env file open
                 else if( e.getSource()==m_bt_EnvOpen )
                 {
                        String [] filter=new String[1]; filter[0]="env";
                        String strEnvName=
                        iOBPRMUtility.getFilename2Open(m_Parent,"Open Environemnt FileName",filter);
                        if( strEnvName!=null )
                        	m_tf_EnvName.setText(strEnvName);
                 }
                 //map file open
                 else if( e.getSource()==m_bt_MapOpen )
                 {
                 	String [] filter=new String[1]; filter[0]="map";
                        String strMapName=
                        iOBPRMUtility.getFilename2Open(m_Parent,"Open Map FileName",filter);
                        if( strMapName==null ) return;
                        m_tf_MapName.setText(strMapName);
                 }
                 else if( e.getSource()==m_bt_cmdOpen )
                 {
                        String strCmdName=
                        iOBPRMUtility.getFilename2Open(m_Parent,"Open Command FileName",null);
                        if( strCmdName==null ) return;
                        m_tf_cmdName.setText(strCmdName);
                 }
	}

        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //		Helper
        //
        //		isComponentCreated_for_obprm
        //		create_Components_for_obprm
        //              iOBPRMUtility.packFileComponent
        //		setEventHandler
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        private boolean isComponentCreated()
        {
        	if( m_tf_obprmDefaultName==null || m_bt_MapOpen==null )
        		return false;
        	return true;
        }
        
        private void
        create_Components()
        {
        	int tfLength=20; //length for textfield
        	ImageIcon open_Icon=new ImageIcon(iOBPRMUtility.getImageDirectory()+"Open16.gif");
                
                //for command name
                m_tf_cmdName=new JTextField(tfLength);
                m_bt_cmdOpen=new JButton(open_Icon);
                
     		//for defaul file name
                m_tf_obprmDefaultName=new JTextField(tfLength);
               	m_bt_obprmDefaulOpen=new JButton(open_Icon);
               	
               	//for switch
                m_cb_obprmDefault=new JCheckBox("use default");
                m_cb_obprmDefault.setFont(new Font("times", Font.BOLD,10));
                
		//for data diectory
                m_tf_EnvDirName=new JTextField(tfLength);
                m_bt_EnvDirOpen=new JButton(open_Icon);

                //for defaul env name
                m_tf_EnvName=new JTextField(tfLength);
                m_bt_EnvOpen=new JButton(open_Icon);
                
                //for defaul map name
	        m_tf_MapName=new JTextField(tfLength);
        	m_bt_MapOpen=new JButton(open_Icon);
        	
        	//for other options
        	m_tf_OptionName=new JTextField(tfLength);
        }
        
        /**
         *Set Listeners for components.
         */
        private void setEventHandler()
        {
                if( m_tf_obprmDefaultName!=null )
                        m_tf_obprmDefaultName.addCaretListener(this);
        
                if( m_bt_obprmDefaulOpen!=null )
                        m_bt_obprmDefaulOpen.addActionListener(this);
                
                 if( m_bt_EnvDirOpen!=null )
                        m_bt_EnvDirOpen.addActionListener(this);
                        
                if( m_bt_EnvOpen!=null )
                        m_bt_EnvOpen.addActionListener(this);
                                
                if( m_bt_MapOpen!=null )
                        m_bt_MapOpen.addActionListener(this);
                
                if( m_bt_cmdOpen!=null )
                        m_bt_cmdOpen.addActionListener(this);
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //		Update 
        //
        //              update_Filename
        //		update_DefaultState
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *Update map/env filename for obprm command by given name.
         */
        private void update_Filename(String defaultName){
                String appendixMap=".map";
                String appendixEnv=".env";
              
		// Adding
		m_strobprm_DefaultName=defaultName;
          
                if( defaultName.length()==0 )
                {
                        appendixMap="";
                        appendixEnv="";
                }
                        
                m_tf_EnvName.setText(defaultName+appendixEnv);
                m_tf_MapName.setText(defaultName+appendixMap);  
        }
        
        /**
         *Switch between deafult mode and file specified mode for obprm command.
         */
        private void update_DefaultState()
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
}

 
final class iEvnQUERY implements CaretListener, ActionListener
{
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Class data member
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        final private String m_str_Default_TextFieldValue="<not specified>";
        private Component m_Parent;
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //      Class data member : for obprm command
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        //for defaul command name
        final private String m_strcmd_Des="Command";
        private String  m_strcmd_Name;
        private JTextField m_tf_cmdName;
        private JButton m_bt_cmdOpen;   //invoke file dialog
        
        //for defaul file name
        final private String m_strquery_DefaultName_CTag="-f";	//command tag
        final private String m_strquery_DefaultName_Des="Default filename";
        private String  m_strquery_DefaultName;
        private JTextField m_tf_queryDefaultName;
        private JButton m_bt_queryDefaulOpen;   //invoke file dialog
        
        private boolean m_bqueryDefault;
        private JCheckBox m_cb_queryDefault;    //Check this to used default filename
        
        //for inmap file name
        final private String m_strFile_inMapName_CTag="-inmapFile";//command tag
        final private String m_strFile_inMapName_Des="Input RoadMap filename";
        private String  m_strFile_inMapName;
        private JButton m_bt_inMapOpen; //invoke file dialog
        private JTextField m_tf_inMapName;
        
        //for query file name
        final private String m_strFile_QryName_CTag="-queryFile";//command tag
        final private String m_strFile_QryName_Des="Input Query filename";
        private String  m_strFile_QryName;
        private JButton m_bt_QryOpen;   //invoke file dialog
        private JTextField m_tf_QryName;
        
        //for path file name
        final private String m_strFile_PathName_CTag="-pathFile";//command tag
        final private String m_strFile_PathName_Des="Output Path filename";
        private String  m_strFile_PathName;
        private JButton m_bt_PathOpen;  //invoke file dialog
        private JTextField m_tf_PathName;
        
        //for more options
        final private String m_strOption_Des="Additional Options";
        private String  m_strOption_Name;
        private JTextField m_tf_OptionName;
        
        /////////////////////////////////////////////////////////////////////////////////////////
     	//	iEvnQUERY
     	/////////////////////////////////////////////////////////////////////////////////////////
        iEvnQUERY( Component parent )
        {
        	m_Parent=parent;
	}
	
        /////////////////////////////////////////////////////////////////////////////////////////
        //		public methods (most of them are following same rules of iOptionalPanel
        //
        //              - setDefaultValues
        //              - updateValues
        //              - parseValues
        //              - validateOptions
        //              - toCommand
        //              - getComponent
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
	 *Set Options in this class to default values.
 	 */
	public void setDefaultValues()
	{
		m_strcmd_Name="query";        
		m_strOption_Name="";
		
          	m_strquery_DefaultName=m_str_Default_TextFieldValue;
        	m_bqueryDefault=true;
                m_strFile_inMapName=m_str_Default_TextFieldValue;
                m_strFile_QryName=m_str_Default_TextFieldValue;
                m_strFile_PathName=m_str_Default_TextFieldValue;
	}
        
        ///update gui according to internal values
        public void updateValues()
        {
         	//create component if necessary        
        	if( !isComponentCreated() )
        		create_Components();
        		
        	//cmd	
        	m_tf_cmdName.setText(m_strcmd_Name);
        	
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
                                        
        	//for more options
        	m_tf_OptionName.setText(m_strOption_Name); 
        }
        
        /**
         *Read OBPRM Environmental related data from given file.
         *@note assume file exist
         */
        public boolean parseValues(String [] strValues) throws Exception
        {
        	//view strValues as a file name
        	String [] values;
		String [] cmdValue;
        	//default
		cmdValue=iOBPRMUtility.getOptionValue(strValues[0],"");
        	values=iOBPRMUtility.getOptionValue(strValues[0], m_strquery_DefaultName_CTag);
        	if( values!=null && values.length==2 
        	    && values[0].compareTo(m_strquery_DefaultName_CTag)==0 )
        	{
        		m_strquery_DefaultName=values[1];
			m_strcmd_Name=cmdValue[0];
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

        /**
	 * validate user input value
         */
        public boolean validateOptions()
        {
        	m_strcmd_Name = m_tf_cmdName.getText();
        	if( m_strcmd_Name.length()==0 )
        		return false;
        		
     		//if use default
        	if( m_bqueryDefault )
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
        	
        	m_strOption_Name=m_tf_OptionName.getText(); 
        	return true;
	}

	///convert user input values to obprm commands
        public String toCommand()
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
        
        	return m_strcmd_Name+" "+m_strOption_Name+" "+Command;
        }
        
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         */
        protected Component getComponent()
        {
                 //init Panel
                JPanel env_query_Panel=new JPanel();
                env_query_Panel.setLayout(new BoxLayout(env_query_Panel, BoxLayout.Y_AXIS));
                env_query_Panel.setBorder(BorderFactory.createTitledBorder("<query> environment"));
                
                //add event hadlere
                m_cb_queryDefault.addActionListener(new ActionListener(){ 
                public void actionPerformed(ActionEvent e){update_DefaultState();} });
                
                //Add to panel
                env_query_Panel.add(iOBPRMUtility.packFileComponent
                (true,m_strcmd_Des,m_tf_cmdName,m_bt_cmdOpen));
                
                env_query_Panel.add(iOBPRMUtility.packFileComponent
                (true,m_strquery_DefaultName_Des,m_tf_queryDefaultName,m_bt_queryDefaulOpen));
                
                env_query_Panel.add(m_cb_queryDefault);
                     
                env_query_Panel.add(iOBPRMUtility.packFileComponent
                (true,m_strFile_inMapName_Des,m_tf_inMapName,m_bt_inMapOpen));
                     
                env_query_Panel.add(iOBPRMUtility.packFileComponent
                (true,m_strFile_QryName_Des,m_tf_QryName,m_bt_QryOpen));
                     
                env_query_Panel.add(iOBPRMUtility.packFileComponent
                (false,m_strFile_PathName_Des,m_tf_PathName,m_bt_PathOpen));
                
                env_query_Panel.add(iOBPRMUtility.packFileComponent
                (true,m_strOption_Des,m_tf_OptionName,null));
                

                setEventHandler();	//set listeners
                
                //return
                return env_query_Panel;
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //	public
        //      Event Listener :
        //
        //		actionPerformed (for Action Listener)
        //		caretUpdate	(for Caret Listener)
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *When user typed in default fields for obprm or query.
         *the file name for other files, such as env, map, path, query files
         *should be updated.
         *@see update_Filename
         */
        public void caretUpdate(CaretEvent e)
        {
                if( e.getSource()==m_tf_queryDefaultName && m_cb_queryDefault.isSelected())
                	update_Filename(m_tf_queryDefaultName.getText());
        }

        
        /**
         *Handle events form buttons, most of them are open a file chooser
         *and choose a file or dir.
         *@see iOBPRMUtility.getFilename2Open and iOBPRMUtility.getDirname
         */
        public void actionPerformed(ActionEvent e) 
        {
        	 //query default file
                 if( e.getSource()==m_bt_queryDefaulOpen)
                 {
                 	String strDefaultName=
                 	iOBPRMUtility.getFilename2Open(m_Parent,"Open Default FileName",null);
                        if( strDefaultName!=null )
                        {
                        	//truncate extension of filename
                                int index=strDefaultName.lastIndexOf('.');
                                if( index>0 )
                                	m_tf_queryDefaultName.setText(strDefaultName.substring( 0, index) );
                                else
                                        m_tf_queryDefaultName.setText(strDefaultName);
                                update_Filename(m_tf_queryDefaultName.getText());
                        }
                 }
                 else if( e.getSource()==m_bt_QryOpen )
                 {
                 	String [] filter=new String[1]; filter[0]="query";
                        String strQryName=
                        iOBPRMUtility.getFilename2Open(m_Parent,"Open Query FileName",filter);
                        if( strQryName!=null )
                        	m_tf_QryName.setText(strQryName);
                 }
                 else if( e.getSource()==m_bt_PathOpen )
                 {
                 	String [] filter=new String[1]; filter[0]="path";
                        String strPathName=
                        iOBPRMUtility.getFilename2Open(m_Parent,"Open Path FileName",filter);
                        if( strPathName!=null )
                        	m_tf_PathName.setText(strPathName);
                 }
                 //map file open
                 else if( e.getSource()==m_bt_inMapOpen )
                 {
                 	String [] filter=new String[1]; filter[0]="map";
                        String strMapName=
                        iOBPRMUtility.getFilename2Open(m_Parent,"Open Map FileName",filter);
                        if( strMapName==null ) return;
                        m_tf_inMapName.setText(strMapName);
                 }
                 //cmd
                 else if( e.getSource()==m_bt_cmdOpen )
                 {
                        String strCmdName=
                        iOBPRMUtility.getFilename2Open(m_Parent,"Open Command FileName",null);
                        if( strCmdName==null ) return;
                        m_tf_cmdName.setText(strCmdName);
                 }
	}

        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //		Helper
        //
        //		isComponentCreated
        //		create_Components
        //		setEventHandler
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        private boolean isComponentCreated()
        {
        	if( m_tf_queryDefaultName==null || m_bt_PathOpen==null )
        		return false;
        	return true;
        }
        
        private void create_Components()
        {
        	int tfLength=20; //length for textfield
        	ImageIcon open_Icon=new ImageIcon(iOBPRMUtility.getImageDirectory()+"Open16.gif"); //icon for file open button.
                
                                
                //for command name
                m_tf_cmdName=new JTextField(tfLength);
                m_bt_cmdOpen=new JButton(open_Icon);
                
                //for defaul file name
                m_tf_queryDefaultName=new JTextField(tfLength);
                m_bt_queryDefaulOpen=new JButton(open_Icon);

                //switch defaul file name
                m_cb_queryDefault=new JCheckBox("use default");
                m_cb_queryDefault.setFont(new Font("times", Font.BOLD,10));

		//input map file
		m_tf_inMapName=new JTextField(tfLength);
                m_bt_inMapOpen=new JButton(open_Icon);
                
                //input query file
                m_tf_QryName=new JTextField(tfLength);
                m_bt_QryOpen=new JButton(open_Icon);
                     
                //output path file
                m_tf_PathName=new JTextField(tfLength);
                m_bt_PathOpen=new JButton(open_Icon);
                //for other options
        	m_tf_OptionName=new JTextField(tfLength);
        }
        
        /**
         *Set Listeners for components.
         */
        private void setEventHandler()
        {
        	if( m_bt_cmdOpen!=null )
                        m_bt_cmdOpen.addActionListener(this);

		if( m_tf_queryDefaultName!=null )
                        m_tf_queryDefaultName.addCaretListener(this);
                        
                if( m_bt_queryDefaulOpen!=null )
                        m_bt_queryDefaulOpen.addActionListener(this);

		if( m_bt_inMapOpen!=null )
			m_bt_inMapOpen.addActionListener(this);
			
		if( m_bt_QryOpen!=null )
			m_bt_QryOpen.addActionListener(this);
			
		if( m_bt_PathOpen!=null )
			m_bt_PathOpen.addActionListener(this);
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //		Update 
        //
        //              update_Filename
        //		update_DefaultState
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
      	/**
         *Update map/path/query filename for query command by given name.
         */
        private void update_Filename(String defaultName){
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
        
        /**
         *Switch between deafult mode and file specified mode for query command.
         */
        private void update_DefaultState()
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
}
