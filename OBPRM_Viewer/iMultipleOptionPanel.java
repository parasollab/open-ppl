/**
 *This file contains class iMultipleOptionPanel which contains
 *a control panel to create/delete/modify created options.
 *
 *For example, user can create more than one node connection
 *or local planer,
 *
 *@author Jyh-Ming Lien
 *@date 1/08/2001
 */
 
////////////////////////////////////////////////////////////////////////
//Package Name
//package dsmft.OBPRM.iOBPRM;

////////////////////////////////////////////////////////////////////////
//Imports
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;
import java.util.Vector;

abstract public class 
iMultipleOptionPanel extends iOptionPanel implements ItemListener, ListSelectionListener, ActionListener
{
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Class data member: Components
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        protected JComboBox m_ng_Selection;
        protected JPanel m_ngPanel;       //list all possible options @see m_possilbeOptions
        protected JList m_ngList; 	//Store created options. @see m_ngVector
        protected JButton m_CreateButton; //create an option
        protected JButton m_DeleteButton; //delete an option
        protected JButton m_UpButton;     //increase priority of an option
        protected JButton m_DownButton;   //decrease priority of an option
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Class data member: Multiple Options
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        /**
         *This data member is added to CardLayout, if option is cloneable.
         *CardLayout distinguish added components by its "name", if options
         *are cloneable, they will have same names... CardLayout can't not distigush
         *them. So if there is any clonealbe option, it will be added to the end with
         *special name "_@cloneable@_".
         *@see #valueChanged
         */
        private iOptionPanel m_Option_for_Display_Only;
        final private String m_strNamefor_Display_Only_Option="_@cloneable@_";
        
        protected Vector m_ngVector;	    //Contains created options.
        //derived classes have responsibility to fill this vector,m_possilbeOptions.
        protected Vector m_possilbeOptions; //this vetor contains a list of iOptionPanels.
        
        //clear all elements in the list
        private boolean bClearAll;
        protected String m_str_CTag;	//command tag
        protected String m_str_COption; // command option
	
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Constructor
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        public iMultipleOptionPanel()
        {     
                setDefaultValues(); //set to default values
                updateValues(); //update components to current values
                
                add(getComponent(),BorderLayout.CENTER);
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Override iOptionPanel's methods
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
	 *Set Options in this class to default values.
	 *This method deletes all created option in the list.
 	 */
	public void setDefaultValues()
	{
		bClearAll=true;
                m_Option_for_Display_Only=null; //assume no cloneable option.
                if( m_possilbeOptions!=null ) //client should create this
                {
                	for( int iV=0;iV<m_possilbeOptions.size();iV++ )
				((iOptionPanel)m_possilbeOptions.elementAt(iV)).setDefaultValues();
		}
	}
	
	public void updateValues()
	{
		//update template sub options
		if( m_possilbeOptions!=null )
		{
			for( int iV=0;iV<m_possilbeOptions.size();iV++ )
				((iOptionPanel)m_possilbeOptions.elementAt(iV)).updateValues();
		}
		
		//update created sub-options
		if( m_ngVector==null )
                	m_ngVector=new Vector(5);
                	
                if(bClearAll)
                {
	        	m_ngVector.removeAllElements();
	        }
	        else
	        {
	        	for( int iV=0;iV<m_ngVector.size();iV++ )
	        		((iOptionPanel)m_ngVector.elementAt(iV)).updateValues();
	        }
	        	
	        if( m_ngList==null )
	        	m_ngList = new JList();
	        m_ngList.setListData(m_ngVector);
	        
	        //no item in the list is selected..so enable it
	        if( m_CreateButton!=null && bClearAll )
	        	m_CreateButton.setEnabled(true);
	        
	        bClearAll=false;
	}
	
	/**
         *Read Node Generation related data from given file.
         *@note assume file exist
         */
         public boolean parseValues(String [] strValues) throws Exception
         {
        	if( strValues==null )
        		return false;
        		      		
         	String [] values;
		values=iOBPRMUtility.getOptionValue(strValues[0], m_str_CTag);
        	if( values!=null && values[0].compareTo(m_str_CTag)==0 )
        	{
			values[0]=strValues[0]; //copy file name
        	}else return false; //not add any thing
        	
        	if( m_possilbeOptions==null )
        		return true; //there is no option to parse values
        		
        	String exceptionMsg="";
        	for( int iV=0;iV<m_possilbeOptions.size();iV++ )
		{
			iOptionPanel currentOp=
			((iOptionPanel)m_possilbeOptions.elementAt(iV));
			
			boolean bResult=false;
			do{
				//this says something wrong when parsing value
				//but it is sure that this option is in values
				//so use default value and create it.
				try{
					bResult=currentOp.parseValues(values);
				}
				catch( Exception ex )
				{
					bResult=true;
					exceptionMsg+=
					(m_str_CTag+" Warning:\n"+ex+"\n");
				}
				finally{
					//this option read values
					if( bResult==true )
						addOption(currentOp);
				}
				
			}while(bResult==true && currentOp instanceof Cloneable ); //do again if clonealbe
		} 
        	
        	if( exceptionMsg.length()>0 )
        		throw new iOBPRM_ParsingExcpetion(toString(),exceptionMsg,null);
        		
        	return true;
         }
         
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         */
        protected Component getComponent()
        {
        	//derived classes have responsibility to fill this vector.
        	if( m_possilbeOptions==null )
        		return null;
                        
                //init Panel
                JPanel ng_inter_Panel=new JPanel();
                ng_inter_Panel.setLayout(new BorderLayout());

                //add node generator
                m_ngPanel=new JPanel();
                m_ngPanel.setLayout(new CardLayout());
                
                for( int iV=0; iV<m_possilbeOptions.size(); iV++ )
                {
                	iOptionPanel tmpOption=(iOptionPanel)m_possilbeOptions.elementAt(iV);
                	m_ngPanel.add(tmpOption.getComponent(), tmpOption.toString());
                }
                
                //create control panel & add
                ng_inter_Panel.add(getControlComponent(), BorderLayout.WEST);
                ng_inter_Panel.add(m_ngPanel, BorderLayout.CENTER);
                
                //return
                return ng_inter_Panel;
        }
        
        /**
         *This method checks if user input/selected option
         *is "correct".
         *@return true if all created NG instance is validated.
         *        false otherwise.
         */
        public boolean validateOptions()
        {
                if( m_possilbeOptions==null )
                        return false;
                        
                for( int iV=0;iV<m_possilbeOptions.size();iV++ )
                        if( ((iOptionPanel)m_possilbeOptions.
                            elementAt(iV)).validateOptions()==false )
                            return false;
                //if every thing is fine, return true
                return true;
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Event Listener
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
             public void actionPerformed(ActionEvent e) 
             {
                        if( e.getSource()==m_CreateButton) //create
                        {
                                iOptionPanel selectOption=
                                (iOptionPanel)(m_ng_Selection.getItemAt(m_ng_Selection.getSelectedIndex()));
                                
                                if( selectOption!=null ) //add to list
                                {
                                	if( !selectOption.validateOptions() ) //not validated
                                        	return;
                                        if( addOption(selectOption)==false )
                                        {
                                        	//createamessageboxandreturn
						JOptionPane.
						showMessageDialog(this,selectOption+"has been created!");
						return;
                                        }
                                        
					m_ngList.setListData(m_ngVector);
					m_ngList.setSelectedIndex(m_ngVector.size()-1);//selectadded item
                                }
                        }
                        else if( e.getSource()==m_DeleteButton) //delete
                        {
                        	
                        	if( !(m_ngList.getSelectedValue() instanceof iOptionPanel) )
                        		return;
                        		
                        	iOptionPanel selectOption=
                                (iOptionPanel)m_ngList.getSelectedValue();
                                
                                if( selectOption!=null ) //add to list
                                {
                                        m_ngVector.remove(selectOption);
                                        m_ngList.setListData(m_ngVector);
                                        //no item in the list is selected..so enable it
                                        m_CreateButton.setEnabled(true);
                                }
                        }
                        else if( e.getSource()==m_UpButton ) //move up (increase priority)
                        {
                        	if( !(m_ngList.getSelectedValue() instanceof iOptionPanel) )
                        		return;
                                iOptionPanel selectOption=
                                (iOptionPanel)m_ngList.getSelectedValue();
                                
                                if( selectOption!=null ) //add to list
                                {
                                	//find index of this element in vertex
                                	int iV=m_ngVector.indexOf(selectOption);
                                	if( iV==0 ) //it is on the top, I can't move it up
                                		return;
                                	//remove it and insert it again
                                        m_ngVector.remove(selectOption);
                                        m_ngVector.insertElementAt(selectOption,iV-1);
                                        m_ngList.setListData(m_ngVector);
                                        m_ngList.setSelectedIndex(iV-1);
                                }	
                        }
                        else if( e.getSource()==m_DownButton ) //move down (decrease priority)
                        {
                                if( !(m_ngList.getSelectedValue() instanceof iOptionPanel) )
                        		return;
                                iOptionPanel selectOption=
                                (iOptionPanel)m_ngList.getSelectedValue();
                                
                                if( selectOption!=null ) //add to list
                                {
                                	//find index of this element in vertex
                                	int iV=m_ngVector.indexOf(selectOption);
                                	//it is last one, I can't move it down.
                                	if( iV==m_ngVector.size()-1 )
                                		return;
                                	//remove it and insert it again
                                        m_ngVector.remove(selectOption);
                                        m_ngVector.insertElementAt(selectOption,iV+1);
                                        m_ngList.setListData(m_ngVector);
                                        m_ngList.setSelectedIndex(iV+1);
                                }
                        }
        }


        /**
         * Implement ItemListener's method. Update CardLayout item.
         */
        public void itemStateChanged(ItemEvent e)
        {
                CardLayout cl = (CardLayout)(m_ngPanel.getLayout());
                cl.show(m_ngPanel, e.getItem().toString());
                m_CreateButton.setEnabled(true); //can create item from default items
        }
            
        /**
         * Implement ListSelectionListener's method.
         */    
        public void valueChanged(ListSelectionEvent e) 
        {
                if( e.getValueIsAdjusting() )   //one of multiple evnets
                        return;
                        
                iOptionPanel selectOption=null;
                try{
                	selectOption=(iOptionPanel)m_ngList.getSelectedValue();
                }
                catch( Exception ex ){ return; }
                if( selectOption==null ) //not thing selected.
                	return;
                	
                CardLayout cl = (CardLayout)(m_ngPanel.getLayout());
                String strTagName=selectOption.toString();

		//check if 
                if( m_Option_for_Display_Only!=null )
                {
                	cl.removeLayoutComponent(m_Option_for_Display_Only);
                	m_Option_for_Display_Only=null;
                }
                
                //if this selected option is a cloneable option, treat it specially
		if( selectOption instanceof Cloneable )
		{
			//add it CardLayout, and "remember" it...
			m_Option_for_Display_Only=selectOption;
			strTagName=m_strNamefor_Display_Only_Option;
			m_ngPanel.add(selectOption.getComponent(),strTagName);
		}
	
                cl.show(m_ngPanel, strTagName);	//show selected component
                
                m_CreateButton.setEnabled(false); //can not create item from created items
        }
        
        /**
         *Add an option to list.
         */
        protected boolean addOption(iOptionPanel selectOption)
        {
		iOptionPanel clonedOption=null;
		try
		{
                	clonedOption=(iOptionPanel)selectOption.clone();
                }
		//this means this option is not cloneable.
		catch(CloneNotSupportedException ex)
		{
			//check if there is same option in the vector
			for( int iV=0; iV<m_ngVector.size(); iV++ )
			{
				//this option is alreay in the list
				if(m_ngVector.elementAt(iV)==selectOption)
					return false;
			}
	
			clonedOption=selectOption;
		}
		//add to vector
		m_ngVector.add(clonedOption);
		return true;
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Private member methods
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *Set Listeners for components.
         */
        private void setEventHandler()
        {
                if( m_ngList!=null )
                	m_ngList.addListSelectionListener(this);
                if( m_CreateButton!=null )
                        m_CreateButton.addActionListener(this);
                if( m_DeleteButton!=null )
                        m_DeleteButton.addActionListener(this);
                if( m_UpButton!=null )
                        m_UpButton.addActionListener(this);
                if( m_DownButton!=null )
                        m_DownButton.addActionListener(this);
                if( m_ng_Selection!=null )
                        m_ng_Selection.addItemListener(this);
        }
        
        /**
         *Get Control componets.
         *These components are located in the left side and 
         *used to create/delete/modify options.
         */
        private Component getControlComponent()
        {
                //Create control panel
                JPanel controlPanel=new JPanel();
                controlPanel.setLayout(new BoxLayout(controlPanel,BoxLayout.Y_AXIS));
                controlPanel.setBorder(BorderFactory.createTitledBorder("Create/Delete"));
                //create combo
                m_ng_Selection=new JComboBox(m_possilbeOptions);
                m_ng_Selection.setEditable(false);
                    
                //Create list
                m_ngList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
                m_ngList.setSelectedIndex(0);
                JScrollPane listScrollPane = new JScrollPane(m_ngList);
                listScrollPane.setPreferredSize(new Dimension(100,250));
                        
                //Create Buttons
                m_CreateButton=new JButton(new ImageIcon(iOBPRMUtility.getImageDirectory()+"New16.gif"));
                m_DeleteButton=new JButton(new ImageIcon(iOBPRMUtility.getImageDirectory()+"Delete16.gif"));
                m_UpButton=new JButton(new ImageIcon(iOBPRMUtility.getImageDirectory()+"Up16.gif"));
                m_DownButton=new JButton(new ImageIcon(iOBPRMUtility.getImageDirectory()+"Down16.gif"));
                
                JPanel up_down_panel=new JPanel();
                        up_down_panel.setLayout(new GridLayout(1,4));
                        up_down_panel.add(m_UpButton);
                        up_down_panel.add(m_DownButton);
                        up_down_panel.add(m_CreateButton);
                        up_down_panel.add(m_DeleteButton);
                up_down_panel.setPreferredSize(new Dimension(100,24));
                
                setEventHandler(); //set component's listener
                
                //add to comboPanel
                controlPanel.add(m_ng_Selection);
                controlPanel.add(listScrollPane);
                controlPanel.add(up_down_panel);
        
                return controlPanel;
        }
 }









