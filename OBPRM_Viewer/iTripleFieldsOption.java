/**
 *This file contains class iTripleFieldsOption which contains
 *two numerical fields.
 *
 *
 *@author Jyh-Ming Lien
 *@date 1/09/2001
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
import java.text.*;

 /**
  *Interface for single field option.
  */
 public
 class iTripleFieldsOption extends iDoubleFieldsOption
 {
	protected iSingleFieldOption m_ThirdOption;

        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Override iOptionPanel's methods
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *This method checks if user input/selected option
         *is "correct".
         *convert data in textfield and check if value is
         *in predefine range.
         *@return true if all created NG instance is validated.
         *        false otherwise.
         */
        public boolean validateOptions()
        {
         
         	if( super.validateOptions()==false )      
         		return false;
                return m_ThirdOption.validateOptions();
        }
        
        /**
	 * Set Defacult value
 	 */
	public void setDefaultValues()
	{
		super.setDefaultValues();
		if( m_ThirdOption==null )
			m_ThirdOption=new iSingleFieldOption();
		m_ThirdOption.setDefaultValues();
	}	
		        
        /**
         *Read Node Generation related data from given file.
         *@return True if this method found this option is defined in strValues.
         *@exception if this option is found but no "correct" values for it,
         *Exception will be throwed.
         *Otherwise false will be returned.
         */
         public boolean parseValues(String [] strValues) throws Exception
         {         	
         	if( strValues==null )
        		return false;

		try{
			Number [] newValus=
			parseNFieldNumbers(3,strValues,m_str_SingleFieldOption);
			
			if(newValus==null)
				return false; //not created
		
			m_iOptionValue=newValus[0];
			if( super.m_SecondOption!=null )
				super.m_SecondOption.m_iOptionValue=newValus[1];
			if( m_ThirdOption!=null )
				m_ThirdOption.m_iOptionValue=newValus[2];
		}
		catch( iOBPRM_ParsingExcpetion e )
		{	//assign parsed numbers
			if( e.parsedNumber!=null && e.parsedNumber[0]!=null )
				m_iOptionValue=e.parsedNumber[0];
			if( e.parsedNumber!=null && e.parsedNumber[1]!=null &&
			    super.m_SecondOption!=null )
			{
				super.m_SecondOption.m_iOptionValue=e.parsedNumber[1];
			}
			throw e;
		}
		
		//alway return true..becasue the tag is found,
		//although option tag is not found (use default)
		return true;
        }
        
	public void updateValues()
	{
		if( m_ThirdOption!=null )
			m_ThirdOption.updateValues();
		super.updateValues();
	}       
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         */
        protected Component getComponent()
        {
		JPanel FirstPanel=(JPanel)super.getComponent();
		JPanel SecondPanel=(JPanel)m_ThirdOption.getComponent();
		
                //add
                int SecP_Size=SecondPanel.getComponentCount();
                //add last component to first panel
		FirstPanel.add(SecondPanel.getComponent(SecP_Size-1));
                
                //free
                SecondPanel=null;
                
                return FirstPanel;
        }
        
        /**
         *Get command for OBPRM's Environemtal options
         *accoding to the state of components in this instance.
         */
        public String toCommand()
        {
                return m_str_CTag + " " + m_iOptionValue + " " + m_SecondOption.m_iOptionValue + " " + m_ThirdOption.m_iOptionValue;
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Event Handle
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        public void caretUpdate(CaretEvent e) 
        {
         	super.caretUpdate(e);
                m_ThirdOption.caretUpdate(e);
        }
 }