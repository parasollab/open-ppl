/**
 *This file contains class iSingleScaleOption which contains
 *one numerical Scale.
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
import java.text.*;

 /**
  *Interface for single scale option.
  */
 abstract public
 class iSingleScaleOption extends iBlankOption implements ChangeListener
 {
 	//value and its range
        protected Number m_iOptionValue;
        protected Number m_iOptionValue_MAX;
        protected Number m_iOptionValue_MIN;
        
        //the short description for that single field
        protected String m_str_OptionName;
        
        //the command tag for that single field
        protected String m_str_SingleFieldOption;
        
        //components
        private JSlider m_sl_ScaleValue;
        private JLabel m_OptionLabel;
        
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Override iOptionPanel's methods
        //	
        //		setDefaultValues
        //		updateValues
        //		parseValues
        //		validateOptions
        //		getComponent
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
                
        /**
	 * Set Defacult value
 	 */
	public void setDefaultValues(){m_str_OptionName="Option Value";}

	/**
         *Read Node Generation related data from given file.
         *@return True if this method found this option is defined in strValues.
         *@exception if this option is found but no "correct" values for it,
         *Exception will be throwed.
         *Otherwise false will be returned.
         */
         public boolean parseValues(String [] strValues) throws Exception
         {         	
         	if( strValues==null || super.parseValues(strValues)==false)
        		return false;
	
		Number [] newValue=
		parseNFieldNumbers(1,strValues,m_str_SingleFieldOption);
		
		if( newValue!=null && newValue[0]!=null )
			m_iOptionValue=newValue[0];
		
		//alway return true..becasue the tag is found,
		//although option tag is not found (use default)
		return true;
        }
		
	public void updateValues()
	{
		if( m_iOptionValue==null || m_iOptionValue_MIN==null || m_iOptionValue_MIN==null )
			return;
			
		if( m_sl_ScaleValue==null )
		{
			m_sl_ScaleValue=new JSlider(JSlider.HORIZONTAL,
                                        ((int)(m_iOptionValue_MIN.doubleValue()*100)), 
                                        ((int)(m_iOptionValue_MAX.doubleValue()*100)),
                                        ((int)(m_iOptionValue.doubleValue()*100)));
                }
			
                m_sl_ScaleValue.setValue( ((int)(m_iOptionValue.doubleValue()*100)) );
	}
	
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         *@note assume m_sl_ScaleValue is created.
         */
        protected Component getComponent()
        {
                //options
                JPanel option_Panel=new JPanel();
                
                m_OptionLabel=
                new JLabel(m_str_OptionName+" ("+(int)(m_iOptionValue.doubleValue()*100)+"%) ");
                m_OptionLabel.setFont(new Font("times", Font.BOLD,10));

	        m_sl_ScaleValue.setMajorTickSpacing(20);
	        m_sl_ScaleValue.setMinorTickSpacing(5);
	        m_sl_ScaleValue.setPaintTicks(true);
	        m_sl_ScaleValue.setPaintLabels(true);
                m_sl_ScaleValue.setFont(new Font("times", Font.PLAIN,10));
                
                option_Panel.add(m_OptionLabel);
                option_Panel.add(m_sl_ScaleValue);
                
        	setEventHandler();
        	 
                //add
                JPanel singleF_Panel=(JPanel)super.getComponent();
                singleF_Panel.add(Box.createVerticalGlue());
                singleF_Panel.add(option_Panel);
                
                return singleF_Panel;
        }
         
        /**
         *Get command for OBPRM's Environemtal options
         *accoding to the state of components in this instance.
         */
        public String toCommand()
        {
                return m_str_CTag + " " + m_iOptionValue;
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Event Handle
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         * for Change Listener. Update the value displayed in label
         */
        public void stateChanged(ChangeEvent e) 
        {
            JSlider source = (JSlider)e.getSource();
            
            if (!source.getValueIsAdjusting()) 
            {
                double value = ((double)source.getValue())/100;
                m_iOptionValue=new Double(value);
                m_OptionLabel.setText(m_str_OptionName+" ("+((int)(value*100))+"%) ");
            }
        }

        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Protected methods
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *Set Listeners for components.
         */
        protected void setEventHandler()
        {
                if( m_sl_ScaleValue!=null )
                	m_sl_ScaleValue.addChangeListener(this);
        }
 }