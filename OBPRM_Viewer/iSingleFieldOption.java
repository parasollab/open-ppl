/**
 *This file contains class iSingleFieldOption which contains
 *one numerical field.
 *
 *For example, one could have more than one random connection
 *or k-closest connection
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
  *Interface for single field option.
  */
 public
 class iSingleFieldOption extends iBlankOption implements CaretListener
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
        private JTextField m_tf_OptionValue;
        
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
         *This method checks if user input/selected option
         *is "correct".
         *convert data in textfield and check if value is
         *in predefine range.
         *@return true if all created NG instance is validated.
         *        false otherwise.
         */
        public boolean validateOptions()
        {
        	try
        	{
        		NumberFormat Num_Format=NumberFormat.getNumberInstance();
        		m_iOptionValue = (Number)Num_Format.parse( m_tf_OptionValue.getText() );
        		m_tf_OptionValue.setText(m_iOptionValue.toString());
                }
                catch( ParseException ex )
                {
                	//output error message
                	JOptionPane.
                        showMessageDialog(this,m_str_OptionName+" = "+
                                               m_tf_OptionValue.getText()+
                                               " is not a "+
                                               m_iOptionValue.getClass().getName()+
                                               "!!");
                	return false;
                }	
                        
                if( m_iOptionValue.doubleValue()>m_iOptionValue_MAX.doubleValue() || 
                    m_iOptionValue.doubleValue()<m_iOptionValue_MIN.doubleValue() )
                {
                	//output error message
                	JOptionPane.showMessageDialog
                	(this,m_str_OptionName+" = "
                	      +m_iOptionValue+" should in ["
                	      +m_iOptionValue_MIN+","
                	      +m_iOptionValue_MAX+"]!");	
                	return false;
                }
                
                return true;
        }

        /**
	 * Default option name~ meaningless.... :<
 	 */
	public void setDefaultValues()
	{
		m_str_OptionName="Option Value";
	}

	public void updateValues()
	{	
		if( m_tf_OptionValue==null )
			m_tf_OptionValue=new JTextField(5);
			
                if( m_iOptionValue!=null )
                	m_tf_OptionValue.setText( m_iOptionValue.toString() );
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
         
        /**
         *Get command for OBPRM's Environemtal options
         *accoding to the state of components in this instance.
         */
        public String toCommand()
        {
                return m_str_CTag + " " + m_iOptionValue;
        }
                
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         *@note here assumed sm_tf_OptionValue is created
         */
        protected Component getComponent()
        {                
                //option
                JPanel option_Panel=new JPanel();
                JLabel OptionLabel=new JLabel(m_str_OptionName);
                OptionLabel.setFont(new Font("times", Font.BOLD,10));
                m_tf_OptionValue.setFont(new Font("times", Font.BOLD,10));
                option_Panel.add(OptionLabel);
                option_Panel.add(m_tf_OptionValue);
        	setToolTips();
        	setEventHandler();
        	 
                //add
                JPanel singleF_Panel=(JPanel)super.getComponent();
                singleF_Panel.add(Box.createVerticalGlue());
                singleF_Panel.add(option_Panel);
                
                return singleF_Panel;
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
                try 
                {
                	NumberFormat Num_Format=NumberFormat.getNumberInstance();
        		m_iOptionValue = (Number)Num_Format.parse( m_tf_OptionValue.getText() );
                }
                catch( ParseException ex )
                {
                        // This should never happen because insertString allows
                        // only properly formatted data to get in the field.
                        Toolkit.getDefaultToolkit().beep();
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
                if( m_tf_OptionValue!=null )
                	m_tf_OptionValue.addCaretListener(this);
        }
        
        /**
         *Set tooltips for components in this class.
         */    
        protected void setToolTips()
        {         
                if( m_tf_OptionValue!=null )
                {
                  m_tf_OptionValue.setToolTipText(
                  "Should be an integer ["+m_iOptionValue_MIN+","+m_iOptionValue_MAX+"]");
                }
        }
 }