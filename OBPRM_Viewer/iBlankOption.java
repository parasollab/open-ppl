 /**
 *This file contains class iBlankOption which contains
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
  *Interface for options without "sub-option".
  */
 abstract
 class iBlankOption extends iOptionPanel
 {
 	
        protected String m_str_Descrition; //a short description about this option
        protected String m_str_CTag;	   //command tag
        
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
	 *Set Options in this class to default values.
 	 */
	public void setDefaultValues()
	{
		//no options, so I don't need to set default values
	}
	
	/**
	 *Update components' values in this class to default values.
 	 */
	public void updateValues()
	{
		//no options, so I don't need to set update values
	}
        
        /**
         *Read Node Generation related data from given file.
         *@return True if this method found this option is defined in strValues.
         *Otherwise false will be returned.
         */
         public boolean parseValues(String [] strValues) throws Exception
         {
         	if( strValues==null )
        		return false;
        		
        	//index for array
		int iA=isTagInValues(strValues,m_str_CTag);

		//if not found, this option won't be added
		return (iA<strValues.length); //smaller than length..means found!!
        }
        
        /**
	 *@return No Options, always return true.
         */
        public boolean validateOptions()
        {       
                return true;
        }
        
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         */
        protected Component getComponent()
        {
                //this must be call after all components in this class are created.
                JPanel Blank_Panel=new JPanel();
                Blank_Panel.setBorder(BorderFactory.createTitledBorder(m_str_Descrition));
                Blank_Panel.setLayout(new BoxLayout(Blank_Panel,BoxLayout.Y_AXIS));
                
                //des & set Default button
                JPanel Des_Panel=new JPanel();
                JLabel Des_Label=new JLabel(toString());
                Des_Label.setFont(new Font("Serif", Font.ITALIC, 18));
                
                //default button
                JButton default_Button=new JButton(new ImageIcon("images/setdedault.gif"));
                default_Button.setToolTipText("Set to default values");
                default_Button.setPreferredSize(new Dimension(18,18));
                default_Button.addActionListener(new ActionListener() 
        	{	public void actionPerformed(ActionEvent e) 
            		{
            			iBlankOption.this.setDefaultValues();
            			iBlankOption.this.updateValues();
            		}
        	});
        	
        	Des_Panel.add(Des_Label); Des_Panel.add(default_Button);
        	
                //add
                Blank_Panel.add(Box.createVerticalGlue());
                Blank_Panel.add(Des_Panel);
                
                return Blank_Panel;
        }
        
        /**
         *Get command for OBPRM's Environemtal options
         *accoding to the state of components in this instance.
         */
        public String toCommand()
        {
                return m_str_CTag;
        }
        
        ////////////////////////////////////////////////////////////////////////////////////
        //
        //	Protected Method for Parsing
        //
        //		isTagInValues (return index for tag)
        //		parseNFieldNumbers (return n values for tag)
        //
        //
        ////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *@param strValues a list of values which might contains given tag.
         *@param strTag a tag.
         *@return index for the give tag in string array.
         *if there is no this tag in given array, a number larger than
         *given length of given array will be returned.
         */
        protected int isTagInValues(String [] strValues,String strTag)
        {
        	if( strValues==null )
        		return 1;
        		
        	//check if this array contains tag of this instance
        	int iA=0;
		for(;iA<strValues.length;iA++)
			if(strValues[iA].compareTo(strTag)==0)
				return iA;
		return iA;
        }
        
                
        /**
         *This method retrives N numbers from given array accroding to given tag.
         *@param iN how many numbers are following given tag
         *@param strTag tag name
         *@return an array of found values
         */
        protected Number[]
        parseNFieldNumbers
        (int iN,String [] strValues, String strTag) throws iOBPRM_ParsingExcpetion
        {
        	//index for array
		int iA=isTagInValues(strValues,m_str_CTag);
		for(;iA<strValues.length;iA++) //find strTag after m_str_CTag
			if(strValues[iA].compareTo(strTag)==0)
				break;
		//no such tag
		if( iA>=strValues.length ) return null;
		
		if( iN<=0 ) return null; //no number is needed....and tag is found
		
		if( iA>=strValues.length-iN) 
		{
			throw new iOBPRM_ParsingExcpetion(
			toString()+" Parse Warning",
			"Not enough value(s) for option "+strTag, null );
		}
		
		Number [] resultValue=new Number[iN];
		try
		{
			NumberFormat Num_Format=NumberFormat.getNumberInstance();
			for( int iV=iA+1; iV<iA+iN+1; iV++ )
        			resultValue[iV-iA-1]=Num_Format.parse( strValues[iV] );

        		return resultValue;
		}
		catch(ParseException ex)
		{
			throw new iOBPRM_ParsingExcpetion(
			toString()+" Parse Warning",
			"Not enough value(s) for option "+strTag, resultValue );
		}
        }
        
        /**
         *Used by cloneable options.
         */
        protected void cloneableOptionModifier(int N, String [] strValues)
        {
                int iA=isTagInValues(strValues,m_str_CTag);
        	      	
        	//test if next value is also a number
        	try{
        		//if this iA is last option with its value
        		if( iA>=strValues.length-N-1 ) return;
        		
        		for( int i=1;i<N+1;i++ )
        			NumberFormat.getNumberInstance().parse( strValues[iA+N+i] );
        		//sorry, next one is a number....so I need to parse it again
        		//trick: set "command tag" before this number so I can 
        		//recognize it next time
        		strValues[iA+N]=m_str_CTag;
        	}
        	catch(ParseException ex)
		{ /*OK. Next value is not a number, it's great~*/ }
		finally
		{ strValues[iA]="#"; /*cumment it out, so next time i won't think it is a tag*/ }
	}
 } 