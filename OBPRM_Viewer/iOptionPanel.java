/**
 *This file contains class iOptionPanel which is
 *a abstract base class for other OBPRM option
 *class, such as iEnvironment, iLocalPlanner....
 *This class defines following methods: 
 *
 *@author Jyh-Ming Lien
 *@date 1/07/2001
 */
 
////////////////////////////////////////////////////////////////////////
//Package Name
//package dsmft.OBPRM.iOBPRM;

////////////////////////////////////////////////////////////////////////
//Imports
import java.awt.*;
import javax.swing.*;

abstract class
iOptionPanel extends JPanel 
{
 	
 	/////////////////////////////////////////////////////
 	// Class Members
 	private String m_str_Icon_name;  //this is the image icon name for this pannel
 	private ImageIcon m_image_Icon;
 	
 	////////////////////////////////////////////////////////////////
 	//Construtor
 	protected iOptionPanel()
 	{
 		setLayout(new BorderLayout());
 	}
 	
 	/**
 	 *Get a copy of this option. (not implemented)
 	 */
 	public Object clone() throws CloneNotSupportedException
 	{
 		return super.clone();
 	}
 	
 	/**
 	 *Get description of this class.
 	 *Override Object#toString.
 	 */
 	public String toString()
 	{
 		return "iOBPRM Option Panel";
	}
	
	/**
 	 *Get Image Icon representing this class.
 	 *The incon will be put to any place to represent this class
 	 */
	public ImageIcon getIcon()
	{
		return m_image_Icon;
	}
	
	/**
	 *Set Options' variables in this class to default values.
	 *@note these variables should not be components, so 
	 *this method setDefaultValues could be called by any
	 *thread.
	 *@see updateValues to update component's values.
 	 */
	abstract public void setDefaultValues();

	/**
	 *Parse values from file.
	 *this method does not change any values of component so
	 *it could be called by any thread. Clients might need to call
	 *updateValues to update components values using event thread.
	 *@return true if this class find it is defined in given array.
	 *@see updateValues
 	 */
	abstract public boolean parseValues(String [] strValues) throws Exception;
	
	/**
	 *Update the components' values accroding to varibles' values
	 *This method might create components if components are not created.
	 *This method should be called by event handling thread.
	 */
	abstract public void updateValues();

        /**
 	 *Get command for OBPRM accoding to the state of options.
 	 *This is an abstract method which returns a string of OBPRM 
 	 *command options.
 	 */
 	abstract public String toCommand();
 	
 	/**
 	 *This method checks if user input/selected option
 	 *is "correct". Correctness is define by derived
 	 *class of this class.
 	 */
 	abstract public boolean validateOptions();
 	
     	/////////////////////////////////////////////////////////////////////////////////////////
 	//
 	//
 	//	Protected Methods
 	//
 	//
 	/////////////////////////////////////////////////////////////////////////////////////////
 	
 	/**
 	 *Load Icon image from file wth given name.
 	 *@return an ImageIcon instance. If given filename 
 	 *is null, null will be returned.
 	 */
 	protected ImageIcon
 	loadIcon( String name ) 
 	{
 		m_str_Icon_name=name;
 		if( m_str_Icon_name==null)
 			return null;
 			
 		m_image_Icon=new ImageIcon(m_str_Icon_name,this.getClass().getName());
 		return m_image_Icon;
 	}
 	
 	/**
 	 *Load Icon image from file wth #m_str_Icon_name.
 	 *@return an ImageIcon instance. If given #m_str_Icon_name 
 	 *is null, null will be returned.
 	 */
 	protected ImageIcon
 	loadIcon() 
 	{
		return loadIcon(m_str_Icon_name);
 	}
 	
 	 /**
 	  *Get components which will be added to this panel.
 	  *@return a container which contains all components.
 	  */
 	abstract protected Component getComponent();
 }