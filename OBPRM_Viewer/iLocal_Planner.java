/**
 *This file contains class iLocal_Planner which contains
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
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;
import java.util.Vector;

final public class 
iLocal_Planner extends iMultipleOptionPanel 
{
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Class data member
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        final private String m_str_Iconname="images/lp_icon.gif";

        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Constructor
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        public iLocal_Planner()
        {  
                loadIcon(m_str_Iconname);
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Override iOptionPanel's methods
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *Get description of this class.
         *Override Object#toString.
         */
        public String toString()
        {
                return "Configure Local Planner related options for OBPRM";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         */
        public void setDefaultValues()
        {
                //init sub options
                if( m_possilbeOptions!=null )
                {
                        for( int iV=0;iV<m_possilbeOptions.size();iV++ )
                                ((iOptionPanel)m_possilbeOptions.elementAt(iV)).setDefaultValues();
                }
                
                if( m_possilbeOptions==null )
                {
                        //create Node-generator-option component
                        m_possilbeOptions=new Vector(5);
                        m_possilbeOptions.addElement(new iLP_StraightLine());
                        m_possilbeOptions.addElement(new iLP_RotateAtS());
                        m_possilbeOptions.addElement(new iLP_ASTAR_Distance());
                        m_possilbeOptions.addElement(new iLP_ASTAR_Clearance());
                        m_possilbeOptions.addElement(new iLP_ApproxSpheres());
                }
                
                m_str_CTag="-lp";       //command tag
                super.setDefaultValues();
        }
        
        /**
         *Parse values from file.
         */
        public boolean parseValues(String [] strValues) throws Exception
        {
                if( strValues==null ) return false;
                if( super.parseValues(strValues)==true ) return true;
                
                //in this case there is no -lp defined,but might have 
                //-clearance or -lineSegment defined....
                String exceptionMsg="";                 
                for( int iV=0;iV<m_possilbeOptions.size();iV++ )
                {
                        iOptionPanel currentOp=
                        ((iOptionPanel)m_possilbeOptions.elementAt(iV));
                        if( !(currentOp instanceof iLP_StraightLine) ) continue;
                        boolean bResult=false;
                
                        try{
                                bResult=currentOp.parseValues(strValues);
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
                        
                        break;
                }
                if( exceptionMsg.length()>0 )
                        throw new iOBPRM_ParsingExcpetion(toString(),exceptionMsg,null);
                return true;
        }
        
        /**
         *Get command for OBPRM's Environemtal options
         *accoding to the state of components in this instance.
         */
        public String toCommand()
        {
        	String strCommand="";
                if( m_ngVector==null || m_ngVector.size()==0 )
                        return strCommand;
                for( int iV=0;iV<m_ngVector.size();iV++ ){
                	iOptionPanel iOP=(iOptionPanel)m_ngVector.elementAt(iV);
                	strCommand+=(" "+m_str_CTag+" "+iOP.toCommand());
                }
                
                return strCommand;
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
                        
                        if( selectOption==null ) return;
                        if( selectOption instanceof iLP_RotateAtS )
                        {
                                JOptionPane.
                                showMessageDialog(this,
                                "You could have more option by creating Straight Line Planner");
                        }
                }
                
                //call parent's actionPerformed
                super.actionPerformed(e);
        }
 }
 
 /**
  *Interface for Straight Line options.
  */
 class iLP_StraightLine extends iSingleFieldOption
 {
        
        //clearance required?!
        private boolean m_bClearance;
        private JCheckBox m_cbClearance;
        protected String m_str_Clearance_CTag;

        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Override iOptionPanel's methods
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *Get description of this class.
         *Override Object#toString.
         */
        public String toString()
        {
                return "Straight Line";
        }
        
                
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *default, max, and min are defined in Input.cpp. Input's constructor.
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                
                m_iOptionValue=new Integer(0);
                m_iOptionValue_MAX=new Integer(5000);
                m_iOptionValue_MIN=new Integer(0);
                
                m_bClearance=false;             
                m_str_OptionName="Line Segment Length";
                m_str_Descrition="Straight Line Local Planner.";
                //for parse
                m_str_CTag="straightline";
                m_str_SingleFieldOption="-lineSegment";
                m_str_Clearance_CTag="-clearance";
        }
        
        /**
         *Read Node Connection related data from given file.
         *@return True if this method found this option is defined in strValues.
         *@exception if this option is found but no "correct" values for it,
         *Exception will be throwed.
         *Otherwise false will be returned.
         */
        public boolean parseValues(String [] strValues) throws Exception
        {               
                boolean bResult=super.parseValues(strValues);

                //find line segment
                String [] values
                =iOBPRMUtility.getOptionValue(strValues[0], m_str_SingleFieldOption);
                if( values!=null && values.length==2 && 
                    values[0].compareTo(m_str_SingleFieldOption)==0 )
                {
                        bResult=true;
                        m_iOptionValue=Double.valueOf(values[1]);
                }
                
                //find clearance
                values
                =iOBPRMUtility.getOptionValue(strValues[0], m_str_Clearance_CTag);
                if( values!=null && values.length==2 && 
                    values[0].compareTo(m_str_Clearance_CTag)==0 )
                {
                        m_bClearance
                        =(Double.parseDouble(values[1])==0)?false:true;
                        bResult=true;
                }
                
                return bResult;
        }
        
        public void updateValues()
        {
                if( m_cbClearance==null )
                        m_cbClearance=new JCheckBox("Clearance Required?");
                m_cbClearance.setSelected(m_bClearance);
                
                super.updateValues();
        }
        
        /**
         *Get command for OBPRM's Environemtal options
         *accoding to the state of components in this instance.
         */
        public String toCommand()
        {
            /* Clearance still needs to be added */
            return m_str_CTag+" "+m_str_SingleFieldOption+" "+m_iOptionValue
                   +" "+m_str_Clearance_CTag+" "+((m_bClearance)?1:0);
        }
        
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         *@note assume m_cbClearance is created
         */
        protected Component getComponent()
        {             
                //create check box for addPartialEdge! 
                m_cbClearance.setFont(new Font("times", Font.BOLD,10));
                m_cbClearance.addActionListener(new ActionListener() 
                {

                        public void actionPerformed(ActionEvent e) 
                        {
                                m_bClearance=
                                ((JCheckBox)(e.getSource())).isSelected();
                        }
                });

                //add
                JPanel SL_Panel=(JPanel)super.getComponent();
                SL_Panel.add(m_cbClearance);
                SL_Panel.add(Box.createVerticalGlue());
                
                return SL_Panel;
        }
 } 
 
 /**
  *Interface for Rotate At S Local Planner options.
  */
 class iLP_RotateAtS extends iSingleScaleOption implements Cloneable
 {

        public iLP_RotateAtS(){} //default
        
        private iLP_RotateAtS(double dAtS )
        {
                setDefaultValues();
                m_iOptionValue=new Double(dAtS); //over write default value.
                updateValues();
        }

        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Override iOptionPanel's methods
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *Get a copy of this instance.
         */        
        public Object clone()
        {
                return new iLP_RotateAtS(m_iOptionValue.doubleValue());
        }
                
        /**
         *Get description of this class.
         *Override Object#toString.
         */
        public String toString()
        {
                return "Rotate At S";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *default, max, and min are defined in Input.cpp. Input's constructor.
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                
                m_iOptionValue=new Double(0.5);
                m_iOptionValue_MAX=new Double(1);
                m_iOptionValue_MIN=new Double(0);
                m_str_OptionName="Rotate At S, S is";
                m_str_Descrition="Rotate At S Local Planner.";
                m_str_CTag="rotate_at_s";
                m_str_SingleFieldOption="rotate_at_s";
        }
        
        public boolean parseValues(String [] strValues) throws Exception
        {
                boolean bResult=true;;
                try{
                        bResult=super.parseValues(strValues);
                }
                finally
                {
                        if( bResult==false ) return false;
                        cloneableOptionModifier(1,strValues); //single field
                        return true;
                }
        }
 }
 
 
 /**
  *Interface for A* Clearance Local Planner options.
  */
 class iLP_ASTAR_Clearance extends iDoubleFieldsOption
 {


        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Override iOptionPanel's methods
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *Get description of this class.
         *Override Object#toString.
         */
        public String toString()
        {
                return "A* Clearance";
        }
        
                        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *default Try N times is 6 and Neighbors is 3, 
         *defined in LocalPlanner.h.
         */
        public void setDefaultValues()
        {
                
                super.setDefaultValues();
                
                //Try N times, only 1~20
                m_iOptionValue=new Integer(6);
                m_iOptionValue_MAX=new Integer(20);
                m_iOptionValue_MIN=new Integer(1);
                m_str_OptionName="Try N times, N is"; //try n times

                //Neighbors, allow only 3,9,15
                m_SecondOption.m_iOptionValue=new Integer(3);
                m_SecondOption.m_iOptionValue_MAX=new Integer(15);
                m_SecondOption.m_iOptionValue_MIN=new Integer(3);
                m_SecondOption.m_str_OptionName
                ="Number of Neighbors"; //try n neighbors
                
                m_str_Descrition="A* Clearance Local Planner.";
                m_str_CTag="a_star_clearance";
                m_str_SingleFieldOption="a_star_clearance";
        }
 }

 /**
  *Interface for A* Distance Local Planner options.
  */
 class iLP_ASTAR_Distance extends iLP_ASTAR_Clearance
 {
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Override iOptionPanel's methods
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *Get description of this class.
         *Override Object#toString.
         */
        public String toString()
        {
                return "A* Distance";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *default Try N times is 6 and Neighbors is 3, 
         *defined in LocalPlanner.h.
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_str_Descrition="A* Distance Local Planner.";
                m_str_CTag="a_star_distance";
                m_str_SingleFieldOption="a_star_distance";
        }
 }
 
 /**
  *Interface for Approximate Sphere Local Planner options.
  */
 class iLP_ApproxSpheres extends iSingleFieldOption
 {

        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Override iOptionPanel's methods
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *Get description of this class.
         *Override Object#toString.
         */
        public String toString()
        {
                return "Approx Sphere";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *default is 3, defined in LocalPlanner.h, LPSets::MakeLPSet
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                
                m_iOptionValue=new Integer(3);
                m_iOptionValue_MAX=new Integer(Integer.MAX_VALUE);
                m_iOptionValue_MIN=new Integer(0);
                m_str_OptionName="Try to approximate clearance N times, N is";
                m_str_Descrition="Approximate Sphere Local Planner.";
                m_str_CTag="approx_spheres";
                m_str_SingleFieldOption="approx_spheres";
        }
 }
        
