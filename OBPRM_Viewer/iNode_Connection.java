/**
 *This file contains class iNode_Connection which contains
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
import java.text.*;

final public class 
iNode_Connection extends iMultipleOptionPanel
{
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Class data member
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        final private String m_str_Iconname="images/nc_icon.gif";

        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Constructor
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        public iNode_Connection()
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
                return "Configure Node Connection related options for OBPRM";
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
                        m_possilbeOptions.addElement(new iNC_Random());
                        m_possilbeOptions.addElement(new iNC_KClosest());
                        m_possilbeOptions.addElement(new iNC_KClosestVE());
                        m_possilbeOptions.addElement(new iNC_ObstacleBased());
                        m_possilbeOptions.addElement(new iNC_Components());
                        m_possilbeOptions.addElement(new iNC_ModifiedLM());
                        m_possilbeOptions.addElement(new iNC_RRTExpand());  
                        m_possilbeOptions.addElement(new iNC_RRTComponents());   
                } 
                
                m_str_CTag="-cNodes";   //command tag           
                super.setDefaultValues();
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
 }
 
 /**
  *Interface for Random options.
  */
 class iNC_Random extends iSingleFieldOption implements Cloneable
 {
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Constructor
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        
        public iNC_Random(){}//default
        
        private iNC_Random(int defaultValue)
        {
                setDefaultValues();
                m_iOptionValue=new Integer(defaultValue);
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
                return new iNC_Random(this.m_iOptionValue.intValue());
        }
        
        /**
         *Get description of this class.
         *Override Object#toString.
         */
        public String toString()
        {
                return "Radom Connection";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *default is 5, defined in ConnectNodeMap.h, DEFAULT_numEdges.
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                
                m_iOptionValue=new Integer(5);
                m_iOptionValue_MAX=new Integer(Integer.MAX_VALUE);
                m_iOptionValue_MIN=new Integer(1);
                m_str_OptionName="Number of Edges";
                m_str_Descrition="Randomly connected.";
                m_str_CTag="random";
                m_str_SingleFieldOption="random";
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
        
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         */
        protected Component getComponent()
        {
                //this must be call after all components in this class are created.
                JPanel random_Panel=(JPanel)super.getComponent();
                random_Panel.setBorder(BorderFactory.createTitledBorder(m_str_Descrition));

                return random_Panel;
        }
 }


 /**
  *Interface for K Closest options.
  */
 class iNC_KClosest extends iSingleFieldOption
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
                return "K-Closest";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *default is 5, defined in ConnectNodeMap.h, MakeCNSet.
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_iOptionValue=new Integer(5);
                m_iOptionValue_MAX=new Integer(Integer.MAX_VALUE);
                m_iOptionValue_MIN=new Integer(1);
                m_str_OptionName="Number of Edges";
                m_str_Descrition="Connect K-Closet Neightbors.";
                m_str_CTag="closest";
                m_str_SingleFieldOption="closest";
        }
 }
 
 /**
  *Interface for K Closest V&E options.
  */
 class iNC_KClosestVE extends iSingleFieldOption
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
                return "K-Closest V&E";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *default is 5, defined in ConnectNodeMap.h, MakeCNSet.
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                
                m_iOptionValue=new Integer(5);
                m_iOptionValue_MAX=new Integer(Integer.MAX_VALUE);
                m_iOptionValue_MIN=new Integer(1);
                m_str_Descrition="Connect K-Closet Vertice&Edges.";
                m_str_OptionName="Number of Edges";
                m_str_CTag="closestVE";
                m_str_SingleFieldOption="closestVE";
        }
 }
 
 
 /**
  *Interface for Obstacle Based options.
  */
 class iNC_ObstacleBased extends iDoubleFieldsOption
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
                return "Obstacle Based";
        }
          
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *default K_SELF is 3 and K_OTHER is 10, 
         *defined in ConnectMapNodes.cpp.
         */
        public void setDefaultValues()
        {                
                super.setDefaultValues();
               
                //K_OTHER
                m_iOptionValue=new Integer(10);
                m_iOptionValue_MAX=new Integer(Integer.MAX_VALUE);
                m_iOptionValue_MIN=new Integer(1);
                m_str_OptionName
                ="K Other (connect to K nodes from other obstacle)";
                 
                //K_SELF
                m_SecondOption.m_iOptionValue=new Integer(3);
                m_SecondOption.m_iOptionValue_MAX=new Integer(Integer.MAX_VALUE);
                m_SecondOption.m_iOptionValue_MIN=new Integer(1);
                m_SecondOption.m_str_OptionName="K Self (connect to K nodes from same obstacle)";
                        
                m_str_Descrition="Obstacle Based Node Connection.";
                m_str_CTag="obstBased";
                m_str_SingleFieldOption="obstBased";
        }
 }
 
 
 /**
  *Interface for Component options.
  */
 class iNC_Components extends iDoubleFieldsOption
 {
        //is add faild path?!
        private boolean m_bAddPartialEdge;
        private JCheckBox m_cbAddPartialEdge;
        protected String m_str_PartialEdge_CTag;
     
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
                return "Components";
        }
                        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *default SmallCC is 3 and K Pair is 4, defined in ConnectMapNodes.h.
         *add partial edge default is false. (not add) in Input.cpp's Input constructor.
         */
        public void setDefaultValues()
        {                        
                super.setDefaultValues();
                
                //SmallCC
                m_iOptionValue=new Integer(3);
                m_iOptionValue_MAX=new Integer(Integer.MAX_VALUE);
                m_iOptionValue_MIN=new Integer(1);
                m_str_OptionName="Small Connected Component Size";
                
                //K Pair
                m_SecondOption.m_iOptionValue=new Integer(4);
                m_SecondOption.m_iOptionValue_MAX=new Integer(Integer.MAX_VALUE);
                m_SecondOption.m_iOptionValue_MIN=new Integer(1);
                m_SecondOption.m_str_OptionName=
                "If NOT Small, Connect K Pair"; //use k pair value if cc is not small
                 
                //defaul is false
                m_bAddPartialEdge=false;
                
                m_str_Descrition="Connect Connected Components.";
                m_str_CTag="components";
                m_str_SingleFieldOption="components";
                m_str_PartialEdge_CTag="-addPartialEdge";
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
                if( super.parseValues(strValues)==false )
                        return false;
                        
                String [] values
                =iOBPRMUtility.getOptionValue(strValues[0], m_str_PartialEdge_CTag);
                if( values!=null && values.length==2 && 
                    values[0].compareTo(m_str_PartialEdge_CTag)==0 )
                {
                        m_bAddPartialEdge
                        =(Double.parseDouble(values[1])==0)?false:true;
                }
                
                return true;
        }
        
        public void updateValues()
        {
                if( m_cbAddPartialEdge==null )
                        m_cbAddPartialEdge=new JCheckBox("Add Partial Edge?");
                m_cbAddPartialEdge.setSelected(m_bAddPartialEdge);
                
                super.updateValues();
        }
        
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         *@note assumed m_cbAddPartialEdge is created
         */
        protected Component getComponent()
        {
                
                //create check box for addPartialEdge! 
                m_cbAddPartialEdge.setFont(new Font("times", Font.BOLD,10));
                m_cbAddPartialEdge.addActionListener(new ActionListener() 
                {

                        public void actionPerformed(ActionEvent e) 
                        {
                                m_bAddPartialEdge=
                                ((JCheckBox)(e.getSource())).isSelected();
                        }
                });

                //this must be call after all components in this class are created.
                JPanel CC_Panel=(JPanel)super.getComponent();
                CC_Panel.add(m_cbAddPartialEdge);
                CC_Panel.add(Box.createVerticalGlue());
                
                return CC_Panel;
        }
        
        /**
         *Get command for OBPRM's Environemtal options
         *accoding to the state of components in this instance.
         */
        public String toCommand()
        {
                return super.toCommand()+" -addPartialEdge "+((m_bAddPartialEdge)?1:0);
        }       
 }

 /**
  *Interface for Modified LM options.
  */
 class iNC_ModifiedLM extends iTripleFieldsOption
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
                return "Modified LM";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *K Closest is 5, defined in ConnectNodeMap.h.
         *Maximum new nodes is 20, defined in ConnectNodeMap.cpp.
         *R FACTOR is 2, defined in ConnectNodeMap.cpp.
         */
        public void setDefaultValues()
        {
                                
                super.setDefaultValues();
                
                //Maximum new nodes
                m_iOptionValue=new Integer(20);
                m_iOptionValue_MAX=new Integer(Integer.MAX_VALUE);
                m_iOptionValue_MIN=new Integer(1);
                m_str_OptionName="Maximum new nodes"; //try how many times
                
                //K Closest

                m_SecondOption.m_iOptionValue=new Integer(5);
                m_SecondOption.m_iOptionValue_MAX=new Integer(Integer.MAX_VALUE);
                m_SecondOption.m_iOptionValue_MIN=new Integer(1);
                m_SecondOption.m_str_OptionName
                ="K Closest"; //connect to k closest node in closest cc

                //R FACTOR
                m_ThirdOption.m_iOptionValue=new Double(2.0);
                m_ThirdOption.m_iOptionValue_MAX=new Double(Double.MAX_VALUE);
                m_ThirdOption.m_iOptionValue_MIN=new Double(1.0/Double.MAX_VALUE);
                m_ThirdOption.m_str_OptionName
                ="Radius Factor"; //check if this node is in cc'radius times this value 
                
                m_str_Descrition="Modified Laumond's Method.";
                m_str_CTag="modifiedLM";
                m_str_SingleFieldOption="modifiedLM";
        }
 }
 
 /**
  *Interface for RRT Components  options.
  */
 class iNC_RRTComponents extends iTripleFieldsOption
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
                return "RRT Components";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *K Closest is 5, defined in ConnectNodeMap.h.
         *Maximum new nodes is 20, defined in ConnectNodeMap.cpp.
         *R FACTOR is 2, defined in ConnectNodeMap.cpp.
         */
        public void setDefaultValues()
        {
                                         
                super.setDefaultValues();
                
                //Try RRT N times
                m_iOptionValue=new Integer(10);
                m_iOptionValue_MAX=new Integer(Integer.MAX_VALUE);
                m_iOptionValue_MIN=new Integer(1);
                m_str_OptionName="Try RRT N times, N is"; //expand small cc only
                
                //step size
                m_SecondOption.m_iOptionValue=new Integer(3);
                m_SecondOption.m_iOptionValue_MAX=new Integer(Integer.MAX_VALUE);
                m_SecondOption.m_iOptionValue_MIN=new Integer(1);
                m_SecondOption.m_str_OptionName
                ="Step Size Factor"; //connect to k closest node in closest cc
                
                //small cc
                m_ThirdOption.m_iOptionValue=new Integer(3);
                m_ThirdOption.m_iOptionValue_MAX=new Integer(Integer.MAX_VALUE);
                m_ThirdOption.m_iOptionValue_MIN=new Integer(1);
                m_ThirdOption.m_str_OptionName
                ="Small Connected Component Size"; //expand small cc only
                
                m_str_Descrition="Connect Connected Component using RRT.";
                m_str_CTag="RRTcomponents";
                m_str_SingleFieldOption="RRTcomponents";
        }
 }
 
 /**
  *Interface for RRT Expand options.
  */
 class iNC_RRTExpand extends iNC_RRTComponents
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
                return "RRT Expand";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *K Closest is 5, defined in ConnectNodeMap.h.
         *Maximum new nodes is 20, defined in ConnectNodeMap.cpp.
         *R FACTOR is 2, defined in ConnectNodeMap.cpp.
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_str_Descrition="Expend Connected Componets using RRT.";
                m_str_CTag="RRTexpand";
                m_str_SingleFieldOption="RRTexpand";
        }
 }
