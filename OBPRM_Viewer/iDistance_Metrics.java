/**
 *This file contains class iDistance_Metrics which contains
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
import java.util.Vector;

final public class 
iDistance_Metrics extends iMultipleOptionPanel 
{
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Class data member
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        final private String m_str_Iconname=iOBPRMUtility.getImageDirectory()+"dm_icon.gif";

        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Constructor
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        public iDistance_Metrics()
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
                return "Configure Distance Metric related options for OBPRM";
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
                        m_possilbeOptions.addElement(new iDM_Euclidean());
                        m_possilbeOptions.addElement(new iDM_ScaledEuclidean());
                        m_possilbeOptions.addElement(new iDM_Minkowski());
                        m_possilbeOptions.addElement(new iDM_Manhattan());
                        m_possilbeOptions.addElement(new iDM_CenterOfMass());
                }
                
                m_str_CTag="-dm";       //command tag
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
  *Interface for Euclidean Distance Metric options.
  */
 class iDM_Euclidean extends iBlankOption
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
                return "Euclidean";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_str_Descrition="Euclidean Distance Metric.";
                m_str_CTag="euclidean";
        }        
 } 
 
 /**
  *Interface for Scaled Euclidean Distance Metric options.
  */
 class iDM_ScaledEuclidean extends iSingleScaleOption implements Cloneable
 {
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Constructor
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////       
        public iDM_ScaledEuclidean(){} //default, call parent
        
        private iDM_ScaledEuclidean(double dAtS )
        {
                this();
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
                return new iDM_ScaledEuclidean(m_iOptionValue.doubleValue());
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *default is 0.5, defined in LocalPlanner.h, DMSets::MakeDMSet
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                
                m_iOptionValue=new Double(0.5);
                m_iOptionValue_MAX=new Double(1);
                m_iOptionValue_MIN=new Double(0);
                m_str_OptionName="Scale for Position.";
                m_str_Descrition="Scaled Euclidean Distance Metric.";
                m_str_CTag="scaledEuclidean";
                m_str_SingleFieldOption="scaledEuclidean";
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
         *Get description of this class.
         *Override Object#toString.
         */
        public String toString()
        {
                return "Scaled Euclidean";
        }
 }
 
 /**
  *Interface for Minkowski Distance Metric options.
  */
 class iDM_Minkowski extends iTripleFieldsOption
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
                return "Minkowski";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *K Closest is 5, defined in ConnectNodeMap.h.
         *3,3,1/3 is default. defined in LocalPlanner.h.
         */
        public void setDefaultValues()
        {
                
                super.setDefaultValues();
                
                //Power for Position
                m_iOptionValue=new Double(3);
                m_iOptionValue_MAX=new Double(Double.MAX_VALUE);
                m_iOptionValue_MIN=new Double(Double.MIN_VALUE);
                m_str_OptionName="Power for Position"; // power 2 is square...

                //Power for Orientation
                m_SecondOption.m_iOptionValue=new Double(3);
                m_SecondOption.m_iOptionValue_MAX=new Double(Double.MAX_VALUE);
                m_SecondOption.m_iOptionValue_MIN=new Double(Double.MIN_VALUE);
                m_SecondOption.m_str_OptionName
                ="Power for Orientation"; // power 2 is square...

                 
                //Root for Position+Orientation
                m_ThirdOption.m_iOptionValue=new Double(1.0/3);
                m_ThirdOption.m_iOptionValue_MAX=new Double(Double.MAX_VALUE);
                m_ThirdOption.m_iOptionValue_MIN=new Double(Double.MIN_VALUE);
                m_ThirdOption.m_str_OptionName
                ="Root for Position+Orientation"; //ex : 2 for square root
                
                m_str_Descrition="Minkowski Distance Metric.";   
                m_str_CTag="minkowski";
                m_str_SingleFieldOption="minkowski";
        }
 }
 
 /**
  *Interface for Manhattan Distance Metric options.
  */
 class iDM_Manhattan extends iBlankOption
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
                return "Manhattan";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_str_Descrition="Manhattan Distance Metric.";
                m_str_CTag="manhattan";
        }
 } 
 
  
 /**
  *Interface for Center Of Mass Distance Metric options.
  */
 class iDM_CenterOfMass extends iBlankOption
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
                return "Center Of Mass";
        }
               
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_str_Descrition="Center Of Mass Distance Metric.";
                m_str_CTag="com";
        }
 } 

