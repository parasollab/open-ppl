/**
 *This file contains class iCollision_Detection which contains
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
iCollision_Detection extends iMultipleOptionPanel 
{
        
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Class data member
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        final private String m_str_Iconname="images/cd_icon.gif";

        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Constructor
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        public iCollision_Detection()
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
                return "Configure Collision Detection related options for OBPRM";
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
                        m_possilbeOptions.addElement(new iCD_BoundingSphere());
                        m_possilbeOptions.addElement(new iCD_InsideSphere());
                        m_possilbeOptions.addElement(new iCD_Naive());
                        m_possilbeOptions.addElement(new iCD_Quinlan());
                        m_possilbeOptions.addElement(new iCD_CSTK());
                        m_possilbeOptions.addElement(new iCD_VCLIP());
                        m_possilbeOptions.addElement(new iCD_RAPID());
                        m_possilbeOptions.addElement(new iCD_PQP());
                }
                
                m_str_CTag="-cd";       //command tag
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
  *Interface for Bounding Sphere Collision Detection options.
  */
 class iCD_BoundingSphere extends iBlankOption
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
                return "Boudning Sphere";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_str_Descrition="Bounding Sphere Collision Detection.";
                m_str_CTag="boundingSpheres";
        }
 } 
 
 
 /**
  *Interface for Inside Sphere Collision Detection options.
  */
 class iCD_InsideSphere extends iBlankOption
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
                return "Inside Sphere";
        }
        
       /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_str_Descrition="Inside Sphere Collision Detection.";
                m_str_CTag="insideSpheres";
        }
 } 
 
  
 /**
  *Interface for Naive Collision Detection options.
  */
 class iCD_Naive extends iBlankOption
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
                return "Naive";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_str_Descrition="Naive Collision Detection.";
                m_str_CTag="naive";
        }
 } 
  
 /**
  *Interface for Quinlan Collision Detection options.
  */
 class iCD_Quinlan extends iBlankOption
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
                return "Quinlan";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_str_Descrition="Quinlan Collision Detection.";
                m_str_CTag="quinlan";
        }
 } 
 
 /**
  *Interface for CSTK Collision Detection options.
  */
 class iCD_CSTK extends iBlankOption
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
                return "CSTK";
        }
                
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_str_Descrition="K Collision Detection.";
                m_str_CTag="cstk";
        }
 } 
 
 /**
  *Interface for VCLIP Collision Detection options.
  */
 class iCD_VCLIP extends iBlankOption
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
                return "VCLIP";
        }
                        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_str_Descrition="VCLIP Collision Detection.";
                m_str_CTag="vclip";
        }
 } 
 
 /**
  *Interface for RAPID Collision Detection options.
  */
 class iCD_RAPID extends iBlankOption
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
                return "RAPID";
        }
                        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_str_Descrition="RAPID Collision Detection.";
                m_str_CTag="RAPID";
        }
 } 
 
 /**
  *Interface for PQP Collision Detection options.
  */
 class iCD_PQP extends iBlankOption
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
                return "PQP";
        }
                        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_str_Descrition="PQP Collision Detection.";
                m_str_CTag="PQP";
        }
 } 
