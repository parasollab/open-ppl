/**
 *This file contains class iNode_Generation which contains
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
iNode_Generation extends iMultipleOptionPanel
{
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Class data member
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        final private String m_str_Iconname="images/ng_icon.gif";
        
        //check if clearance will be calulated for every generated nodes
        private boolean m_bCalculateClearance;
        private JCheckBox m_cbCalculateClearance;

        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Constructor
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        public iNode_Generation()
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
                return "Configure Node Generation related options for OBPRM";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         */
        public void setDefaultValues()
        {
                //init this class
                m_bCalculateClearance=false; //by default defined in Input.cpp.
                
                if( m_possilbeOptions==null )
                {
                        //create Node-generator-option component
                        m_possilbeOptions=new Vector(5);
                        m_possilbeOptions.addElement(new iNG_BasicPRM());
                        m_possilbeOptions.addElement(new iNG_BasicOBPRM());
                        m_possilbeOptions.addElement(new iNG_OBPRM());
                        m_possilbeOptions.addElement(new iNG_GaussPRM());
                        m_possilbeOptions.addElement(new iNG_BasicMAPRM());
                        m_possilbeOptions.addElement(new iNG_CSpaceMAPRM());
                }
                
                m_str_CTag="-gNodes";   //command tag
                super.setDefaultValues();
        }
         
        public void updateValues()
        {
                if( m_cbCalculateClearance==null )
                        m_cbCalculateClearance=new JCheckBox("Calculate Clearance?");
                m_cbCalculateClearance.setSelected(m_bCalculateClearance);
                
                super.updateValues();
         }
        
        /**
         *Get command for OBPRM's Environemtal options
         *accoding to the state of components in this instance.
         *This method call toCommand of its "sub-options"
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
                
                return strCommand+(m_bCalculateClearance?" -calcClear 3":"");
        }
        
        
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         */
        protected Component getComponent()
        {
                //create common component
                JPanel comm_Panel=new JPanel();
                comm_Panel.setBorder(BorderFactory.createTitledBorder("Common Option"));
                                
                m_cbCalculateClearance.addActionListener(new ActionListener() 
                {

                        public void actionPerformed(ActionEvent e) 
                        {
                                m_bCalculateClearance=
                                ((JCheckBox)(e.getSource())).isSelected();
                        }
                });
                m_cbCalculateClearance.setFont(new Font("times", Font.BOLD,10));
                
                comm_Panel.add(m_cbCalculateClearance);
                
                //add common component under control panel
                JPanel ng_Panel=(JPanel)super.getComponent();
                JPanel control_Panel=(JPanel)ng_Panel.getComponent(0);
                ng_Panel.remove(0);
                
                //create common control Panel
                JPanel cc_Panel=new JPanel();
                cc_Panel.setLayout(new BoxLayout(cc_Panel,BoxLayout.Y_AXIS));
                cc_Panel.add(control_Panel); cc_Panel.add(comm_Panel);
                ng_Panel.add(cc_Panel,BorderLayout.WEST);
                
                //return
                return ng_Panel;
        }
 }
 
 /**
  *Interface for BasicPRM options.
  */
 class iNG_BasicPRM extends iSingleFieldOption
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
                return "Basic PRM";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *default is 10 in OBPRM::GNSet::MakeGNSet.
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                
                m_iOptionValue=new Integer(10);
                m_iOptionValue_MAX=new Integer(5000000);
                m_iOptionValue_MIN=new Integer(1);
                
                m_str_OptionName="Number of Nodes";
                m_str_Descrition="Baisc PRM. Use uniform sampling.";
                m_str_CTag="BasicPRM";
                m_str_SingleFieldOption="nodes";
        }
        
        /**
         *Get command for OBPRM's Environemtal options
         *accoding to the state of components in this instance.
         */
        public String toCommand()
        {
        	return m_str_CTag+" "+m_str_SingleFieldOption+" "+m_iOptionValue;
        }
}


 /**
  *Interface for BasicOBPRM options.
  */
 class iNG_BasicOBPRM extends iNG_BasicPRM
 {       
        protected String m_str_3rdOption_CTag; 
        
        //number of shell
        private int m_iNumberOfShells;
        private final int m_iNumberOfShells_MAX=50;
        private final int m_iNumberOfShells_MIN=1;
               
        //components
        private JTextField m_tf_NumberOfShells;
        
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
                return "Basic OBPRM";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *default is 10 in OBPRM::GNSet::MakeGNSet.
         *default is in OBPRM::GNSet::MakeGNSet
         */
        public void setDefaultValues()
        {
                //set parent's default values
                super.setDefaultValues();
                
                //set my default values
                m_iNumberOfShells=3;
                m_str_Descrition=
                "Basic OBPRM. Sampling nodes close to obstacles.";
                m_str_CTag="BasicOBPRM";
                m_str_3rdOption_CTag="shells";
        }
        
        public void updateValues()
        {
                if( m_tf_NumberOfShells==null )
                        m_tf_NumberOfShells=new JTextField(5);
                m_tf_NumberOfShells.setText(""+m_iNumberOfShells);
                
                super.updateValues();
        }
        
        /**
         *Read Node Generation related data from given file.
         *@note assume file exist
         */
         public boolean parseValues(String [] strValues) throws Exception
         {
                if( strValues==null )
                        return false;
                        
                if( super.parseValues(strValues)==false )
                        return false;
                
                Number [] newValue
                =parseNFieldNumbers(1,strValues,m_str_3rdOption_CTag);
                if( newValue!=null && newValue[0]!=null )
                        m_iNumberOfShells=newValue[0].intValue();
                
                //alway return true, please see iSingleFieldOption.parseValues
                return true;
        }
         
        /**
         *Get command for OBPRM's Environemtal options
         *accoding to the state of components in this instance.
         */
        public String toCommand()
        {
            

                return super.toCommand()+" " + m_str_3rdOption_CTag + 
                       " " + m_iNumberOfShells;;
        }
        
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
                try
                {
                        m_iNumberOfShells = Integer.parseInt( m_tf_NumberOfShells.getText() );
                }
                catch( NumberFormatException ex )
                {
                        JOptionPane.
                        showMessageDialog(this,"Number of Shells="+
                                               m_tf_NumberOfShells.getText()+
                                               " is not an int!");      
                        return false;
                }       
                               
                //check number of shell
                if( m_iNumberOfShells>m_iNumberOfShells_MAX && 
                    m_iNumberOfShells<m_iNumberOfShells_MIN )
                {
                        //output error message
                        JOptionPane.
                        showMessageDialog
                        (this,"Number of Shells="
                              +m_iNumberOfShells+" should in ["
                              +m_iNumberOfShells_MIN+","
                              +m_iNumberOfShells_MAX+"]!");
                        return false;
                }
                
                return true;
        }
        
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         *@note assumed m_tf_NumberOfShells is created
         */
        protected Component getComponent()
        {             
              
                //options
                JPanel option_Panel=new JPanel();
                JLabel ShellSizeLabel=new JLabel("Number of Shells");
                ShellSizeLabel.setFont(new Font("times", Font.BOLD,10));
                m_tf_NumberOfShells.setFont(new Font("times", Font.BOLD,10));
                
                option_Panel.add(ShellSizeLabel);
                option_Panel.add(m_tf_NumberOfShells);
                
                ShellSizeLabel.setToolTipText
                  ("This parameter defines how many layer of shells will be generated"+
                  " around a obstacle during node generation pharse" );
                
                //add
                //this must be call after all components in this class are created.
                JPanel basicobprm_Panel=(JPanel)super.getComponent();
                basicobprm_Panel.add(option_Panel);

                return basicobprm_Panel;
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
                if( e.getSource()!=m_tf_NumberOfShells )
                {
                        super.caretUpdate(e);
                        return;
                }
                
                try 
                {
                        m_iNumberOfShells = Integer.parseInt( m_tf_NumberOfShells.getText() );
                }
                catch (NumberFormatException ex) 
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
         *Set Listeners for components in this class.
         */
        protected void setEventHandler()
        {
                super.setEventHandler();
                
                if( m_tf_NumberOfShells!=null )
                        m_tf_NumberOfShells.addCaretListener(this);
        }
        
        /**
         *Set tooltips for components in this class.
         */       
        protected void setToolTips()
        {
                super.setToolTips();
                
                if( m_tf_NumberOfShells!=null )
                {
                  m_tf_NumberOfShells.setToolTipText(
                  "Should be a positive integer ["+m_iNumberOfShells_MIN+","+m_iNumberOfShells_MAX+"]");        
                }
        }

 }
 
 /**
  *Interface for OBPRM options.
  */
 class iNG_OBPRM extends iNG_BasicOBPRM implements ChangeListener, ItemListener
 {
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Class data member : Node Generation
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        private final static String m_str_CenterOfMass="cM";
        private final static String m_str_RandomVertex="rV";
        private final static String m_str_PtOnRandomTriangle="rT";
        private final static String m_str_RandomExtVertex="rE";
        private final static String m_str_PtOnRandomWeightedTriangle="rW";
        private final static String m_str_cM_or_rV="cM_rV";
        private final static String m_str_rV_or_rW="rV_rW";
        private final static String m_str_rV_or_rT="rV_rT";
        private final static String m_str_NormalRandomTriangle="N_rT";
        private final static String m_str_All="all";
        
        //Proportion of surface nodes
        final private String m_str_SurfaceProportion_CTag="pctSurf";    //command tag
        private double m_dSurfaceProportion;
        private final double m_dSurfaceProportion_MAX=1.0;
        private final double m_dSurfaceProportion_MIN=0.0;
  
        //clearance of generated nodes
        final private String m_str_ClearanceFactor_CTag="clearFact";    //command tag
        private double m_dClearanceFactor;
        private final double m_dClearanceFactor_MAX=1.0;
        private final double m_dClearanceFactor_MIN=0.0;
              
        //Collision Pair of generated nodes
        final private String m_strCollPair_CTag="collPair"; //command tag
        private String m_strCollPair[];
        //Free Pair of generated nodes
        final private String m_strFreePair_CTag="freePair"; //command tag
        private String m_strFreePair[];
        
        //components
        private JSlider m_sl_SurfaceProportion;
        private JSlider m_sl_ClearanceFactor;
        private JComboBox m_cb_CollPair_Robot;
        private JComboBox m_cb_FreePair_Robot;
        private JComboBox m_cb_CollPair_Obs;
        private JComboBox m_cb_FreePair_Obs;
        private JLabel m_lb_SurfaceProportion;
        private JLabel m_lb_ClearanceFactorl;  

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
                return "OBPRM";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *default is 10 in OBPRM::GNSet::MakeGNSet.
         *default is in OBPRM::GNSet::MakeGNSet
         */
        public void setDefaultValues()
        {       
                //parent't setDefaultValues
                super.setDefaultValues();
                                
                m_dSurfaceProportion=m_dSurfaceProportion_MAX;
                m_dClearanceFactor=m_dClearanceFactor_MAX;
                 
                //init coll pair and frer pair
                if( m_strCollPair==null )
                        m_strCollPair=new String[2];
                m_strCollPair[0]=m_str_CenterOfMass;
                m_strCollPair[1]=m_str_PtOnRandomTriangle;
                if( m_strFreePair==null )
                        m_strFreePair=new String[2];
                m_strFreePair[0]=m_str_CenterOfMass;
                m_strFreePair[1]=m_str_PtOnRandomTriangle;
                
                m_str_Descrition="Advanced OBPRM. Sampling nodes close to obstacles";
                m_str_CTag="OBPRM";
        }
        
        public void updateValues()
        {
                //init component
                updateValuesForProportion();
                updateValuesForClearanceFactor();
                updateValuesForCollPairs();
                updateValuesForFreePairs();
                
                super.updateValues();
        }
        
        /**
         *Read Node Generation related data from given file.
         *@note assume file exist
         */
         public boolean parseValues(String [] strValues) throws Exception
         {
                if( strValues==null )
                        return false;
        
                if( super.parseValues(strValues)==false )
                        return false; //there is no OBPRM, won't be created
                
                parseValuesForProportion(strValues);
                parseValuesForClearanceFactor(strValues);
                parseValuesForCollPair(strValues);
                parseValuesForFreePair(strValues);
                
                return true;
        }
        
        /**
         *Get command for OBPRM's Environemtal options
         *accoding to the state of components in this instance.
         */
        public String toCommand()
    
        {
                return super.toCommand()+" "+m_strCollPair_CTag+" "+m_strCollPair[0]+
                       " "+m_strCollPair[1]+" "+m_strFreePair_CTag+" "+m_strFreePair[0]+
                       " "+m_strFreePair[1]+" clearFact "+m_dClearanceFactor+
                       " pctSurf "+m_dSurfaceProportion;
        }
        
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         */
        protected Component getComponent()
        {
                //add above together
                JPanel advObprm_Panel=new JPanel();
                advObprm_Panel.setBorder(BorderFactory.createEtchedBorder());
                advObprm_Panel.setLayout(new BoxLayout(advObprm_Panel,BoxLayout.Y_AXIS));
                advObprm_Panel.add(getComponentForProportion());
                advObprm_Panel.add(getComponentForClearanceFactor());
                advObprm_Panel.add(getComponentForCollPairs());
                advObprm_Panel.add(getComponentForFreePairs());
                
                //add
                //this must be call after all components in this class are created.
                JPanel basicprm_Panel=(JPanel)super.getComponent();
                basicprm_Panel.add(advObprm_Panel);
                return basicprm_Panel;
        }
                
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Event Handle
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        public void stateChanged(ChangeEvent e) 
        {
                
                if( e.getSource()==m_sl_SurfaceProportion )
                {
                        if (m_sl_SurfaceProportion.getValueIsAdjusting()) 
                                return;
                        m_dSurfaceProportion = ((double)(m_sl_SurfaceProportion.getValue()))/100;
                        m_lb_SurfaceProportion.setText(
                        "Proprotion of Surface Node("+(int)(m_dSurfaceProportion*100)+"%)");
                }
                else if( e.getSource()==m_sl_ClearanceFactor )
                {
                        if (m_sl_ClearanceFactor.getValueIsAdjusting()) 
                                return;
                        m_dClearanceFactor = ((double)(m_sl_ClearanceFactor.getValue()))/100;
                        m_lb_ClearanceFactorl.setText(
                        "Clearance Factor("+(int)(m_dClearanceFactor*100)+"%)");
                }
        }

        /**
         * Implement ItemListener's method.
         */
        public void itemStateChanged(ItemEvent e)
        {
                if( e.getSource()==m_cb_CollPair_Robot )
                {
                        m_strCollPair[0]=TranslateCodes((String)e.getItem());
                }
                else if( e.getSource()==m_cb_FreePair_Robot )
                {
                        m_strFreePair[0]=TranslateCodes((String)e.getItem());
                }
                else if( e.getSource()==m_cb_CollPair_Obs )
                {
                        m_strCollPair[1]=TranslateCodes((String)e.getItem());
                }
                else if( e.getSource()==m_cb_FreePair_Obs )
                {
                        m_strFreePair[1]=TranslateCodes((String)e.getItem());
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
         *Set Listeners for components in this class.
         */
        protected void setEventHandler()
        {
                super.setEventHandler();
                
                if( m_sl_SurfaceProportion!=null )
                        m_sl_SurfaceProportion.addChangeListener(this);
                        
                if( m_sl_ClearanceFactor!=null )
                        m_sl_ClearanceFactor.addChangeListener(this);
                        
                if( m_cb_CollPair_Robot!=null )
                        m_cb_CollPair_Robot.addItemListener(this);
                        
                if( m_cb_FreePair_Robot!=null )
                        m_cb_FreePair_Robot.addItemListener(this);
                
                if( m_cb_CollPair_Obs!=null )
                        m_cb_CollPair_Obs.addItemListener(this);
               
                if( m_cb_FreePair_Obs!=null )
                        m_cb_FreePair_Obs.addItemListener(this);
        }
        
        /**
         *Set tooltips for components in this class.
         */       
        protected void setToolTips()
        {
                super.setToolTips();
                
                /*if( m_sl_SurfaceProportion!=null )
                {
                  m_tf_NumberOfShells.setToolTipText(
                  "Should be a positive integer ["+m_iNumberOfShells_MIN+","+m_iNumberOfShells_MAX+"]");        
                }*/
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //      Private methods
        //
        //
        /////////////////////////////////////////////////////////////////////////////////////////
        
        /**
         *Set components for surface proportion to default values.
         */     
        private void updateValuesForProportion()
        {
                if( m_lb_SurfaceProportion==null )
                {
                        m_lb_SurfaceProportion=
                        new JLabel("Proprotion of Surface("+m_dSurfaceProportion*100+"%)");
                }
                m_lb_SurfaceProportion.setText("Proprotion of Surface("+m_dSurfaceProportion*100+"%)");
                
                
                if(m_sl_SurfaceProportion==null)
                {
                        m_sl_SurfaceProportion=
                        new JSlider(JSlider.HORIZONTAL,
                                    (int)(m_dSurfaceProportion_MIN*100),
                                    (int)(m_dSurfaceProportion_MAX*100),
                                    (int)(m_dSurfaceProportion*100));
                 }
                 
                 m_sl_SurfaceProportion.setValue((int)(m_dSurfaceProportion*100));
        }
        
        /**
         *parse values for components for surface proportion .
         */  
        public void parseValuesForProportion(String [] strValues) throws Exception
        {
                Number [] newValue
                =parseNFieldNumbers(1,strValues,m_str_SurfaceProportion_CTag);
                if( newValue!=null && newValue[0]!=null )
                        m_dSurfaceProportion=newValue[0].doubleValue();
        }
        
        /**
         *Get components for surface proportion options
         *@note assume m_lb_SurfaceProportion and m_sl_SurfaceProportion is created
         */
        private Component getComponentForProportion()
        {
                //surface proportion options
                JPanel Proportion_Panel=new JPanel();
                Proportion_Panel.setLayout(new FlowLayout(FlowLayout.RIGHT));
                
                //set font
                m_lb_SurfaceProportion.setFont(new Font("times", Font.BOLD,10));
                                         
                Proportion_Panel.add(m_lb_SurfaceProportion);
                Proportion_Panel.add(m_sl_SurfaceProportion);
                
                return Proportion_Panel;
        }
        
        /**
         *Set components for clearance factor to default values.
         */     
        private void updateValuesForClearanceFactor()
        {
                if( m_lb_ClearanceFactorl==null )
                {
                        m_lb_ClearanceFactorl=
                        new JLabel("Clearance Factor("+m_dClearanceFactor*100+"%)");
                }
                m_lb_ClearanceFactorl.setText("Clearance Factor("+m_dClearanceFactor*100+"%)");
                
                
                if(m_sl_ClearanceFactor==null)
                {
                        //create slider
                        m_sl_ClearanceFactor=
                        new JSlider(JSlider.HORIZONTAL,
                                    (int)(m_dClearanceFactor_MIN*100),
                                    (int)(m_dClearanceFactor_MAX*100),
                                    (int)(m_dClearanceFactor*100));     
                 }
                 
                 m_sl_ClearanceFactor.setValue((int)(m_dClearanceFactor*100));
        }
        
        /**
         *parse values for components for clearance factor .
         */  
        public void parseValuesForClearanceFactor(String [] strValues) throws Exception
        {
                Number [] newValue
                =parseNFieldNumbers(1,strValues,m_str_ClearanceFactor_CTag);
                if( newValue!=null && newValue[0]!=null )
                        m_dClearanceFactor=newValue[0].doubleValue();
        }
        
        
        /**
         *Get components for clearance factor
         *@note assume m_lb_ClearanceFactorl and m_sl_ClearanceFactor is created
         */
        private Component getComponentForClearanceFactor()
        {
                ///////////////////////////////////////////////////////////////////////////////////
                //clearance factor
                JPanel CF_Panel=new JPanel();
                CF_Panel.setLayout(new FlowLayout(FlowLayout.RIGHT));
                
                //create label
                m_lb_ClearanceFactorl.setFont(new Font("times", Font.BOLD,10));
                
                CF_Panel.add(m_lb_ClearanceFactorl);
                CF_Panel.add(m_sl_ClearanceFactor);
                
                return CF_Panel;
        }

                
        /**
         *Set components for Collision Pairs to default values.
         */        
        private void updateValuesForCollPairs()
        {
                if( m_cb_CollPair_Robot==null )
                        m_cb_CollPair_Robot=new JComboBox(getComboItemForPairs());
                if( m_cb_CollPair_Obs==null )
                        m_cb_CollPair_Obs=new JComboBox(getComboItemForPairs());
                        
                //set default values to components
                
                //roobt
                int code=TranslateCodesToInt(m_strCollPair[0]);
                if( code<0 )
                {
                        System.err.println("Can't understand "+m_strCollPair[0]+", use default");
                        code=0;
                }
                m_cb_CollPair_Robot.setSelectedIndex(code);
                code=TranslateCodesToInt(m_strCollPair[1]);
                
                //ob
                if( code<0 )
                {
                        System.err.println("Can't understand "+m_strCollPair[1]+", use default");
                        code=2;
                }
                m_cb_CollPair_Obs.setSelectedIndex(code);
        }
       
        /**
         *parse values for components for Coll Pair .
         */  
        public void parseValuesForCollPair(String [] strValues) throws Exception
        {
                int iA=isTagInValues(strValues,m_strCollPair_CTag);
                if( iA>=strValues.length ) return; //no such option
                
                if( iA>=strValues.length-2 )
                        throw new Exception(toString()+" Parse Error:\n"+
                        "No value for option "+m_strCollPair_CTag);
                        
                m_strCollPair[0]=strValues[iA+1];
                m_strCollPair[1]=strValues[iA+2];
        }
        
        /**
         *Get components for Collision Pairs.
         *
         *@note assume m_cb_CollPair_Robot and m_cb_CollPair_Obs is created
         */
        private Component getComponentForCollPairs()
        {
                
                ///////////////////////////////////////////////////////////////////////////////////
                //create Collision Pair combox box
                JLabel CP_Ro_Label=new JLabel("CollPair for Robot:");
                JLabel CP_Ob_Label=new JLabel("CollPair for Obstacle:");
                
                m_cb_CollPair_Robot.setFont(new Font("times", Font.PLAIN,9));
                m_cb_CollPair_Obs.setFont(new Font("times", Font.PLAIN,9));
                CP_Ro_Label.setFont(new Font("times", Font.BOLD,10));
                CP_Ob_Label.setFont(new Font("times", Font.BOLD,10));
                
                ///////////////////////////////////////////////////////////////////////////////////
                //add Collision Pair combox box
                JPanel CP_Panel=new JPanel();
                CP_Panel.setLayout(new GridLayout(2,1));
                
                JPanel c_Ro_Panel=new JPanel(); //put CP_Ro_Label and m_cb_CollPair_Robot together
                c_Ro_Panel.setLayout(new FlowLayout(FlowLayout.RIGHT));
                c_Ro_Panel.add(CP_Ro_Label); c_Ro_Panel.add(m_cb_CollPair_Robot);
                
                JPanel c_Ob_Panel=new JPanel(); //put CP_Ob_Label and m_cb_CollPair_Obs together
                c_Ob_Panel.setLayout(new FlowLayout(FlowLayout.RIGHT));
                c_Ob_Panel.add(CP_Ob_Label); c_Ob_Panel.add(m_cb_CollPair_Obs);
                
                JPanel c_Ro_Ob_Panel=new JPanel(); //put c_Ob_Panel and c_Ro_Panel together
         
                CP_Panel.add(c_Ro_Panel);
                CP_Panel.add(c_Ob_Panel);
                
                return CP_Panel;
        }
        
        /**
         *Set components for Free Pairs to default values.
         */        
        private void updateValuesForFreePairs()
        {
                if( m_cb_FreePair_Robot==null )
                        m_cb_FreePair_Robot=new JComboBox(getComboItemForPairs());
                if( m_cb_FreePair_Obs==null )
                        m_cb_FreePair_Obs=new JComboBox(getComboItemForPairs());
                        
                //set default values to components
                
                //for robot
                int code=TranslateCodesToInt(m_strFreePair[0]);
                if( code<0 )
                {
                        System.err.println("Can't understand "+m_strFreePair[0]+", use default");
                        code=0;
                }
                m_cb_FreePair_Robot.setSelectedIndex(code);
                //for ob
                code=TranslateCodesToInt(m_strFreePair[1]);
                if( code<0 )
                {
                        System.err.println("Can't understand "+m_strFreePair[1]+", use default");
                        code=2;
                }
                m_cb_FreePair_Obs.setSelectedIndex(code);
        }
       
        /**
         *parse values for components for Free Pair .
         */  
        public void parseValuesForFreePair(String [] strValues) throws Exception
        {
                int iA=isTagInValues(strValues,m_strFreePair_CTag);
                if( iA>=strValues.length ) return; //no such option
                
                if( iA>=strValues.length-2 )
                        throw new Exception(toString()+" Parse Error:\n"+
                        "No value for option "+m_strFreePair_CTag);
                        
                m_strFreePair[0]=strValues[iA+1];
                m_strFreePair[1]=strValues[iA+2];
        }
       
        /**
         *Get components for Free Pairs.
         *
         *@note assume m_cb_FreePair_Robot and m_cb_FreePair_Obs is created
         */
        private Component getComponentForFreePairs()
        {
                ///////////////////////////////////////////////////////////////////////////////////
                //create Free Pair combox box
                JLabel FP_Ro_Label=new JLabel("Free for Robot:");
                JLabel FP_Ob_Label=new JLabel("Free for Obstacle:");
                m_cb_FreePair_Robot.setFont(new Font("times", Font.PLAIN,9));
                m_cb_FreePair_Obs.setFont(new Font("times", Font.PLAIN,9));
                FP_Ro_Label.setFont(new Font("times", Font.BOLD,10));
                FP_Ob_Label.setFont(new Font("times", Font.BOLD,10));
                
                ///////////////////////////////////////////////////////////////////////////////////
                //add Free Pair combox box
                JPanel FP_Panel=new JPanel();
                FP_Panel.setLayout(new GridLayout(2,1));
                
                JPanel f_Ro_Panel=new JPanel(); //put CF_Ro_Label and m_cb_CollPair_Robot together
                f_Ro_Panel.setLayout(new FlowLayout(FlowLayout.RIGHT));
                f_Ro_Panel.add(FP_Ro_Label); f_Ro_Panel.add(m_cb_FreePair_Robot);
                
                JPanel f_Ob_Panel=new JPanel(); //put CF_Ob_Label and m_cb_CollPair_Obs together
                f_Ob_Panel.setLayout(new FlowLayout(FlowLayout.RIGHT));
                f_Ob_Panel.add(FP_Ob_Label); f_Ob_Panel.add(m_cb_FreePair_Obs);
                
                JPanel f_Ro_Ob_Panel=new JPanel(); //put c_Ob_Panel and c_Ro_Panel together
                
                FP_Panel.add(f_Ro_Panel);
                FP_Panel.add(f_Ob_Panel);
                
                return FP_Panel;
        }

        /**
         *@return an array of components which will be add to a combo box.
         */
        private Object[] getComboItemForPairs()
        {
                //Pairs, this labels will be items of comboxes for CollPair and FreePair
                String [] comboItems={
                new String("Center Of Mass"), new String("Random Vertex"),
                new String("Point On Random Triangle"), new String("Random Extreme Vertex"),
                new String("Point On Random Weighted Triangle"), 
                new String("Center Of Mass or Random Vertex"),
                new String("Random Vertex or Point On Random Triangle"),
                new String("Random Vertex or Point On Random Weighed Triangle"),
                new String("Normal Vector of Random Triangle"),
                new String("One of above.(Randomly)")};
                return comboItems;
        }
        
        /**
         *@return A translated code. EX: Center Of Mass -> cM
         *@see getComboItemForPairs
         */
        private String TranslateCodes(String desc)
        {       
                if( desc.compareTo("Center Of Mass")==0 )
                        return m_str_CenterOfMass;
                if( desc.compareTo("Random Vertex")==0 )
                        return m_str_RandomVertex;
                if( desc.compareTo("Point On Random Triangle")==0 )
                        return m_str_PtOnRandomTriangle;
                if( desc.compareTo("Random Extreme Vertex")==0 )
                        return m_str_RandomExtVertex;
                if( desc.compareTo("Point On Random Weighted Triangle")==0 )
                        return m_str_PtOnRandomWeightedTriangle;        
                if( desc.compareTo("Center Of Mass or Random Vertex")==0 )
                        return m_str_cM_or_rV;
                if( desc.compareTo("Random Vertex or Point On Random Triangle")==0 )
                        return m_str_rV_or_rT;
                if( desc.compareTo("Random Vertex or Point On Random Weighed Triangle")==0 )
                        return m_str_rV_or_rW;
                if( desc.compareTo("Normal Vector of Random Triangle")==0 )
                        return m_str_NormalRandomTriangle;
                if( desc.compareTo("One of above.(Randomly)")==0 )
                        return m_str_All;
                
                return null;
        }
        
        /**
         *@return A translated code. EX: cM-->0
         */
        private int TranslateCodesToInt(String code)
        {       
                if( code.compareTo(m_str_CenterOfMass)==0 )
                        return 0;
                if( code.compareTo(m_str_RandomVertex)==0 )
                        return 1;
                if( code.compareTo(m_str_PtOnRandomTriangle)==0 )
                        return 2;
                if( code.compareTo(m_str_RandomExtVertex)==0 )
                        return 3;
                if( code.compareTo(m_str_PtOnRandomWeightedTriangle)==0 )
                        return 4;       
                if( code.compareTo(m_str_cM_or_rV)==0 )
                        return 5;
                if( code.compareTo(m_str_rV_or_rT)==0 )
                        return 6;
                if( code.compareTo(m_str_rV_or_rW)==0 )
                        return 7;
                if( code.compareTo(m_str_NormalRandomTriangle)==0 )
                        return 8;
                if( code.compareTo(m_str_All)==0 )
                        return 9;
                
                return -1;
        }       
 }
 
 /**
  *Interface for GaussPRM options.
  */
 class iNG_GaussPRM extends iNG_BasicPRM
 {      
        protected String m_str_3rdOption_CTag;
        
        //number of shell
        private double m_iDistance;
        private final double m_iDistance_MAX=5000000.0;
        private final double m_iDistance_MIN=0.0;
               
        //components
        private JTextField m_tf_Distance;

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
                return "Gaussian PRM";
        }
        
        /**
         *Set Options in this class to default values.
         *This method call setDefaultValues of its "sub-options"
         *default is 10 in OBPRM::GNSet::MakeGNSet.
         *default is in OBPRM::GNSet::MakeGNSet
         */
        public void setDefaultValues()
        {
                super.setDefaultValues();
                
                m_iDistance=0;
                m_str_Descrition=
                "Gaussian PRM. Sampling nodes using Gaussian Function.";
                m_str_3rdOption_CTag="d";
                m_str_CTag="GaussPRM";
        }
        
                
        /**
         *Read Node Generation related data from given file.
         *@note assume file exist
         */
         public boolean parseValues(String [] strValues) throws Exception
         {
                if( strValues==null )
                        return false;
                        
                if( super.parseValues(strValues)==false )
                        return false;
                                
                Number [] newValue
                =parseNFieldNumbers(1,strValues,m_str_3rdOption_CTag);
                if( newValue!=null && newValue[0]!=null )
                        m_iDistance=newValue[0].doubleValue();
                
                //alway return true, please see iSingleFieldOption.parseValues
                return true;
        }
        
        public void updateValues()
        {
                if( m_tf_Distance==null )
                        m_tf_Distance=new JTextField(5);
                m_tf_Distance.setText(""+m_iDistance);
                
                super.updateValues();
        }

        /**
         *Get command for OBPRM's Environemtal options
         *accoding to the state of components in this instance.
         */
        public String toCommand()
        {
            String Command="";
            Command+=m_str_CTag+" "+m_str_SingleFieldOption+" "+m_iOptionValue+" " +m_str_3rdOption_CTag + " " +m_iDistance;
            System.out.println("bbb " +Command);
            return Command;
        }
        
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
                try
                {
                        m_iDistance = Double.parseDouble( m_tf_Distance.getText() );
                }
                catch( NumberFormatException ex )
                {
                        JOptionPane.
                        showMessageDialog(this,"Distance="+
                                               m_tf_Distance.getText()+
                                               " is not an double!");   
                        return false;
                }       
                               
                //check distance
                if( m_iDistance>m_iDistance_MAX && 
                    m_iDistance<m_iDistance_MIN )
                {
                        //output error message
                        JOptionPane.
                        showMessageDialog
                        (this,"Distance="
                              +m_iDistance+" should in ["
                              +m_iDistance_MIN+","
                              +m_iDistance_MAX+"]!");
                        return false;
                }
                
                return true;
        }
        
        /**
         *Get components which will be added to this panel.
         *@return a container which contains all components.
         */
        protected Component getComponent()
        {             
              
                //options
                JPanel option_Panel=new JPanel();
                JLabel DisSizeLabel=new JLabel("Sampling Distance");
                DisSizeLabel.setFont(new Font("times", Font.BOLD,10));
                m_tf_Distance=new JTextField(5);
                m_tf_Distance.setFont(new Font("times", Font.BOLD,10));
                
                option_Panel.add(DisSizeLabel);
                option_Panel.add(m_tf_Distance);
                
                DisSizeLabel.setToolTipText
                  ("This parameter defines how far between two serial sampled"+
                   " node will be.");
                m_tf_Distance.setText( String.valueOf(m_iDistance) );
                
                //add
                //this must be call after all components in this class are created.
                JPanel gaussprm_Panel=(JPanel)super.getComponent();
                gaussprm_Panel.add(option_Panel);

                return gaussprm_Panel;
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
                if( e.getSource()!=m_tf_Distance )
                {
                        super.caretUpdate(e);
                        return;
                }
                
                try 
                {
                        m_iDistance = Integer.parseInt( m_tf_Distance.getText() );
                }
                catch (NumberFormatException ex) 
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
         *Set Listeners for components in this class.
         */
        protected void setEventHandler()
        {
                super.setEventHandler();
                
                if( m_tf_Distance!=null )
                        m_tf_Distance.addCaretListener(this);
        }
        
        /**
         *Set tooltips for components in this class.
         */       
        protected void setToolTips()
        {
                super.setToolTips();
                
                if( m_tf_Distance!=null )
                {
                  m_tf_Distance.setToolTipText(
                  "Should be a positive integer ["+m_iDistance_MIN+","+m_iDistance_MAX+"]");    
                }
        }
 }

 /**
  *Interface for BasicMAPRM options.
  */
 class iNG_BasicMAPRM extends iNG_BasicPRM
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
                return "Basic MAPRM";
        }
        
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_str_Descrition=
                "MAPRM. Generate nodes in Medial Axis.";
                m_str_SingleFieldOption="BasicMAPRM";
        }
 }
 
 /**
  *Interface for CSpaceMAPRM options.
  */
 class iNG_CSpaceMAPRM extends iNG_BasicPRM
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
                return "CSpace MAPRM";
        }
        
        public void setDefaultValues()
        {
                super.setDefaultValues();
                m_str_Descrition=
                "CSpaceMAPRM. Generate nodes in CSpace Medial Axis.";
                m_str_SingleFieldOption="CSpaceMAPRM";
        }
 }
