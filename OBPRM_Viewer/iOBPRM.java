/**
 *This file is start up of iOBPRM, an interface program for OBPRM.
 *Because obprm program is not so easy to use for newcomers, and
 *it requires user to remember many "options", an user friendly interface
 *might be needed. This interface should be easy to use and should provide
 *self-expanasion options.
 *
 *Following are functions for iOBPRM
 *
 *
 *This interface program is implemented in JFC/Swing with JNI.
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
import java.lang.*;
import java.io.*;


public class iOBPRM extends JPanel
{
        
        private String str_MKMP_FileName;       //currnetly opened mkmp file name
        private String str_QRY_FileName;        //currnetly opened qry file name

        ////////////////////////////////////////////////////////////////
        //Construtor
        public iOBPRM()
        {
                setLayout( new BorderLayout() );
                add( get_iOBPRM_ToolBar(),BorderLayout.WEST );
                add( get_iOBPRM_Component(),BorderLayout.CENTER );
        }
        
        /////////////////////////////////////////////////////////////////////////////
        //
        //
        //
        //      Protected Functions:
        //              get_iOBPRM_ToolBar (Create components for Toolbar)
        //              get_iOBPRM_Component (Create components for options)
        //
        //
        /////////////////////////////////////////////////////////////////////////////
        
        ////////////////////////////////////////////////////////////////
        /**
         *Initailize all component for iOBPRM.
         *@return Return a TabbedPanel which contains components for
         *environment, node generation/connection, local planner, distance 
         *metrics, and collision.
         */
        protected Component
        get_iOBPRM_Component()
        {
                JTabbedPane tabbed_Panel=new JTabbedPane();
                tabbed_Panel.setFont(new Font("times", Font.BOLD,10));
                
                ////////////////////////////////////////////////////////
                iEnvironment iEnv=null;
                iNode_Generation iNode_Gen=new iNode_Generation();
                iNode_Connection iNode_Con=new iNode_Connection();
                iLocal_Planner iLP=new iLocal_Planner();
                iDistance_Metrics iDM=new iDistance_Metrics();
                iCollision_Detection iCD=new iCollision_Detection();
                iEnv=new iEnvironment();
                
                ////////////////////////////////////////////////////////
                //Check error
                if( iEnv==null || iNode_Gen==null ||iNode_Con==null ||
                    iLP==null || iDM==null || iCD==null )
                {
                        System.err.println("iOBPRM error: not enough memory");
                        System.exit(1);
                }
                        
                ////////////////////////////////////////////////////////
                //Add to tabbed panel
                tabbed_Panel.addTab("Environment",iEnv.getIcon(),iEnv,iEnv.toString());
                tabbed_Panel.addTab("Map Node Generation",iNode_Gen.getIcon(),iNode_Gen,iNode_Gen.toString());
                tabbed_Panel.addTab("Map Node Connection",iNode_Con.getIcon(),iNode_Con,iNode_Con.toString());
                tabbed_Panel.addTab("Local Planner",iLP.getIcon(),iLP,iLP.toString());
                tabbed_Panel.addTab("Distance Metrics",iDM.getIcon(),iDM,iDM.toString());
                tabbed_Panel.addTab("Collision Detection",iCD.getIcon(),iCD,iCD.toString());
                
                ////////////////////////////////////////////////////////
                return tabbed_Panel;
        }
        
        /**
         *Create a tool bar which is a tool for creating, modify,
         *and execute iOBPRM and OBPRM.
         *@return a JToolBar
         */
        protected JToolBar
        get_iOBPRM_ToolBar()
        {
                JToolBar toolBar = new JToolBar(SwingConstants.VERTICAL);
                toolBar.setFont(new Font("times", Font.BOLD,10));
                //New
                JButton bt_New=new JButton(new ImageIcon("images/toolbar/new.gif"));
                                        bt_New.addActionListener(new ActionListener()
                {       public void actionPerformed(ActionEvent e) 
                        {iOBPRM.this.doNew();}
                });
                bt_New.setToolTipText("new");
                
                //Open
                JButton bt_Open=new JButton(new ImageIcon("images/toolbar/open.gif"));
                bt_Open.addActionListener(new ActionListener() 
                {       public void actionPerformed(ActionEvent e) 
                        {iOBPRM.this.doOpen();}
                });
                bt_Open.setToolTipText("open");
                
                //Save
                JButton bt_Save=new JButton(new ImageIcon("images/toolbar/save.gif"));
                bt_Save.addActionListener(new ActionListener() 
                {       public void actionPerformed(ActionEvent e) 
                        {iOBPRM.this.doSave();}
                });
                bt_Save.setToolTipText("save");
                
                //Save as
                JButton bt_SaveAs=new JButton(new ImageIcon("images/toolbar/saveas.gif"));
                bt_SaveAs.addActionListener(new ActionListener() 
                {       public void actionPerformed(ActionEvent e) 
                        {iOBPRM.this.doSaveAs();}
                });
                bt_SaveAs.setToolTipText("save as");
                
                //Run OBPRM & QUERY
                JButton bt_Run=new JButton(new ImageIcon("images/toolbar/run.gif"));
                bt_Run.addActionListener(new ActionListener() 
                {       public void actionPerformed(ActionEvent e) 
                        {iOBPRM.this.doRun();}
                });
                bt_Run.setToolTipText("run obprm & query");
                
                //Run OBPRM
                JButton bt_RunOB=new JButton(new ImageIcon("images/toolbar/runob.gif"));
                bt_RunOB.addActionListener(new ActionListener() 
                {       public void actionPerformed(ActionEvent e) 
                        {iOBPRM.this.doRunObprm();}
                });
                bt_RunOB.setToolTipText("run obprm");
                
                //Run QUERY
                JButton bt_RunQu=new JButton(new ImageIcon("images/toolbar/runqu.gif")); 
                bt_RunQu.addActionListener(new ActionListener() 
                {       public void actionPerformed(ActionEvent e) 
                        {iOBPRM.this.doRunQuery();}
                });             
                bt_RunQu.setToolTipText("run query");
                
                //Stop
                JButton bt_Stop=new JButton(new ImageIcon("images/toolbar/stop.gif"));    
                bt_Stop.addActionListener(new ActionListener() 
                {       public void actionPerformed(ActionEvent e) 
                        {iOBPRM.this.doStop();}
                }); 
                bt_Stop.setToolTipText("stop!!");
                
                //add
                toolBar.add(bt_New);    toolBar.add(bt_Open);   toolBar.add(bt_Save);
                toolBar.add(bt_SaveAs); toolBar.add(new JLabel("   "));
                toolBar.add(bt_Run);    toolBar.add(bt_RunOB);  toolBar.add(bt_RunQu);
                toolBar.add(bt_Stop);
                
                return toolBar;
        }
             
        /////////////////////////////////////////////////////////////////////////////
        //
        //
        //
        //      Private Functions
        //
        //
        /////////////////////////////////////////////////////////////////////////////
        private void doNew()
        {
                //get option panel
                JComponent optionPanl=(JComponent)getComponent(1);
                
                //loop until find all obprm options
                for( int iC=0;iC<optionPanl.getComponentCount();iC++ )
                {
                        if( optionPanl.getComponent(iC) instanceof iOptionPanel )
                        {
                                ((iOptionPanel)(optionPanl.getComponent(iC))).setDefaultValues();
                                ((iOptionPanel)(optionPanl.getComponent(iC))).updateValues();
                        }
                }
                str_MKMP_FileName=null;
                str_QRY_FileName=null;
                //set frame title
                setFrameContainerTitle();
        }
        
        /**
         *Open a file ( mkmp or qry ) and read information, and display it.
         */
        private void doOpen()
        {                        
                String [] filter=new String[2];
                filter[0]="mkmp";       filter[1]="qry";
                final String strEnvName=
                iOBPRMUtility.getFilename2Open(this,"Open OBPRM Script FileName",filter);
                if( strEnvName==null )
                        return;
                        
                if( iOBPRMUtility.isFileExist(strEnvName)==false )//no such file
                {
                        JOptionPane.
                        showMessageDialog(this,strEnvName+" not found");
                }
                
                //call another to parse..
                Runnable doParse=new Runnable(){
                        public void run()
                        {
                                if( strEnvName.endsWith("qry") )
                                {
                                        str_QRY_FileName=strEnvName;
                                        doOpenQRY();
                                }
                                else if( strEnvName.endsWith("mkmp") )
                                {
                                        str_MKMP_FileName=strEnvName;
                                        doOpenMKMP();
                                }
                                try{
                                        //call update by event handling thread
                                        Runnable doUpdate=
                                        new Runnable(){public void run(){updateValues();}};
                                        SwingUtilities.invokeAndWait(doUpdate);
                                }catch(Exception e){/*Do nothign*/}
                        }       
                };//end of Runnable doParse
                (new Thread(doParse)).start();
        }
        
        /**
         */     
        private void doSave()
        {
                if(str_MKMP_FileName!=null)
                        doSaveMKMP(str_MKMP_FileName);
                else
                        doSaveAsMKMP();
                if( str_QRY_FileName!=null )
                        doSaveQRY(str_QRY_FileName);
                else
                        doSaveAsQRY();
        }
        
        private void doSaveAs()
        {
                doSaveAsMKMP();
                doSaveAsQRY();
        }
        
        private void doRun()
        {
		doRunObprm();
		doRunQuery();
        }
        
        private void doRunObprm()
        {
                // Adding *****************
                String StrCommand=""; //empty command
                //loop until find iOptionPanel
                JComponent optionPanl=(JComponent)getComponent(1);
                for( int iC=0;iC<optionPanl.getComponentCount();iC++ )
                {
                        iOptionPanel tmpOp=(iOptionPanel)(optionPanl.getComponent(iC));
                        try{
                                if( tmpOp instanceof iOptionPanel )
                                {
                                        if(tmpOp.validateOptions())
                                                StrCommand+=tmpOp.toCommand();
                                        else{
                                                System.err.println(tmpOp.toString()+": Invalid Parameters Found");
                                                return;
                                        }
                                }
                        }
                        catch(Exception e)
                        {
                                System.err.println(e);
                        }
                }

                // finished adding
                System.out.println(StrCommand);

                String s;
                try{
                        //      Process myProcess=Runtime.getRuntime().exec("../obprm -f narrow -cd RAPID -gNodes OBPRM nodes 100 shells 3 collPair cM rT freePair cM rV -cNodes closest components 5 5");
                        Process myProcess=Runtime.getRuntime().exec(StrCommand);
                        DataInputStream execln=
                        new DataInputStream(new BufferedInputStream(myProcess.getInputStream()));

                        while((s=execln.readLine())!=null)
                        {
                                System.out.println(s);
                        }
                }
                catch(IOException e)
                {
                        //System.err.println("IOException starting process!");
                        System.err.println("Command not Found");
                }
        }

        private void doRunQuery()
        {
                String StrCommand=""; //empty command
                //loop until find iOptionPanel
                JComponent optionPanl=(JComponent)getComponent(1);
                for( int iC=0;iC<optionPanl.getComponentCount();iC++ )
                {
                        iOptionPanel tmpOp=(iOptionPanel)(optionPanl.getComponent(iC));
                        try{
                                if( tmpOp instanceof iEnvironment )
                                {
                                        if(((iEnvironment)tmpOp).validateQryOptions())
                                                StrCommand+=((iEnvironment)tmpOp).toQryCommand();
                                        else{
                                                System.err.println("Invalid Parameters Found");
                                                return;
                                        }
                                }
                        }
                        catch(Exception e)
                        {
                                System.err.println(e);
                        }
                }

                // finished adding
                System.out.println(StrCommand);

                String s;
                try{
                        //      Process myProcess=Runtime.getRuntime().exec("../obprm -f narrow -cd RAPID -gNodes OBPRM nodes 100 shells 3 collPair cM rT freePair cM rV -cNodes closest components 5 5");
                        Process myProcess=Runtime.getRuntime().exec(StrCommand);
                        DataInputStream execln=
                        new DataInputStream(new BufferedInputStream(myProcess.getInputStream()));

                        while((s=execln.readLine())!=null)
                        {
                                System.out.println(s);
                        }
                }
                catch(IOException e)
                {
                        //System.err.println("IOException starting process!");
                        System.err.println("Command not Found");
                }
        }
        
        private void doStop()
        {
                System.out.println("Stop");
        }
        
        /////////////////////////////////////////////////////////////////////////////
        //
        //
        //
        //      Private's Private Functions
        //
        //
        /////////////////////////////////////////////////////////////////////////////
        private void doOpenMKMP()
        {
                //output message
                System.out.println
                ("###################### Parse File : "+str_MKMP_FileName+" ######################\n");
                
                //check if this is really a mkmp file
                if( iOBPRMUtility.getOptionValue(str_MKMP_FileName, "obprm")==null )
                {
                        //output message
                        System.err.println
                        ("********************** Error:"+ str_MKMP_FileName+" is not a mkmp file.");
                        return;
                }
                                
                //get option panel
                JComponent optionPanl=(JComponent)getComponent(1);
                String [] strValues=new String[1];
                strValues[0]=str_MKMP_FileName;
                //loop until find all obprm options
                for( int iC=0;iC<optionPanl.getComponentCount();iC++ )
                {
                        iOptionPanel tmpOp=(iOptionPanel)(optionPanl.getComponent(iC));
                        try{
                                if( tmpOp instanceof iOptionPanel )
                                {
                                        if( !(tmpOp instanceof iEnvironment) )
                                                tmpOp.setDefaultValues();
                                        tmpOp.parseValues(strValues);
                                }
                        }
                        catch(Exception e)
                        {
                                System.err.println(e);
                        }
                }
        }

        private void doOpenQRY()
        {
                //output message
                System.out.println
                ("###################### Parse File : "+str_QRY_FileName+" ######################\n");
                
                //check if this is really a qry file
                if( iOBPRMUtility.getOptionValue(str_QRY_FileName, "query")==null )
                {
                        //output message
                        System.err.println
                        ("********************** Error:"+ str_QRY_FileName+" is not a qry file.");
                        return;
                }
                        
                //get option panel
                JComponent optionPanl=(JComponent)getComponent(1);
                String [] strValues=new String[1];
                strValues[0]=str_QRY_FileName;
                //loop until find iEnvironment, which is the only one related to Qry
                for( int iC=0;iC<optionPanl.getComponentCount();iC++ )
                {
                        try{
                                if( optionPanl.getComponent(iC) instanceof iEnvironment )
                                        ((iOptionPanel)(optionPanl.getComponent(iC))).
                                        parseValues(strValues);
                        }
                        catch(Exception e)
                        {
                                System.err.println(e);
                        }
                }
        }
        
        private void doSaveMKMP( String FileName )
        {       
                String StrCommand=""; //empty command
                //loop until find iOptionPanel
                JComponent optionPanl=(JComponent)getComponent(1);
                System.out.println(optionPanl.getComponentCount());
                for( int iC=0;iC<optionPanl.getComponentCount();iC++ )
                {
                        iOptionPanel tmpOp=(iOptionPanel)(optionPanl.getComponent(iC));
                        try{
                                if( tmpOp instanceof iOptionPanel )
                                {
                                        tmpOp.validateOptions();
                                        StrCommand+=tmpOp.toCommand();
                                }
                        }
                        catch(Exception e)
                        {
                                System.err.println(e);
                        }
                }
                
                System.out.println("\ndoSaveMKMP" + StrCommand);

                //save command to file
                iOBPRMUtility.
                SaveStringToFile(StrCommand,FileName);
        }
        
        private void doSaveQRY( String FileName )
        {
                String StrCommand=""; //empty command
                //loop until find iOptionPanel
                JComponent optionPanl=(JComponent)getComponent(1);
                for( int iC=0;iC<optionPanl.getComponentCount();iC++ )
                {
                        iOptionPanel tmpOp=(iOptionPanel)(optionPanl.getComponent(iC));
                        try{
                                if( tmpOp instanceof iEnvironment )
                                {
                                        if(((iEnvironment)tmpOp).validateQryOptions())
                                                StrCommand+=((iEnvironment)tmpOp).toQryCommand();
                                        else
                                                System.err.println("Invalid Parameters Found");
                                }
                        }
                        catch(Exception e)
                        {
                                System.err.println(e);
                        }
                }

                System.out.println("\ndoSaceQRY " + StrCommand);

                //save command to file
                iOBPRMUtility.
                SaveStringToFile(StrCommand,FileName);
        }
        
        private void doSaveAsMKMP()
        {
                String [] filter=new String[1]; filter[0]="mkmp";
                String strMKMPName=
                iOBPRMUtility.getFilename2Save(this,"Save OBPRM MKMP File",filter);
                if( strMKMPName==null )
                        return;
                if( !strMKMPName.endsWith(filter[0]) ) 
                        strMKMPName+=".mkmp";
                doSaveMKMP(strMKMPName);
        }
        
        private void doSaveAsQRY()
        {
                String [] filter=new String[1]; filter[0]="qry";
                String strQRYName=
                iOBPRMUtility.getFilename2Save(this,"Save OBPRM QRY File",filter);
                if( strQRYName==null )
                        return;
                if( !strQRYName.endsWith(filter[0]) ) 
                        strQRYName+=".qry";
                doSaveQRY(strQRYName);          
        }
        
        private void updateValues()
        {
                //get option panel
                JComponent optionPanl=(JComponent)getComponent(1);
                
                //loop until find iEnvironment, which is the only one related to Qry
                for( int iC=0;iC<optionPanl.getComponentCount();iC++ )
                {
                        if( optionPanl.getComponent(iC) instanceof iOptionPanel )
                                ((iOptionPanel)(optionPanl.getComponent(iC))).updateValues();
                }
                
                //set frame title
                setFrameContainerTitle();
        }
        
        private void setFrameContainerTitle()
        {
                JFrame parentFrame=getFrameContainer();
                if( parentFrame!=null )
                {
                        if( str_QRY_FileName!=null && str_MKMP_FileName!=null )
                                parentFrame.
                                setTitle("iOBPRM - "+str_MKMP_FileName+" & "+str_QRY_FileName);
                        else if( str_MKMP_FileName!=null )
                                parentFrame.
                                setTitle("iOBPRM - "+str_MKMP_FileName);
                        else if( str_QRY_FileName!=null )
                                parentFrame.
                                setTitle("iOBPRM - "+str_QRY_FileName);
                        else
                                parentFrame.
                                setTitle("iOBPRM - no file opened");
                }
        }
        
        private JFrame getFrameContainer()
        {
                Component parent=this;
                do{
                        parent=parent.getParent();
                }while( parent!=null && !(parent instanceof JFrame ));
                
                return (JFrame)parent;
        }
        
        /////////////////////////////////////////////////////////////////////////////
        //
        //
        //
        //      Main is Here
        //
        //
        /////////////////////////////////////////////////////////////////////////////
        public static void main(String args[])
        {
                JFrame iOBPRM_Frame=new JFrame("iOBPRM");
                iOBPRM_Frame.addWindowListener(new WindowAdapter() {
                        public void windowClosing(WindowEvent e) {System.exit(0);}
                });
                
                iOBPRM iobprm=new iOBPRM();
                
                //show frame....
                iOBPRM_Frame.setContentPane(iobprm);
                iOBPRM_Frame.pack();
                iOBPRM_Frame.setSize(new Dimension(585,400)); //try and try
                iOBPRM_Frame.setResizable(false);       //not allow resize
                iOBPRM_Frame.show();
        }
 }
