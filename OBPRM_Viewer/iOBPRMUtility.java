/**
 *This file is served as a tool class provide utilities
 *like file/dir open
 *
 *@author Jyh-Ming Lien
 *@date 1/07/2001
 */
 
////////////////////////////////////////////////////////////////////////
//Package Name
//package dsmft.OBPRM.iOBPRM;
import java.io.*;
import java.util.ArrayList;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;
import java.text.*;

class iOBPRMUtility
{
	
	
	/////////////////////////////////////////////////////////////////////////////
	//
	//
	//
	//	File System Infomation Functions
	//
	//		getFilename2Open
	//		getFilename2Save
	//		getDirname
	//		isFileExist
	//		SaveStringToFile
	//
	//
	/////////////////////////////////////////////////////////////////////////////
	
	private static File c_CurrentDir;
	static
	{
		c_CurrentDir=null;
	}
	
	/**
         *Get filename using common file dialog.
         *@param strDec the title of file dialog.
         *@param extension if this is not null, file dialog will filter files accroding 
         *to this extension. User cna see files with this extension only.
         *@return user selected file name.
         */
        static public String 
        getFilename2Open(Component parent, String strDec, String [] extension)
        {
                //createe file chooser
                JFileChooser fc = new JFileChooser(getCurrentDirectory());
                fc.setDialogTitle(strDec);
                if( extension!=null )   //set file filter if extension is not null
                {
                        fc.addChoosableFileFilter(new DataFileFilter(extension));
                }
                int returnVal = fc.showOpenDialog(parent);
                
                //check result
                if( returnVal==JFileChooser.APPROVE_OPTION )
                {
                	c_CurrentDir=fc.getSelectedFile().getParentFile();
                        return fc.getSelectedFile().getPath();
                }
                
                return null;    //nothing selected
        }
        
	/**
         *Get filename using common file dialog.
         *@param strDec the title of file dialog.
         *@param extension if this is not null, file dialog will filter files accroding 
         *to this extension. User cna see files with this extension only.
         *@return user selected file name.
         */
        static public String 
        getFilename2Save(Component parent, String strDec, String [] extension)
        {
                //createe file chooser
                JFileChooser fc = new JFileChooser(getCurrentDirectory());
                fc.setDialogTitle(strDec);
                if( extension!=null )   //set file filter if extension is not null
                {
                        fc.addChoosableFileFilter(new DataFileFilter(extension));
                }
                int returnVal = fc.showSaveDialog(parent);
                
                //check result
                if( returnVal==JFileChooser.APPROVE_OPTION )
                {
                	c_CurrentDir=fc.getSelectedFile().getParentFile();
                        return fc.getSelectedFile().getPath();
                }
                
                return null;    //nothing selected
        }
        
        /**
         *Get directory name using common file dialog.
         *@param strDec the title of file dialog.
         *@return user selected file name.
         */
        static public String 
        getDirname(Component parent,String strDec)
        {       
                //create file chooser
                JFileChooser fc = new JFileChooser(getCurrentDirectory());
                fc.setDialogTitle(strDec);
                fc.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY );
                int returnVal = fc.showOpenDialog(parent);
                
                //check result
                if( returnVal==JFileChooser.APPROVE_OPTION )
                {
                	c_CurrentDir=fc.getSelectedFile().getParentFile();
                        return fc.getSelectedFile().getPath();
                }
                
                return null;    //nothing selected
        }
        
        static public boolean isFileExist(String filename)
        {
        	if( filename==null ) return false;
        	
        	try
        	{
        		File sourceFile=new File(filename);
        		if( sourceFile.exists() && sourceFile.isFile() )
				return true;
        	}
        	catch( SecurityException e )
        	{
        	        System.err.println("iOBPRMUtility:isFileExist Error:"+
        	                           "I don't have permission to check file named "+
        	                           filename);
        		return false;	
        	}
        	
        	return false;
        }
	static public String getImageDirectory()
	{
		return "/share/iOBPRM/images/";		
	}
        
        static public File getCurrentDirectory()
        {
        	if( c_CurrentDir==null )
        	{
        		//get current directory
                	try{
	                        c_CurrentDir=new File(System.getProperty("user.dir","."));
	                        return c_CurrentDir;
                	}
                	catch(Exception e)
                	{/*do nothing*/}
                }
                
		return c_CurrentDir;
	}
	
	static public File getCurrentProgramDirectory()
        {
       		//get current directory
               	try{
                        return new File(System.getProperty("user.dir","."));
               	}
               	catch(Exception e)
               	{/*do nothing*/}
               	return null;
	}
	
	static public void SaveStringToFile(String str, String strFilename)
	{
		try
		{
			PrintStream out=
			new PrintStream(new FileOutputStream(strFilename,false));
			out.println(str);
			out.flush();
		}
		catch(FileNotFoundException e)
		{
			System.err.println
			("********************** IOBPRM Output Error: File not found");
		}
		catch(SecurityException  e)
		{
			System.err.println
			("********************** IOBPRM Output Error: No right to write");
		}
		catch(IOException e)
		{
			System.err.println
			("********************** IOBPRM Output Error: I/O Error");
		}
	}
        	
	/////////////////////////////////////////////////////////////////////////////
	//
	//
	//
	//	Command File Parse Helper Functions
	//
	//		getOptionValue
	//		isATag
	//
	//
	//
	/////////////////////////////////////////////////////////////////////////////
        
        /**
         *@param str_FileName where the value from
         *@param str_OptionName Specified option name.
         *@return option name and its values (if have any ), if there is no 
         *this option in the file, return value will be null.
         *@note option values are found in following rule
         *	-find option name if file
         *	-if there is this option name in file
         *		let strings btween this option name
         *		and the string starting with '-' as
         *		the value(s) of this option.
         *	-else return null
         *@note the first value in return array is always str_FileName if
         *return is not null.
         *@note this method assumed given file is exiting.
         */
        static public String []
        getOptionValue(String str_FileName, String str_OptionName)
        {
        	if( str_FileName==null || str_OptionName==null )
        		return null;
        		
        	try
        	{
        		//init local varialbe
        		ArrayList reslutArray=new ArrayList(5); //reserve 5 elements
        		boolean bFound=false; //not found
        		
        		//create input stream
        		StreamTokenizer sourceToken=
        		new StreamTokenizer(new FileReader(str_FileName));
        		
        		//set # as comment char
        		sourceToken.resetSyntax();
        		sourceToken.wordChars(33,127);
        		sourceToken.whitespaceChars(0,32);
        		sourceToken.quoteChar('\\'); //make \ as a new line
        		sourceToken.ordinaryChar('[');
        		sourceToken.ordinaryChar(']');
        		sourceToken.ordinaryChar(',');
        		sourceToken.commentChar('#'); //comment start
        		
        		//start parseing
        		int iA=0;	//array index
        		while(sourceToken.nextToken()!=StreamTokenizer.TT_EOF) //if end of file
        		{        			
        			//not a string, ? wierd stuiation
        			if( sourceToken.ttype!=sourceToken.TT_WORD ) continue;
        			
        			//check if this is start of another option or a number
        			if( bFound==true && isATag(sourceToken.sval) ) break;
        			
        			//found this option
        			if( sourceToken.sval.endsWith(str_OptionName) )
        				bFound=true;
        			
        			//found value
        			if( bFound==true ) reslutArray.add(iA++, sourceToken.sval);
        		}
        		
        		if( bFound==false ) return null;	//no such option found
        		
        		//copy
        		Object [] tmpObj=reslutArray.toArray();
        		String [] resultString=new String[tmpObj.length];
        		for( iA=0;iA<tmpObj.length;iA++ )
        			resultString[iA]=(String)tmpObj[iA];
        		return resultString;	
        	}
        	catch(FileNotFoundException e)
        	{
        		//I don't care this case..client should make sure this file exists.
        		return null;
        	}
        	catch(IOException e)
        	{
        		System.err.println("iOBPRMUtility:getOptionValue Error:"+e);
        		return null;
        	}
        }
        
        /**
         *@return true if this given string is a tag for obprm or query.
         */
        private static boolean isATag(String strName)
        {
        	if( strName.startsWith("-") )
        	{
        		try{
        			NumberFormat.
        			getNumberInstance().parse(strName);
        			return false; /*This is a number*/
        		}
        		catch(ParseException e){//this is another option
        			return true;
        		}
        	}
        	
        	return false;
       	}
       	
       	/////////////////////////////////////////////////////////////////////////////
	//
	//
	//
	//	Env GUI Helper Functions
	//
	//		packFileComponent
	//
	//
	//
	/////////////////////////////////////////////////////////////////////////////
       	               
        /**
         *Get a container for packing given JComponent (textfield and button).
         *For getComponent_for_obprm, getComponent_for_query, 
         *and getComponent_for_advobprm.
         */
        static public Component packFileComponent
        (boolean bIN,String strLabel, JTextField tfText, AbstractButton btOpen)
        {
        	//create images
        	ImageIcon iconIN_OUT=null;
        	if( bIN ) //if this is a parameter for input (file or dir)
        		iconIN_OUT = new ImageIcon(getImageDirectory()+"Import16.gif");
        	else	//output file/dir
        		iconIN_OUT = new ImageIcon(getImageDirectory()+"Export16.gif");

             	//Description label
                JLabel D_label=new JLabel(strLabel,iconIN_OUT,SwingConstants.LEFT);
                D_label.setFont(new Font("times", Font.BOLD,10));
             	 
        	JPanel return_Panel=new JPanel(); //return this panel.       
                return_Panel.setLayout(new BoxLayout(return_Panel, BoxLayout.Y_AXIS));
                
       		JPanel op_Panel=new JPanel(); //use default layout manager. for filename.
       		op_Panel.setLayout(new FlowLayout(FlowLayout.LEFT));
       		
       		//text field
                tfText.setFont(new Font("times", Font.BOLD,10));
                op_Panel.add(tfText);
                
                //button
                if( btOpen!=null ){
                	btOpen.setPreferredSize(new Dimension(20,20));
                	btOpen.setToolTipText("Browse File System");
                	op_Panel.add(btOpen);
                }

                //put every thing to return panel
                D_label.setAlignmentX(JComponent.LEFT_ALIGNMENT);
                op_Panel.setAlignmentX(JComponent.LEFT_ALIGNMENT);
                
                return_Panel.add(D_label);
                return_Panel.add(op_Panel);
                
                //return
                return return_Panel;
        }
}
