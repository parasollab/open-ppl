/**
 *This file contains class DataFileFilter which contains
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
import java.io.File;
import javax.swing.*;
import javax.swing.filechooser.*;


public class DataFileFilter extends FileFilter {
	
	/////////////////////////////////////////////////////////////////////////////////////////
 	//
 	//
 	//	Class data member
 	//
 	//
 	/////////////////////////////////////////////////////////////////////////////////////////
	String [] m_strFilters;
	
     	/////////////////////////////////////////////////////////////////////////////////////////
 	//
 	//
 	//	Constructor
 	//
 	//
 	/////////////////////////////////////////////////////////////////////////////////////////
	public DataFileFilter(String [] filters)
	{
		m_strFilters=filters;
	}
    
	/** Accept all directories and specified data files.
	 */
	public boolean accept(File f) 
	{
		if (f.isDirectory()) {
			return true;
		}
		
		for( int iF=0; iF<m_strFilters.length;iF++ )
		{
			if( f.getName().endsWith(m_strFilters[iF]) )
				return true;
		}
				
		return false;
	}
    
	/**
	 * The description of this filter
	 */
	public String getDescription() {
		if( m_strFilters==null || m_strFilters.length==0 )
			return "No File";
			
		String fileExtensions=m_strFilters[0];
		for( int iA=1;iA<m_strFilters.length;iA++)
			fileExtensions+=" & "+m_strFilters[iA];
		
		return "OBPRM "+fileExtensions+ " files";
	}
}