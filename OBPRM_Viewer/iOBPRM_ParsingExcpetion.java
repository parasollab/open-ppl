/**
 *An exception for parsing input command for obprm and query
 *inherient from Exception
 *
 *@author Jyh-Ming Lien
 *@date 1/13/2001
 */
 
////////////////////////////////////////////////////////////////////////
//Package Name
//package dsmft.OBPRM.iOBPRM;
 
/**
 *This class is throwed when pasring methof meets errors, such as
 *not enough values are provided for options.
 *If one option requires 10 values, but only 5 values are found
 *this exception will be thrown. the parsedNumber in this class
 *will contains all values found, such as 5 values in prevous
 *example.
 */
public class iOBPRM_ParsingExcpetion extends Exception
{
	public Number [] parsedNumber;
	private String strTitle;
	
	//Constructor
	public iOBPRM_ParsingExcpetion(String strTitle, String strMsg, Number [] Values)	
	{
		super(strMsg);
		this.strTitle=strTitle;
		parsedNumber=Values;
	}
	
	//override Exception.toString
	public String toString()
	{
		return "********************** "+strTitle+"\n"+super.toString();
	}
}