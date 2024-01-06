/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


import java.io.*;
import javax.xml.parsers.*;
import org.w3c.dom.*;
import org.xml.sax.*;

import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerConfigurationException;

import javax.xml.transform.dom.DOMSource;

import javax.xml.transform.stream.StreamResult; 


public class tt{

  
  static Document doc = null;
  
 
    public  tt ( String filename)
    {
	doc = parseXmlFile( filename, false );
    } 
	public tt() {
	  doc = parseXmlFile("input.xml", false);
	}

    public Document getDoc(){ return doc; }

    public static void main(String[] args) {
          tt ch = new tt();		
	  NodeList nodelist =	ch._getNodeList("/prototype/common/ngroups.int");
          Modify(doc, nodelist); 
          //createNewDocument();    

    }


     public static NodeList  _getNodeList (String xpath) {	  
            NodeList nodelist= null;
		try  {		  
              nodelist = org.apache.xpath.XPathAPI.selectNodeList(doc, xpath);
		 		}
		catch (Exception e) {
		  e.printStackTrace();
		}
    		return nodelist;
	}
  

	public static String printNodeValue (NodeList nodelist) {

	  Node node = nodelist.item(0);
	  Node parent = node.getParentNode();
	  System.out.println ("parent nodValue: "+parent.getNodeValue());
	  System.out.println ("parent nodname: "+parent.getNodeName());
	  System.out.println ("parent local name: "+parent.getLocalName());
	  System.out.println ("parent type : "+parent.getNodeType());



	  System.out.println ("nodValue: "+node.getNodeValue());
	  System.out.println ("nodname: "+node.getNodeName());
	  System.out.println ("local name: "+node.getLocalName());
	  System.out.println ("type : "+node.getNodeType());

	  NodeList childNodes = node.getChildNodes();
	  Node childNode = childNodes.item(0);
	  String nValue = childNode.getNodeValue();
	  System.out.println ("first child Value: "+nValue);
	  System.out.println ("child nodname: "+childNode.getNodeName());
	  System.out.println ("child local name: "+childNode.getLocalName());
	  System.out.println ("child local type: "+childNode.getNodeType());
            
          return nValue;

  }

    // there are two way to do modifications:
    // direct on the document or using a nodeList
 
	public static void Modify ( Document document, NodeList nodelist ) {

	  Node node = nodelist.item(0);
	  Node parent = node.getParentNode();
	  System.out.println ("parent nodValue: "+parent.getNodeValue());
	  System.out.println ("parent nodname: "+parent.getNodeName());
	  System.out.println ("parent local name: "+parent.getLocalName());
	  System.out.println ("parent type : "+parent.getNodeType());



	  System.out.println ("nodValue: "+node.getNodeValue());
	  System.out.println ("nodname: "+node.getNodeName());
	  System.out.println ("local name: "+node.getLocalName());
	  System.out.println ("type : "+node.getNodeType());

	  NodeList childNodes = node.getChildNodes();
	  Node childNode = childNodes.item(0);
	  String nValue = childNode.getNodeValue();
	  System.out.println ("first child Value: "+nValue);
	  System.out.println ("child nodname: "+childNode.getNodeName());
	  System.out.println ("child local name: "+childNode.getLocalName());
	  System.out.println ("child local type: "+childNode.getNodeType());




	  try{

      // Get the first <dimension> element in the DOM
      NodeList list = document.getElementsByTagName("dimension");

      node = list.item(0);
	  
	    
	     Node b1 = node.appendChild( document.createElement("BBB"));

	    // Use a Transformer for output
	    TransformerFactory tFactory =           TransformerFactory.newInstance();
	    Transformer transformer = tFactory.newTransformer();
	    
	    //DOMSource source = new DOMSource(doc_new);
            DOMSource source = new DOMSource(node);
      
	    StreamResult result = new StreamResult(new BufferedWriter(new  FileWriter("output2.xml")));
	    
	    transformer.transform(source, result);
	  }
	  catch (Exception e){
	    System.out.println("Exception");
	  }
   }




 public static void  create_newDoc(String filename) {

       Document doc_new =null;

	  try {
	    // Create a builder factory
            DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
            factory.setValidating(false);
            // Create the builder and parse the file
	    doc_new = factory.newDocumentBuilder().newDocument();
	  } 
	  catch (Exception e) {
	  }

	  try{
	  	    
	     Element root =(Element) doc_new.createElement("AAA"); 
	     doc_new.appendChild(root);

	     Node bb = root.appendChild( doc_new.createElement("BBB"));

	     bb.appendChild( doc_new.createTextNode("BBB") );
	     bb.appendChild( doc_new.createTextNode(" ")    );
	     root.appendChild( doc_new.createTextNode("text") );


	    // Use a Transformer for output
	    TransformerFactory tFactory =   TransformerFactory.newInstance();
	    Transformer transformer = tFactory.newTransformer();
	    
	    DOMSource source = new DOMSource(root);
	    StreamResult result = new StreamResult(new BufferedWriter(new FileWriter(filename)));
	    
	    transformer.transform(source, result);
	  }
	  catch (Exception e){
	    System.out.println("Exception");
	  }
	  }


public static void  writeXML(String filename) {

     
	 

	  try{
	 	      // Element root =(Element) doc.createElement("AAA"); 
	   
	   
	    TransformerFactory tFactory =   TransformerFactory.newInstance();
	    Transformer transformer = tFactory.newTransformer();
	    
	    DOMSource source = new DOMSource(doc);
	    StreamResult result = new StreamResult(new BufferedWriter(new FileWriter(filename)));
	    
	    transformer.transform(source, result);
	  }
	  catch (Exception e){
	    System.out.println("Exception");
	  }
	  }





    

 
    // Parses an XML file and System.out.printlns a DOM document.
    // If validating is true, the contents is validated against the DTD
    // specified in the file.
    public static Document parseXmlFile(String filename, boolean validating) {
    	try {
        	// Create a builder factory
            DocumentBuilderFactory factory = DocumentBuilderFactory.
															newInstance();
            factory.setValidating(validating);
            // Create the builder and parse the file
            Document doc = factory.newDocumentBuilder().parse(new File(filename));
            return doc;
        } 
		catch (SAXException e) {
        	// A parsing error occurred; the xml input is not valid
        } 
		catch (ParserConfigurationException e) {
        } 
		catch (IOException e) {
        }
        return null;
	}

}
