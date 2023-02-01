/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


import java.io.*;
import javax.swing.*;
import javax.swing.event.*;
import java.awt.*;
import java.awt.event.*;
import java.util.*;  //needed for vectors
import org.w3c.dom.*;
import org.xml.sax.*;
import javax.xml.parsers.*;

class ErrorDialog extends JDialog {
  public ErrorDialog(JFrame parent, int x) {
    super(parent, "Error", true);
    
    Box b = Box.createVerticalBox();
    b.add(Box.createGlue());
    switch(x) {
      case 1: b.add(new JLabel("The selected Group is already in a Group Set.")); break;
      case 2: b.add(new JLabel("Select a Group Set before choosing Quadrature Type.")); break;
      case 3: b.add(new JLabel("Enter only integers.")); break;
      case 4: b.add(new JLabel("Enter only numbers.")); break;
      case 5: b.add(new JLabel("The selected Angle is already in an Angle Set.")); break;
      case 6: b.add(new JLabel("Number of Angle Sets must be >= Order and <= Order*(Order+2).")); break;
    }
    b.add(Box.createGlue());
    getContentPane().add(b, "Center");
    
    JPanel p2 = new JPanel();
    JButton ok = new JButton("Ok");
    p2.add(ok);
    getContentPane().add(p2, "South");
    
    ok.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent evt) {
        setVisible(false);
      }
    } );
    
    setSize(375,150);
  }
}
