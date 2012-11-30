

MPComparer::
MPComparer(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
  MPStrategyMethod(in_Node,in_pProblem) {
    ParseXML(in_Node);    
  }

void 
MPComparer::
PrintOptions(ostream& out_os) { 
}

void 
MPComparer::
ParseXML(XMLNodeReader& in_Node) {

  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "input") {
      string strategy = citr->stringXMLParameter("Method",true,"","Method");
      m_input_methods.push_back(strategy);    
    } else if (citr->getName() == "comparer_method") {
      string evaluator_method = citr->stringXMLParameter("Method",true,"","Method");
      m_comparer_methods.push_back(evaluator_method);
    } else {
      citr->warnUnknownNode();
    }
  }
}

void 
MPComparer::
operator()() { 
}
