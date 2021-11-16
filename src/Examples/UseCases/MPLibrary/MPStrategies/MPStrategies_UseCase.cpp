void MPStrategiesUseCase() {

    // The MP strategy (MPS) label is a string that corresponds to the XML node
    // label for the mp strategy that the user would like to use.
    std::string m_mpsLabel = "BasicPRM";

    // Given a MP strategy label, the function GetMPStrategy returns a MPStrategyPointer 
    // object that can be used to run the MP strategy.
    MPStrategyPointer mps = this->GetMPStrategy(m_mpsLabel);

    // Calling operator on the MPStrategy object with pointer notation. This will 
    // execute all the associated functionalities of the MP strategy algorithm including
    // preprocessing, processing, and postprocessing.
    mps -> operator()(); 

    // Calling the MP strategy as a funciton object. A different way to call but
    // performs the same task as above. 
    (*mps)();

}


