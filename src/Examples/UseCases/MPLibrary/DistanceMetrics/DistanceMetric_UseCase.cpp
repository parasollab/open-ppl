void DistanceMetricUseCase() {

    //first and second configurations that will be inputted into the distance computation function.
    CfgType _c1;
    CfgType _c2;

    //dist is the computed distance between configurations _c1 and _c2.
    double dist = Distance(_c1, _c2);

    //length and configuration that will be inputted into configuration scaling method.
    double _length = <<Specified Length>>;
    CfgType _c;

    //configuration is scaled with default origin.
    ScaleCfg(_length, _c);

}