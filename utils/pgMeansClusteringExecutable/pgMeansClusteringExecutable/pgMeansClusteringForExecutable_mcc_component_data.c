/*
 * MATLAB Compiler: 4.3 (R14SP3)
 * Date: Mon Aug 31 02:29:19 2009
 * Arguments: "-B" "macro_default" "-W" "main" "pgMeansClusteringForExecutable"
 * "-d" "pgMeansClusteringExecutable/" 
 */

#include "mclmcr.h"

#ifdef __cplusplus
extern "C" {
#endif
const unsigned char __MCC_pgMeansClusteringForExecutable_session_key[] = {
        '7', '9', '5', '6', '0', '7', '8', 'D', '4', 'F', '3', '6', 'B', '1',
        '4', '7', 'C', '2', '8', '6', '2', 'E', '9', 'D', '0', 'A', '2', 'C',
        '7', '9', 'F', '6', '2', '2', '6', 'A', 'E', '5', '7', '2', '8', 'B',
        'E', 'C', '6', 'D', 'B', '0', 'F', 'D', '1', 'E', 'F', 'D', '7', '0',
        '8', '6', '3', 'C', 'C', '9', 'E', '4', 'A', '2', 'A', 'E', 'C', 'D',
        '3', 'C', 'B', 'F', '2', '5', 'E', 'A', '1', '0', '5', 'A', '7', '9',
        '8', '8', '7', 'C', '4', 'B', '3', '1', '2', '1', '9', '2', 'A', 'A',
        '5', 'B', '0', '0', 'E', '0', 'E', 'D', '4', '4', '3', '8', '5', 'C',
        'D', 'A', '2', 'E', 'A', '1', '2', '6', '3', '3', '5', 'A', '8', '3',
        '2', '5', 'E', '3', 'D', '8', '3', '7', '5', '9', 'B', '1', '4', '0',
        'E', 'A', '5', '8', 'B', '6', '8', '0', 'B', 'C', 'F', '8', 'E', 'C',
        'A', '7', '7', 'E', '2', '0', 'B', 'B', 'A', '7', 'F', '1', 'E', 'B',
        'F', '6', 'F', 'B', '3', '1', 'E', '3', 'E', '0', '6', 'F', '0', 'E',
        '7', '5', 'B', '3', 'F', 'C', 'A', '8', 'A', 'C', 'C', '2', '2', '5',
        'A', '9', '6', '7', '4', '2', '7', 'D', '4', '0', '6', 'D', '5', '1',
        '5', '0', 'B', '9', 'B', '9', '6', 'E', 'F', '7', 'B', 'E', 'D', '0',
        'B', '7', 'D', '0', 'F', 'C', 'F', '8', '5', '1', '8', 'A', '4', '4',
        '6', '4', '3', '6', 'C', 'C', 'F', '2', '9', 'E', '3', '9', 'B', '3',
        'D', 'E', 'A', 'A', '\0'};

const unsigned char __MCC_pgMeansClusteringForExecutable_public_key[] = {
        '3', '0', '8', '1', '9', 'D', '3', '0', '0', 'D', '0', '6', '0', '9',
        '2', 'A', '8', '6', '4', '8', '8', '6', 'F', '7', '0', 'D', '0', '1',
        '0', '1', '0', '1', '0', '5', '0', '0', '0', '3', '8', '1', '8', 'B',
        '0', '0', '3', '0', '8', '1', '8', '7', '0', '2', '8', '1', '8', '1',
        '0', '0', 'C', '4', '9', 'C', 'A', 'C', '3', '4', 'E', 'D', '1', '3',
        'A', '5', '2', '0', '6', '5', '8', 'F', '6', 'F', '8', 'E', '0', '1',
        '3', '8', 'C', '4', '3', '1', '5', 'B', '4', '3', '1', '5', '2', '7',
        '7', 'E', 'D', '3', 'F', '7', 'D', 'A', 'E', '5', '3', '0', '9', '9',
        'D', 'B', '0', '8', 'E', 'E', '5', '8', '9', 'F', '8', '0', '4', 'D',
        '4', 'B', '9', '8', '1', '3', '2', '6', 'A', '5', '2', 'C', 'C', 'E',
        '4', '3', '8', '2', 'E', '9', 'F', '2', 'B', '4', 'D', '0', '8', '5',
        'E', 'B', '9', '5', '0', 'C', '7', 'A', 'B', '1', '2', 'E', 'D', 'E',
        '2', 'D', '4', '1', '2', '9', '7', '8', '2', '0', 'E', '6', '3', '7',
        '7', 'A', '5', 'F', 'E', 'B', '5', '6', '8', '9', 'D', '4', 'E', '6',
        '0', '3', '2', 'F', '6', '0', 'C', '4', '3', '0', '7', '4', 'A', '0',
        '4', 'C', '2', '6', 'A', 'B', '7', '2', 'F', '5', '4', 'B', '5', '1',
        'B', 'B', '4', '6', '0', '5', '7', '8', '7', '8', '5', 'B', '1', '9',
        '9', '0', '1', '4', '3', '1', '4', 'A', '6', '5', 'F', '0', '9', '0',
        'B', '6', '1', 'F', 'C', '2', '0', '1', '6', '9', '4', '5', '3', 'B',
        '5', '8', 'F', 'C', '8', 'B', 'A', '4', '3', 'E', '6', '7', '7', '6',
        'E', 'B', '7', 'E', 'C', 'D', '3', '1', '7', '8', 'B', '5', '6', 'A',
        'B', '0', 'F', 'A', '0', '6', 'D', 'D', '6', '4', '9', '6', '7', 'C',
        'B', '1', '4', '9', 'E', '5', '0', '2', '0', '1', '1', '1', '\0'};

static const char * MCC_pgMeansClusteringForExecutable_matlabpath_data[] = 
    { "pgMeansClusteringForExecutable/", "toolbox/compiler/deploy/",
      "$TOOLBOXMATLABDIR/general/", "$TOOLBOXMATLABDIR/ops/",
      "$TOOLBOXMATLABDIR/lang/", "$TOOLBOXMATLABDIR/elmat/",
      "$TOOLBOXMATLABDIR/elfun/", "$TOOLBOXMATLABDIR/specfun/",
      "$TOOLBOXMATLABDIR/matfun/", "$TOOLBOXMATLABDIR/datafun/",
      "$TOOLBOXMATLABDIR/polyfun/", "$TOOLBOXMATLABDIR/funfun/",
      "$TOOLBOXMATLABDIR/sparfun/", "$TOOLBOXMATLABDIR/scribe/",
      "$TOOLBOXMATLABDIR/graph2d/", "$TOOLBOXMATLABDIR/graph3d/",
      "$TOOLBOXMATLABDIR/specgraph/", "$TOOLBOXMATLABDIR/graphics/",
      "$TOOLBOXMATLABDIR/uitools/", "$TOOLBOXMATLABDIR/strfun/",
      "$TOOLBOXMATLABDIR/imagesci/", "$TOOLBOXMATLABDIR/iofun/",
      "$TOOLBOXMATLABDIR/audiovideo/", "$TOOLBOXMATLABDIR/timefun/",
      "$TOOLBOXMATLABDIR/datatypes/", "$TOOLBOXMATLABDIR/verctrl/",
      "$TOOLBOXMATLABDIR/codetools/", "$TOOLBOXMATLABDIR/helptools/",
      "$TOOLBOXMATLABDIR/demos/", "$TOOLBOXMATLABDIR/timeseries/",
      "$TOOLBOXMATLABDIR/hds/", "toolbox/local/", "toolbox/compiler/",
      "toolbox/control/control/", "toolbox/database/database/",
      "toolbox/images/images/", "toolbox/images/imuitools/",
      "toolbox/images/iptutils/", "toolbox/shared/imageslib/",
      "toolbox/images/medformats/", "toolbox/optim/", "toolbox/stats/" };

static const char * MCC_pgMeansClusteringForExecutable_classpath_data[] = 
    { "java/jar/toolbox/control.jar", "java/jar/toolbox/database.jar",
      "java/jar/toolbox/images.jar" };

static const char * MCC_pgMeansClusteringForExecutable_libpath_data[] = 
    { "bin/glnx86/" };

static const char * MCC_pgMeansClusteringForExecutable_app_opts_data[] = 
    { "" };

static const char * MCC_pgMeansClusteringForExecutable_run_opts_data[] = 
    { "" };

static const char * MCC_pgMeansClusteringForExecutable_warning_state_data[] = 
    { "" };


mclComponentData __MCC_pgMeansClusteringForExecutable_component_data = { 

    /* Public key data */
    __MCC_pgMeansClusteringForExecutable_public_key,

    /* Component name */
    "pgMeansClusteringForExecutable",

    /* Component Root */
    "",

    /* Application key data */
    __MCC_pgMeansClusteringForExecutable_session_key,

    /* Component's MATLAB Path */
    MCC_pgMeansClusteringForExecutable_matlabpath_data,

    /* Number of directories in the MATLAB Path */
    42,

    /* Component's Java class path */
    MCC_pgMeansClusteringForExecutable_classpath_data,
    /* Number of directories in the Java class path */
    3,

    /* Component's load library path (for extra shared libraries) */
    MCC_pgMeansClusteringForExecutable_libpath_data,
    /* Number of directories in the load library path */
    1,

    /* MCR instance-specific runtime options */
    MCC_pgMeansClusteringForExecutable_app_opts_data,
    /* Number of MCR instance-specific runtime options */
    0,

    /* MCR global runtime options */
    MCC_pgMeansClusteringForExecutable_run_opts_data,
    /* Number of MCR global runtime options */
    0,
    
    /* Component preferences directory */
    "pgMeansClusteringForExecutable_3F6F9F5B3DEE5EDAC2537E624A4A0D9B",

    /* MCR warning status data */
    MCC_pgMeansClusteringForExecutable_warning_state_data,
    /* Number of MCR warning status modifiers */
    0,

    /* Path to component - evaluated at runtime */
    NULL

};

#ifdef __cplusplus
}
#endif


