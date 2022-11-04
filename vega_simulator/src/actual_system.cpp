#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>
#include <cstdio>
#include <cassert>
#include <float.h>

#include <cmath>
// #include <filesystem>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
using namespace std;


#if defined(WIN32) || defined(_WIN32)
  #include <windows.h>
#endif

#ifdef __APPLE__
  #include "TargetConditionals.h"
#endif

#include "getIntegratorSolver.h"
#include "generateMeshGraph.h"
#include "generateMassMatrix.h"

#include "massSpringSystem.h"
#include "massSpringSystemFromObjMeshConfigFile.h"
#include "massSpringStencilForceModel.h"

#include "forceModelAssembler.h"

#include "graph.h"
#include "configFile.h"

#include "listIO.h"
#include "matrixIO.h"

#include "integratorBase.h"
#include "integratorBaseSparse.h"
#include "implicitNewmarkSparse.h"

// config file
string configFilename;
char renderingMeshFilename[4096]="beam.obj";
char fixedVerticesFilename[4096]="constraints.bou";
char massSpringSystemObjConfigFilename[4096]="beam3_vox.massspring";
char initialPositionFilename[4096]="__none";
char initialVelocityFilename[4096]="__none";
char forceLoadsFilename[4096]="loads.csv";
char vegaConfigurationsFilename[4096]="vegaConfigurations.csv";
char momentRigidSectionLoadsFilename[4096]="moments.csv";
char momentRigidSectionLoadsTimedFilename[4096]="__none";
char solverMethod[4096]="implicitNewmark";
float dampingMassCoef; // Rayleigh mass damping
float dampingStiffnessCoef; // Rayleigh stiffness damping
float dampingLaplacianCoef = 0.0; // Laplacian damping (rarely used)
float deformableObjectCompliance = 0.0; // scales all user forces by the provided factor

// adjusts the stiffness of the object to cause all frequencies scale by the provided factor:
// keep it to 1.0 (except for experts)
float frequencyScaling = 1.0;
int maxIterations=1; // for implicit integration
double epsilon=0.0001; // for implicit integration
char backgroundColorString[4096] = "255 255 255";
int numInternalForceThreads=4;
int numSolverThreads=1;

// simulation
bool stopSimulation=false;
int syncTimestepWithGraphics=1;
float timeStep = 1.0 / 30;
float newmarkBeta = 0.25;
float newmarkGamma = 0.5;
int use1DNewmarkParameterFamily = 1;
int substepsPerTimeStep = 1;
double inversionThreshold;
double fps = 0.0;
double cpuLoad = 0;
double forceAssemblyTime = 0.0;
double systemSolveTime = 0.0;
int enableTextures = 0;
int staticSolver = 0;
int graphicFrame = 0;
int lockAt30Hz = 0;
int pulledVertex = -1;
int forceNeighborhoodSize = 5;
int dragStartX, dragStartY;
int explosionFlag = 0;
int timestepCounter = 0;
int subTimestepCounter = 0;
int numFixedVertices;
int * fixedVertices;
int numexternalLoads;
int * externalLoads;
int numForceLoads = 0;
double * forceLoads = nullptr;
IntegratorBase * integratorBase = nullptr;
ImplicitNewmarkSparse * implicitNewmarkSparse = nullptr;
IntegratorBaseSparse * integratorBaseSparse = nullptr;
ForceModel * forceModel = nullptr;
int enableCompressionResistance = 1;
double compressionResistance = 500;
int centralDifferencesTangentialDampingUpdateMode = 1;
int addGravity=0;
double g=9.81;

VolumetricMesh * volumetricMesh = nullptr;
Graph * meshGraph = nullptr;

enum massSpringSystemSourceType { OBJ, TETMESH, CUBICMESH, CHAIN, NONE } massSpringSystemSource = NONE;
enum deformableObjectType { STVK, COROTLINFEM, LINFEM, MASSSPRING, INVERTIBLEFEM, UNSPECIFIED } deformableObject = UNSPECIFIED;
enum invertibleMaterialType { INV_STVK, INV_NEOHOOKEAN, INV_MOONEYRIVLIN, INV_NONE } invertibleMaterial = INV_NONE;
enum solverType { IMPLICITNEWMARK, IMPLICITBACKWARDEULER, EULER, SYMPLECTICEULER, CENTRALDIFFERENCES, UNKNOWN } solver = UNKNOWN;

StencilForceModel * stencilForceModel = nullptr;
ForceModelAssembler *forceModelAssembler = nullptr;
MassSpringStencilForceModel * massSpringStencilForceModel = nullptr;

MassSpringSystem * massSpringSystem = nullptr;
SparseMatrix * massMatrix = nullptr;
SparseMatrix * LaplacianDampingMatrix = nullptr;
SparseMatrix * tangentStiffnessMatrix = nullptr;
SparseMatrix * massRemovedDofs = nullptr;
SparseMatrix * tangentStiffnessRemovedDofs = nullptr;
int numFixedDOFs;
int * fixedDOFs;

int n;
double error_q = 0.0;
double convergence_eps =4.00e-13;
double * u = nullptr;
double * uvel = nullptr;
double * uaccel = nullptr;
double * f_ext = nullptr;
double * f_extBase = nullptr;
double * uSecondary = nullptr;
double * uInitial = nullptr;
double * velInitial = nullptr;
double * q_ = nullptr;
double * q_prev = nullptr;
double * time_force_data = nullptr;

double *moment_ext = nullptr;
int *moment_location = nullptr;
int nSections=0;
int nTimeStepMoment=0;
int timed_force=0;
double *fExt_file =nullptr;
int *fExt_locations =nullptr;
int n_fExt=0;
int increaseF_extGradually=0;
int impulse_force=0;
int n_tip_mesh=0;
double beamWidth=0.0;
double totalTimeCount=0.0;
float time_for_full_load=0.0;
ros::Subscriber sub;
ros::Subscriber reset_sub;
ros::Publisher pose_pub;
ros::Publisher state_pub;
ros::Publisher episode_reset_pub;
std_msgs::Float32MultiArray pose_pub_msg;
std_msgs::Bool reset_msg;
float x,y;


// interpolation to secondary mesh
int secondaryDeformableObjectRenderingMesh_interpolation_numElementVertices;
int * secondaryDeformableObjectRenderingMesh_interpolation_vertices = nullptr;
double * secondaryDeformableObjectRenderingMesh_interpolation_weights = nullptr;

// for the two force on body
double x1_support=0.0971*0.5;
double y1_support=-0.1;
double x2_support=-0.0511;
double y2_support=-0.2;
float f1_loc=0.333333;
float f2_loc=1;
int F1=19;
int F2=26;

// for the given mesh, the parameters are
double b = 0.05;
double l = 1;
double del_x = 0.0125;
double del_y = 0.0125;

//size of the mesh
int n_x = (b/del_x) + 1;
int n_y = (l/del_y) + 1;
int n_tot = n_x*n_y;

int row_id1 = (f1_loc * l)/del_y;
int start_index1 = row_id1*n_x ;  //in vega it is +1, but in c we dont need +1

double y1_init = -row_id1*del_y;
double x1_init = 0.0;

int row_id2 = (f2_loc * l)/del_y;
int start_index2 = row_id2*n_x ;  //in vega it is +1, but in c we dont need +1

double y2_init = -row_id2*del_y;
double x2_init = 0.0;

double square(double value)
{
    // Multiply value two times
    return value*value;
}

void stopDeformations_buttonCallBack(int code);

//font is, for example, GLUT_BITMAP_9_BY_15
void resetCallBack(std_msgs::Bool msg)
{
  if(msg.data == true)
  { double q_system[3*n_tot]= { 0.0 };
    integratorBaseSparse->SetExternalForces(q_system);
    integratorBaseSparse->SetqState(q_system,q_system,q_system);
  }
  reset_msg.data = false;
  episode_reset_pub.publish(reset_msg);
  q_=integratorBase->Getq();
  std_msgs::Float32MultiArray state_msg;
  x=0.0;
  y=0.0;
  state_msg.data.push_back(x);
  state_msg.data.push_back(y);
  state_pub.publish(state_msg);
}
void forceCallBack(const std_msgs::Float32MultiArray msg)
{
  // storing prev value
  q_prev=integratorBase->Getq();

  // Added this for adding forces at each time step
  F1 = msg.data[0];
  F2 = msg.data[1];

  double denom1 = sqrt(square(x1_support-x1_init- q_prev[3*(start_index1+3)+0]) + square(y1_support-y1_init- q_prev[3*(start_index1+3)+1]));
  double denom2 = sqrt(square(x2_support-x2_init- q_prev[3*(start_index2+3)+0]) + square(y2_support-y2_init- q_prev[3*(start_index2+3)+1]));

  double f1_x = F1 * ((x1_support-x1_init - q_prev[3*(start_index1+3)+0])/denom1);
  double f1_y = F1 * ((y1_support-y1_init - q_prev[3*(start_index1+3)+1])/denom1);
  // double F1_new = sqrt(square(f1_x) + square(f1_y));

  double f2_x = F2 * ((x2_support-x2_init - q_prev[3*(start_index2+3)+0])/denom2);
  double f2_y = F2 * ((y2_support-y2_init - q_prev[3*(start_index2+3)+1])/denom2);
  // double F2_new = sqrt(square(f2_x) + square(f2_y));

  for(int i=0;i<n_x;i+=1)
  { 
    if( i == 0 || i==(n_x-1))
    {
      // cout<<start_index2+i<<endl;
      f_ext[3*(start_index1+i)+0] = f1_x/((n_x-2)*2 + 2);
      f_ext[3*(start_index1+i)+1] = f1_y/((n_x-2)*2 + 2);
      f_ext[3*(start_index2+i)+0] = f2_x/((n_x-2)*2 + 2);
      f_ext[3*(start_index2+i)+1] = f2_y/((n_x-2)*2 + 2);
    }
    else
    {
      f_ext[3*(start_index1+i)+0] = 2*f1_x/((n_x-2)*2 + 2);
      f_ext[3*(start_index1+i)+1] = 2*f1_y/((n_x-2)*2 + 2);
      f_ext[3*(start_index2+i)+0] = 2*f2_x/((n_x-2)*2 + 2);
      f_ext[3*(start_index2+i)+1] = 2*f2_y/((n_x-2)*2 + 2);
    }
  }

  integratorBaseSparse->SetExternalForces(f_ext);
  for(int i=0; i<substepsPerTimeStep; i++)
  { 
  //integratorBaseSparse->SetExternalForces(f_ext);
  int code = integratorBase->DoTimestep();
  fflush(nullptr);
  subTimestepCounter++;
  }
  timestepCounter++;
  q_=integratorBase->Getq();

  // calculating the new x and y
  {
    x=0.0;
    for(int gg=0;gg<n_x;gg++)
    {
      if(gg==0 || gg==(n_x-1))
        x=x+q_[3*(n_tot-n_x+gg)]*2; 
      else
        x=x+q_[3*(n_tot-n_x+gg)];
    }
    x=x/(2+n_x);

    y=0.0;
    for(int gg=0;gg<n_x;gg++)
    {
      if(gg==0 || gg==(n_x-1))
        y=y+q_[3*(n_tot-n_x+gg)+1]*2;
      else
        y=y+q_[3*(n_tot-n_x+gg)+1];
    }
    y=y/(2+n_x);
  }

  std_msgs::Float32MultiArray state_msg;
  //for(int i=0;i<n;i++)
  //{
  //  state_msg.data.push_back(q_[3*i]);
  //  state_msg.data.push_back(q_[3*i+1]);
  //}
  state_msg.data.push_back(x);
  state_msg.data.push_back(y);
  ros::spinOnce();
  state_pub.publish(state_msg);
  //pose_pub_msg.data={x};
  //pose_pub.publish(pose_pub_msg);
}

// called periodically by GLUT:
void idleFunction(void)
{
  ros::NodeHandle node_handle;
  cout.precision(10);
  sub = node_handle.subscribe("force", 1, forceCallBack);
  reset_sub = node_handle.subscribe("reset",1,resetCallBack);
  cout<<"node initialized. Subscribed to force"<<endl;

  //episode_reset_pub = node_handle.advertise<std_msgs::Bool>("reset", 1);
  state_pub= node_handle.advertise<std_msgs::Float32MultiArray>("state", 1);

  // reset external forces (usually to zero)
  memcpy(f_ext, f_extBase, sizeof(double) * 3 * n);
  f_ext=f_extBase;
  cout<<"timeStep is:"<<timeStep<<endl;
  integratorBase->SetTimestep(timeStep/substepsPerTimeStep);
  ros::spin();
}


/////////////////////////////////////
// program initialization
void initSimulation()
{
  if (strcmp(massSpringSystemObjConfigFilename, "__none") != 0)
    massSpringSystemSource = OBJ;

  if ((massSpringSystemSource == OBJ) || (massSpringSystemSource == TETMESH) || (massSpringSystemSource == CUBICMESH) || (massSpringSystemSource == CHAIN))
    deformableObject = MASSSPRING;

  // load mass spring system (if any)
  if (deformableObject == MASSSPRING)
  {
    switch (massSpringSystemSource)
    {
    case OBJ:
    {
      printf("Loading mass spring system from an obj file...\n");
      MassSpringSystemFromObjMeshConfigFile massSpringSystemFromObjMeshConfigFile;
      MassSpringSystemObjMeshConfiguration massSpringSystemObjMeshConfiguration;
      if (massSpringSystemFromObjMeshConfigFile.GenerateMassSpringSystem(massSpringSystemObjConfigFilename, &massSpringSystem, &massSpringSystemObjMeshConfiguration) != 0)
      {
        printf("Error initializing the mass spring system.\n");
        exit(1);
      }
      strcpy(renderingMeshFilename, massSpringSystemObjMeshConfiguration.massSpringMeshFilename);
    }
    break;

    default:
      printf("Error: mesh spring system configuration file was not specified.\n");
      exit(1);
      break;
    }

    n = massSpringSystem->GetNumParticles();

    // create the mass matrix
    massSpringSystem->GenerateMassMatrix(&massMatrix);
    // cout<<"massMatrix:";
    // massMatrix->Print();
    // create the mesh graph (used only for the distribution of user forces over neighboring vertices)
    meshGraph = new Graph(massSpringSystem->GetNumParticles(), massSpringSystem->GetNumEdges(), massSpringSystem->GetEdges());
  }

  int scaleRows = 1;
  meshGraph->GetLaplacian(&LaplacianDampingMatrix, scaleRows);
  LaplacianDampingMatrix->ScalarMultiply(dampingLaplacianCoef);

  if (!((deformableObject == MASSSPRING) && (massSpringSystemSource == CHAIN)))
  {
    // read the fixed vertices
    // 1-indexed notation
    if (strcmp(fixedVerticesFilename, "__none") == 0)
    {
      numFixedVertices = 0;
      fixedVertices = nullptr;
    }
    else
    {
      if (ListIO::load(fixedVerticesFilename, &numFixedVertices,&fixedVertices) != 0)
      {
        printf("Error reading fixed vertices.\n");
        exit(1);
      }
      ListIO::sort(numFixedVertices, fixedVertices);
    }
  }
  else
  {
    numFixedVertices = 1;
    fixedVertices = (int*) malloc (sizeof(int) * numFixedVertices);
    fixedVertices[0] = massSpringSystem->GetNumParticles();
  }
  // ListIO::print(numFixedVertices,fixedVertices);

  //fixing just the desired DOF out of 3 for each node/vertex
  numFixedDOFs=0;
  cout<<"numFixedVertices:"<<numFixedVertices<<endl;
  for (int i=0;i<numFixedVertices;i++)
  {
    if(fixedVertices[i]%10==1)
      numFixedDOFs++;
    if((fixedVertices[i]/10)%10==1)
      numFixedDOFs++;
    if((fixedVertices[i]/100)%10==1)
      numFixedDOFs++;
  } // this loop was to find the number of DOF to allocate memory accordingly

  numFixedDOFs=numFixedDOFs;
  free(fixedDOFs);
  fixedDOFs = (int*) malloc (sizeof(int) * numFixedDOFs);
  int indexCounter=0; // this variable is to store index as we find DOFs to constaint
  for(int i=0; i<numFixedVertices; i++)
  {
    if((fixedVertices[i]/100)%10==1)
    {
      fixedDOFs[indexCounter]= 3*(fixedVertices[i]/1000-1) + 0;
      indexCounter++;
    }
    if((fixedVertices[i]/10)%10==1)
    {
      fixedDOFs[indexCounter]= 3*(fixedVertices[i]/1000-1) + 1;
      indexCounter++;
    }
    if(fixedVertices[i]%10==1)
    {
      fixedDOFs[indexCounter]= 3*(fixedVertices[i]/1000-1) + 2;
      indexCounter++;
    }
  }
  ListIO::sort(numFixedDOFs, fixedDOFs);


  //finding reduced mass matrix
  // massRemovedDofs = new SparseMatrix(*massMatrix);
  // massRemovedDofs->RemoveRowsColumns(numFixedDOFs, fixedDOFs);
  // massRemovedDofs->BuildSuperMatrixIndices(numFixedDOFs, fixedDOFs, massMatrix);
  // massRemovedDofs->AssignSuperMatrix(*massMatrix);
  // massRemovedDofs->SaveToMatlabFormat("M.csv");
  // massMatrix->SaveToMatlabFormat("M2.csv");

  for(int i=0; i<numFixedVertices; i++)
    fixedVertices[i]=fixedVertices[i]/1000-1;
  printf("Boundary vertices processed.\n");

  // make room for deformation and force vectors
  u = (double*) calloc (3*n, sizeof(double));
  uvel = (double*) calloc (3*n, sizeof(double));
  uaccel = (double*) calloc (3*n, sizeof(double));
  f_ext = (double*) calloc (3*n, sizeof(double));
  f_extBase = (double*) calloc (3*n, sizeof(double));
  q_ = (double*) calloc (3*n, sizeof(double));
  q_prev = (double*) calloc (3*n, sizeof(double));

  // load initial condition
  if (strcmp(initialPositionFilename, "__none") != 0)
  {
    cout<<"here"<<endl;
    int nf;
    string line;
    string pose="";
    ifstream PoseFile(initialPositionFilename);

    //finding number of nodes in the file
    getline(PoseFile,line);
    nf=stoi(line);
    if(nf!=n)
    {
      cout<<"the file parsed is not correct.Number of particles dont match!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
      exit(1);
    }

    int i=0,j=0,cnt=0;
    uInitial = (double*) calloc (3*n, sizeof(double));
    while (getline (PoseFile, line)) {
      // Output the text file are interpreted as x,y,z and stored in uInitial
      while(line[cnt]!='\0')
      {
        if(line[cnt]==',')
        {
          uInitial[3*i+j]=stod(pose);
          pose="";
          j++;
        }
        else
          pose=pose+line[cnt];
        cnt++;
      }
      uInitial[3*i+j]=stod(pose);
      pose="";
      j=0;
      cnt=0;
      i++;
    }

    //printing the Uinitial matrix
    for(int i=0;i<n;i++)
    {
      cout<<uInitial[3*i]<<"X"<<uInitial[3*i+1]<<"Y"<<uInitial[3*i+2]<<"Z"<<endl;
    }

  }
  else
  {
    uInitial = (double*) calloc (3*n, sizeof(double));
  }

  // load initial velocity
  if (strcmp(initialVelocityFilename, "__none") != 0)
  {
    int m1, n1;
    ReadMatrixFromDisk_(initialVelocityFilename, &m1, &n1, &velInitial);
    if ((m1 != 3*n) || (n1 != 1))
    {
      printf("Error: initial position matrix size mismatch.\n");
      exit(1);
    }
  }

  // create force model, to be used by the integrator
  printf("Creating force model...\n");

  if (deformableObject == MASSSPRING)
  {
    printf("Force model: MASSSPRING\n");

    massSpringStencilForceModel = new MassSpringStencilForceModel(massSpringSystem);
    stencilForceModel = massSpringStencilForceModel;
  }

  assert(stencilForceModel != nullptr);
  forceModelAssembler = new ForceModelAssembler(stencilForceModel);
  forceModel = forceModelAssembler;

  // initialize the integrator
  printf("Initializing the integrator, n = %d...\n", n);
  printf("Solver type: %s\n", solverMethod);

  integratorBaseSparse = nullptr;
  if (solver == IMPLICITNEWMARK)
  {
    // cout<<"number of fixed vertices are:"<<numFixedDOFs<<endl;
    implicitNewmarkSparse = new ImplicitNewmarkSparse(3*n, timeStep, massMatrix, forceModel, numFixedDOFs, fixedDOFs,
       dampingMassCoef, dampingStiffnessCoef, maxIterations, epsilon, newmarkBeta, newmarkGamma, numSolverThreads);
    integratorBaseSparse = implicitNewmarkSparse;
  }

  integratorBase = integratorBaseSparse;
  if (integratorBase == nullptr)
  {
    printf("Error: failed to initialize numerical integrator.\n");
    exit(1);
  }

  // set integration parameters
  integratorBaseSparse->SetDampingMatrix(LaplacianDampingMatrix);
  integratorBase->ResetToRest();
  integratorBase->SetState(uInitial, velInitial);
  integratorBase->SetTimestep(timeStep / substepsPerTimeStep);

  if (implicitNewmarkSparse != nullptr)
  {
    implicitNewmarkSparse->UseStaticSolver(staticSolver);
    if (velInitial != nullptr)
      implicitNewmarkSparse->SetState(implicitNewmarkSparse->Getq(), velInitial);
  }

  // reading beam dimensions and some configurations file
  if (strcmp(vegaConfigurationsFilename, "__none") != 0)
  {
    //read the configuration file
    std::ifstream Dfile(vegaConfigurationsFilename);
    if (Dfile.is_open())
    {
      cout<<"reading the beam dimensions file."<<endl;
      string word;
      if(Dfile>>word)
        n_tip_mesh=stoi(word);
      else
      {
        cout<<"there is a possible error in beam dimensions file!!!"<<endl;
        exit(1);
      }
      if(Dfile>>word)
        beamWidth=stod(word);
      else
      {
        cout<<"there is a possible error in beam dimensions file!!!"<<endl;
        exit(1);
      }
      if(Dfile>>word)
        increaseF_extGradually=stoi(word);
      else
      {
        cout<<"there is a possible error in beam dimensions file!!!"<<endl;
        exit(1);
      }
      if(Dfile>>word)
        time_for_full_load=stof(word);
      else
      {
        cout<<"there is a possible error in beam dimensions file!!!"<<endl;
        exit(1);
      }
      if(Dfile>>word)
        impulse_force=stoi(word);
      else
      {
        cout<<"there is a possible error in beam dimensions file!!!"<<endl;
        exit(1);
      }
      if(Dfile>>word)
        timed_force=stoi(word);
      else
      {
        cout<<"there is a possible error in beam dimensions file!!!"<<endl;
        exit(1);
      }
      cout<<"The width of the beam is:"<<beamWidth<<"  and the number of nodes in a row is:"<<n_tip_mesh<<"and impulse force is:"<<impulse_force<<endl;
    }
    else
    {
      cout<<"error opening the beam dimensions file"<<endl;
      exit(1);
    }
  }

  // reading external moment file
  if (strcmp(momentRigidSectionLoadsFilename, "__none") != 0)
  {
    //read the moment file file
    std::ifstream Mfile(momentRigidSectionLoadsFilename);
    if (Mfile.is_open())
    {
      cout<<"reading the external moment file."<<endl;
      string word;
      Mfile>>word;
      nSections=stoi(word);
      moment_ext = (double*) calloc (nSections, sizeof(double));
      moment_location = (int*) calloc (2*nSections, sizeof(int));
      for(int i=0;i<nSections;i++)
      {
        if(Mfile>>word)
          moment_location[2*i]=stoi(word)-1;
        else
        {
          cout<<"there is a possible error in moment file!!!"<<endl;
          exit(1);
        }
        if(Mfile>>word)
          moment_location[2*i+1]=stoi(word)-1;
        else
        {
          cout<<"there is a possible error in moment file!!!"<<endl;
          exit(1);
        }
        if(Mfile>>word)
          moment_ext[i]=stod(word);
        else
        {
          cout<<"there is a possible error in moment file!!!"<<endl;
          exit(1);
        }
        cout<<"applying moment of :"<<moment_ext[i]<<" between nodes :"<<moment_location[2*i]+1<<" and "<<moment_location[2*i+1]+1<<endl;
      }
    }
    else
    {
      cout<<"error opening the external moments file"<<endl;
      exit(1);
    }
  }

  // reading external moment timed file
  if (strcmp(momentRigidSectionLoadsTimedFilename, "__none") != 0 and timed_force!=0)
  {
    //read the moment file file
    std::ifstream Mfile(momentRigidSectionLoadsTimedFilename);
    if (Mfile.is_open())
    {
      cout<<"reading the external moment timed file."<<endl;
      string word;
      Mfile>>word;
      nTimeStepMoment=stoi(word);
      Mfile>>word;
      nSections=stoi(word);
      moment_ext = (double*) calloc (nTimeStepMoment*nSections, sizeof(double));
      moment_location = (int*) calloc (2*nSections, sizeof(int));
      for(int i=0;i<nSections;i++)
      {
        if(Mfile>>word)
          moment_location[2*i]=stoi(word)-1;
        else
        {
          cout<<"there is a possible error in moment timed file!!!"<<endl;
          exit(1);
        }
        if(Mfile>>word)
          moment_location[2*i+1]=stoi(word)-1;
        else
        {
          cout<<"there is a possible error in moment timed file!!!"<<endl;
          exit(1);
        }
      }
      for(int t=0;t<nTimeStepMoment;t++)
      {
        for(int i=0;i<nSections;i++)
        {
          if(Mfile>>word)
            moment_ext[t*nSections+i]=stod(word);
          else
          {
            cout<<"there is a possible error in moment timed file!!!"<<endl;
            exit(1);
          }
          // cout<<"applying moment of :"<<moment_ext[i]<<" between nodes :"<<moment_location[2*i]+1<<" and "<<moment_location[2*i+1]+1<<endl;
        }
      }
    }
    else
    {
      cout<<"error opening the external moments timed file"<<endl;
      exit(1);
    }
  }

  // reading external load file and applying the corresponding force
  if (strcmp(forceLoadsFilename, "__none") != 0)
  {
    //read the force file
    std::ifstream Ffile(forceLoadsFilename);
    if (Ffile.is_open())
    {
      cout<<"reading the external force file."<<endl;
      string word;
      if(Ffile>>word)
        n_fExt=stoi(word);
      cout<<"the number of nodes on which f_ext is applied is:"<<n_fExt<<endl;
      fExt_file = (double*) calloc (n_fExt, sizeof(double));
      fExt_locations = (int*) calloc (n_fExt, sizeof(int));
      for(int fExt_count=0;fExt_count<n_fExt;fExt_count++)
      {
        if(Ffile>>word)
          fExt_locations[fExt_count]=stoi(word);
        else
        {
          cout<<"there is a possible error in force file!!!"<<endl;
          exit(1);
        }
        if(Ffile>>word)
          fExt_file[fExt_count]=stod(word);
        else
        {
          cout<<"there is a possible error in force file!!!"<<endl;
          exit(1);
        }
      }
    }
    else
    {
      cout<<"error opening the external force file"<<endl;
      exit(1);
    }
  }
}

// set up the configuration file
void initConfigurations()
{
  printf("Parsing configuration file %s...\n", configFilename.c_str());
  ConfigFile configFile;

  // specify the entries of the config file

  // at least one of the following must be present:
  configFile.addOptionOptional("massSpringSystemObjConfigFilename", massSpringSystemObjConfigFilename, "__none");
  configFile.addOptionOptional("solver", solverMethod, "implicitNewmark");
  configFile.addOptionOptional("initialPositionFilename", initialPositionFilename, "__none");
  configFile.addOptionOptional("initialVelocityFilename", initialVelocityFilename, "__none");
  configFile.addOptionOptional("g", &g, g);

  configFile.addOptionOptional("renderingMeshFilename", renderingMeshFilename, "__none");
  configFile.addOptionOptional("fixedVerticesFilename", fixedVerticesFilename, "__none");
  configFile.addOptionOptional("substepsPerTimeStep", &substepsPerTimeStep, substepsPerTimeStep);
  configFile.addOption("dampingMassCoef", &dampingMassCoef);
  configFile.addOption("dampingStiffnessCoef", &dampingStiffnessCoef);
  configFile.addOptionOptional("forceLoadsFilename", forceLoadsFilename, "__none");
  configFile.addOptionOptional("vegaConfigurationsFilename", vegaConfigurationsFilename, "__none");
  configFile.addOptionOptional("momentRigidSectionLoadsFilename", momentRigidSectionLoadsFilename, "__none");
  configFile.addOptionOptional("momentRigidSectionLoadsTimedFilename", momentRigidSectionLoadsTimedFilename, "__none");

  // parse the configuration file
  if (configFile.parseOptions((char*)configFilename.c_str()) != 0)
  {
    printf("Error parsing options.\n");
    exit(1);
  }

  // the config variables have now been loaded with their specified values

  // informatively print the variables (with assigned values) that were just parsed
  configFile.printOptions();

  if (strcmp(solverMethod, "implicitNewmark") == 0)
    solver = IMPLICITNEWMARK;
  if (strcmp(solverMethod, "implicitBackwardEuler") == 0)
    solver = IMPLICITBACKWARDEULER;
  if (strcmp(solverMethod, "Euler") == 0)
    solver = EULER;
  if (strcmp(solverMethod, "symplecticEuler") == 0)
    solver = SYMPLECTICEULER;
  if (strcmp(solverMethod, "centralDifferences") == 0)
    solver = CENTRALDIFFERENCES;
  if (solver == UNKNOWN)
  {
    printf("Error: unknown implicit solver specified.\n");
    exit(1);
  }
}

void deformableObjectCompliance_spinnerCallBack(int code)
{
  if (deformableObjectCompliance < 0)
    deformableObjectCompliance = 0;
}

void timeStep_spinnerCallBack(int code)
{
  if (timeStep < 0)
    timeStep = 0;

  integratorBase->SetTimestep(timeStep / substepsPerTimeStep);
}

void frequencyScaling_spinnerCallBack(int code)
{
  if (frequencyScaling < 0)
    frequencyScaling = 0;
  integratorBase->SetInternalForceScalingFactor(frequencyScaling * frequencyScaling);
}

void newmarkBeta_spinnerCallBack(int code)
{
  if (newmarkBeta < 0)
    newmarkBeta = 0;

  if (newmarkBeta > 0.5)
    newmarkBeta = 0.5;

  if (use1DNewmarkParameterFamily)
  {
    if (newmarkBeta > 0.25)
      newmarkGamma = sqrt(4.0 * newmarkBeta) - 0.5;
    else
      newmarkGamma = 0.5;
  }

  if (implicitNewmarkSparse != nullptr)
  {
    implicitNewmarkSparse->SetNewmarkBeta(newmarkBeta);
    implicitNewmarkSparse->SetNewmarkGamma(newmarkGamma);
  }
}

void newmarkGamma_spinnerCallBack(int code)
{
  if (newmarkGamma < 0.5)
    newmarkGamma = 0.5;

  if (newmarkGamma > 1.0)
    newmarkGamma = 1.0;

  if (use1DNewmarkParameterFamily)
    newmarkBeta = (newmarkGamma + 0.5) * (newmarkGamma + 0.5) / 4.0;

  if (implicitNewmarkSparse != nullptr)
  {
    implicitNewmarkSparse->SetNewmarkBeta(newmarkBeta);
    implicitNewmarkSparse->SetNewmarkGamma(newmarkGamma);
  }
}

void newmark_checkboxuse1DNewmarkParameterFamilyCallBack(int code)
{
  if (use1DNewmarkParameterFamily)
  {
    newmarkBeta = (newmarkGamma + 0.5) * (newmarkGamma + 0.5) / 4.0;

    if (implicitNewmarkSparse != nullptr)
    {
      implicitNewmarkSparse->SetNewmarkBeta(newmarkBeta);
      implicitNewmarkSparse->SetNewmarkGamma(newmarkGamma);
    }
  }
}

void rayleighMass_spinnerCallBack(int code)
{
  if (dampingMassCoef < 0)
    dampingMassCoef = 0;

  integratorBase->SetDampingMassCoef(dampingMassCoef);
}

void rayleighStiffness_spinnerCallBack(int code)
{
  if (dampingStiffnessCoef < 0)
    dampingStiffnessCoef = 0;

  integratorBase->SetDampingStiffnessCoef(dampingStiffnessCoef);
}

void timeStepSubdivisions_spinnerCallBack(int code)
{
  if (substepsPerTimeStep < 1)
    substepsPerTimeStep = 1;

  integratorBase->SetTimestep(timeStep / substepsPerTimeStep);
}

void staticSolver_checkboxCallBack(int code)
{
  implicitNewmarkSparse->UseStaticSolver(staticSolver);
}

// main function
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "vega_simulator");
  // parse command line options
  timeStep=0.1;
  substepsPerTimeStep=5;

  char configFilenameC[4096]="beam3_vox_massspring.config" ;

  printf("Starting application.\n");
  configFilename = string(configFilenameC);
  printf("Loading scene configuration from %s.\n", configFilename.c_str());

  initConfigurations(); // parse the config file
  initSimulation(); // init the simulation
  idleFunction();
  return 0;
}
