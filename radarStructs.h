/// @file      radarStructs.h
/// @author    Matteo Marone, BIT Technology Solutions GmbH
/// @copyright Apache 2.0 license
/// @date      2021-10-24
/// @brief     Interface between simulation and sensor model
/// @version   0.9

#ifndef RADARSTRUCTS_H
#define RADARSTRUCTS_H

#include <complex>
#include <vector>
#include "uuid.h"

using namespace std;

/// sensor->simulation: additional parameters needed for chirp description
typedef struct
{
  /// time of each chirp
  float t_chirp;
  /// number of frames
  unsigned int num_frames;
  /// phase sequence vector
  std::vector<std::vector<double>> Phase_sequence;
  /// power_out
  float P_out_dBm;
  /// number of configurations
  unsigned int num_Cfg;
  /// data sampling frequency
  float F_sampling;
  /// data sampling frequency
  float F_sampling_OSC;
  /// used radar phase at moment of simulation
  std::vector<double> fPhaseVector;
  /// used radar phase at moment of simulation
  std::vector<double> fTimeVector;
  /// number of samples per run
  int num_samples;
  /// time of one ramp
  double ramp_time;
  /// number of ramps per tx
  double num_ramps;
  /// bandwith
  double F_Span;
  /// time between ramps
  double T_rri;
} custom;

/// sensor -> simulation: parameters to define the properties of an antenna (gain and possible FOV)
typedef struct
{
    /// frequency of pattern in GHz
    float fFrequency;
    /// number of points in theta direction. Theta is always defined in equidistant steps from 0° to 90°
    unsigned int uSizeTheta;
    /// number of points in phi direction. Phi is always defined in equidistant steps from 0° to 360° minus one step, e.g. 0:5:355 or 0:10:350.
    unsigned int uSizePhi;
    /// complex horizontal gain Values in defined angular directions
    vector<vector<complex<float>>> fGainHorizontalMatrix;
    /// complex vertical gain Values in defined angular directions
    vector<vector<complex<float>>> fGainVerticalMatrix;
} stPattern;

/// sensor -> simulation: antenna object, which matches the stPattern properties
typedef struct
{
    /// antenna ID
    int id;
    /// reference to antenna rx-tx gain contained in stPattern
    stPattern gain;
} stAntenna;


/// sensor -> simulation: parameters to define the global FOV
typedef struct
{
    /// range gate size of radar system in present mode in m
    float fRangeGateSize;
    /// factor to calculate dopplershift in m from velocity (f*B/T); factor in s
    float fVelo2RangeFactor;
    //// all waves that are within the angular FoV should be taken into account, this includes also waves from objects that are not in the FoV, but reached by a multi-bounce
    float fStartAngleAzimuth;
    /// stop azimuth angle in deg
    float fStopAngleAzimuth;
    /// start azimuth angle in deg
    float fStartAngleElevation;
    /// stop elevation angle in deg
    float fStopAngleElevation;
    /// minimum 2-way pathlength that should be taken into account for simulation in m
    float fMinPathlength;
    /// maximum 2-way pathlength that should be taken into account for simulation in m
    float fMaxPathlength;
} stFov;


/// spacial coordinates for antenna positioning
typedef struct
{
    /// x-component
    float x;
    /// y-component
    float y;
    /// z-component
    float z;
} stPoint;


/// sensor -> simulation: antenna spatial distribution in the sensor
typedef struct
{
    ///// number of antennas; can be larger as number of physical antennas
    unsigned int uNumAntennas;
    /// ids of used  antennas (same type of antenna can be used several times)
    std::vector<stAntenna*> antennaVector;
    ///// position of phase origin of all antennas used in the array
    std::vector<stPoint*> fPositionVector;
} stArray;


/// sensor time evolution (antenna pattern vs time)
typedef struct 
{
    /// number of requiered E_field simulations
    unsigned int uNumberOfSamplingPoints;
    /// moment of simulation in µs (dimension = uNumberOfSamplingPoints)
    std::vector<double> fTimeVector;
    /// used radar frequency at moment of simulation
    std::vector<double>  fFrequencyVector;
    /// which rx antennas are active for simulation
    std::vector<std::vector<bool>> activeRxMatrix;
    /// which tx antennas are active for simulation
    std::vector<std::vector<bool>> activeTxMatrix;
    /// additional parameters needed for the simulation
    custom parameters;
} stMeasSetup;


/// sensor -> simulation: radar modes information
typedef struct
{
    /// name
    std::string Name;
    /// link to FOV
    stFov fov;
    /// antenna tx setup (sensor -> simulation)
    stArray TxArray;
    /// antenna rx setup (simulation -> sensor)
    stArray RxArray;
    /// global offset time
    float fOffsettime;
    /// measurement setup
    stMeasSetup measSetup;
} stRadarMode;

/// sensor -> simulation: overall container
typedef struct
{
    /// radar name
    std::string Name;
    /// number of radar models
    unsigned int numRadarModes;
    /// vector with radar modes
    std::vector<stRadarMode> RadarMode;
} stRadar;


/// simulation -> sensor: output parameters for binning by path length
typedef struct
{
    /// minmum path length in m
    float fMinPathLength;
    /// path length delta of given data in m
    float fDeltaPathLength;
    /// number of path lenght intervals
    unsigned int uDataMatrixLength;
    /// number of sampling points
    unsigned int uDataMatrixSamplingPoints;
} stDataInfo;


/// output from BIT to sensor model in each tx-rx antenna match
typedef struct
{
    /// this id is a reference to the index of the antenna in the stArray TxArray struct
    int iTxId;
    /// this id is a reference to the index of the antenna in the stArray RxArray struct
    int iRxId;
    /// loss of E-field due to pathlength and Antenna gain and polarization
    vector<vector<float>> fEFieldPathLoss;  // [NUMBER_SAMPLING_POINTS][NUMBER_DISCRETE_PATHLENGHTS];
    /// resulting phase due to pathlength and Antenna gain and polarization
    vector<vector<float>> fPhase;           // [NUMBER_SAMPLING_POINTS][NUMBER_DISCRETE_PATHLENGHTS];
} stTxRx;


/// output from BIT to sensor model -- general container
typedef struct
{
    /// it contains the result for each couple tx-rx antenna
    std::vector<std::vector<stTxRx>> antennaSignal; //[TX][RX]
} stDataTxRx;

/// simulation -> sensor: object information for bins depending on path length
typedef struct
{
    /// ID of the hit object
    Uuid uObjectUUID;
    /// material of the hit object
    Uuid uMaterialUUID;
} stSceneObject;

/// simulation -> sensor: ground truth information about object in bins/voxels
typedef struct
{
    /// id of the hit object
    std::vector<stSceneObject> EvaluatedObjects;
} stGroundTruth;

/// simulation -> sensor: overall simulation results container
typedef struct
{
    /// binning for path lenghts
    stDataInfo BinningInfo;
    /// e_phase and pathloss in interval of voxels
    stDataTxRx PathlossPhase;
    /// information of ground truth in interval of voxels
    std::vector<stGroundTruth> GroundTruth;
} stSimulationResults;

/// point cloud
typedef struct
{
    /// x-component
    float x;
    /// y-component
    float y;
    /// z-component
    float z;
    /// intensity
    float magnitude;
    /// speed
    float radial_speed;
    //// object uuid id
    Uuid uuid;
} stPointCloudField;

/// simulation -> sensor: final results passed through the sensor model
typedef struct
{
    /// final result for PCL library
    std::vector<stPointCloudField> pcd;
} stSensorResults;


///
typedef struct
{
 /// percentage of allowed clipping
 float clipping_threshold;
} stRunParameters;

///
typedef struct
{
 /// ratio of allowed clipping
 float clipping_ratio;
 /// scaling factor applied
 float scaling_factor;
} stReturnRunParameters;


#endif
