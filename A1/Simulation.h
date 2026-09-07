#ifndef SIMULATION_H
#define SIMULATION_H

#include "CLoopReader.h"
#include "CRender.h"
#include "WallFollower.h"

#include <string>

// A Simulation owns the map reader, renderer and wall-following robot.
// It advances the robot using a fixed timestep and draws each displayed frame.
class Simulation
{
  public:
    Simulation();

    void Run();

  private:
    void Update();
    void Draw();
    void DrawWalls();
    void ReportSummary() const;

    const std::string mMapFilename;
    const double mTimeStep;
    const int mMaximumUpdates;
    const int mUpdatesPerFrame;

    CRender mRender;
    CLoopReader mWallReader;
    WallFollower mWallFollower;

    int mUpdateCount;
    bool mMapLoaded;
};

#endif
