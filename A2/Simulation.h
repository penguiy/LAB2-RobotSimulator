#ifndef SIMULATION_H
#define SIMULATION_H

#include <string>
#include <vector>

#include "CRender.h"
#include "CLoopReader.h"
#include "WallFollower.h"
#include "LineFollower.h"

class Simulation
{
  public:
    Simulation();
    void Run();

  private:
    void Update();
    void Draw();
    void DrawWalls();
    void DrawLines();
    void ReportSummary() const;

    const std::string mMapFilename;
    const std::string mLineFilename;
    const double mTimeStep;
    const int mMaximumUpdates;
    const int mUpdatesPerFrame;

    CRender mRender;
    CLoopReader mWallReader;
    CLoopReader mLineReader;
    WallFollower mWallFollower;
    LineFollower mLineFollower;

    int mUpdateCount;
    bool mMapLoaded;
    bool mLineLoaded;
};

#endif