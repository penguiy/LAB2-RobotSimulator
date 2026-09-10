#include "Simulation.h"
#include <iostream>
#include <vector>

Simulation::Simulation()
    : mMapFilename("SimpleWalls.map"),
      mLineFilename("SimpleLine.map"),
      mTimeStep(0.02),
      mMaximumUpdates(3000),
      mUpdatesPerFrame(8),
      mRender(),
      mWallReader(),
      mLineReader(),
      mWallFollower(0.0, 0.0, 0.0),
      mLineFollower(0.0, 0.0, 0.0),
      mUpdateCount(0),
      mMapLoaded(false),
      mLineLoaded(false)
{
    mMapLoaded = mWallReader.ReadFile(mMapFilename);
    mLineLoaded = mLineReader.ReadFile(mLineFilename);

    if (mMapLoaded)
    {
        const CPose& StartPose = mWallReader.GetStartPose();
        mWallFollower.SetPose(StartPose.mPosition.x, StartPose.mPosition.y, StartPose.mHeading);
    }

    if (mLineLoaded)
    {
        const CPose& LineStartPose = mLineReader.GetStartPose();
        mLineFollower.SetPose(LineStartPose.mPosition.x, LineStartPose.mPosition.y, LineStartPose.mHeading);
    }
}
void Simulation::Run()
{
    if (!mMapLoaded || !mLineLoaded)
    {
        std::cout << "Simulation could not start because a map failed to load." << std::endl;
    }
    else
    {
        // Main simulation loop
        while (!mRender.WindowShouldClose() && mUpdateCount < mMaximumUpdates)
        {
            int FrameUpdateCount = 0;
            while (FrameUpdateCount < mUpdatesPerFrame && mUpdateCount < mMaximumUpdates)
            {
                Update();
                ++FrameUpdateCount;
            }
            Draw();
        }

        ReportSummary();

        // Keep rendering final positions until manually closed
        while (!mRender.WindowShouldClose())
        {
            Draw();
        }
    }

    mRender.CloseWindow();
}
void Simulation::Update()
{
    // Update the wall follower using the wall vertices
    mWallFollower.Update(mTimeStep, mWallReader.GetVertices());

    // Update the line follower using the line track vertices
    mLineFollower.Update(mTimeStep, mLineReader.GetVertices());

    ++mUpdateCount;
}

void Simulation::Draw()
{
    mRender.BeginDrawing();

    // Draw environment
    DrawWalls();
    DrawLines();

    // Draw robots
    mWallFollower.Draw(mRender);
    mLineFollower.Draw(mRender);

    mRender.EndDrawing();
}

void Simulation::DrawWalls()
{
    const std::vector<Vec2D>& Vertices = mWallReader.GetVertices();
    if (Vertices.size() >= 2)
    {
        for (std::vector<Vec2D>::size_type i = 0; i < Vertices.size(); ++i)
        {
            const std::vector<Vec2D>::size_type Next = (i + 1) % Vertices.size();
            mRender.DrawLine(Vertices[i], Vertices[Next], 4.0f, WHITE);
        }
    }
}

void Simulation::DrawLines()
{
    const std::vector<Vec2D>& Vertices = mLineReader.GetVertices();
    if (Vertices.size() >= 2)
    {
        for (std::vector<Vec2D>::size_type i = 0; i < Vertices.size(); ++i)
        {
            const std::vector<Vec2D>::size_type Next = (i + 1) % Vertices.size();
            // Draw the line as 5 units wide (using gray or a distinct color)
            mRender.DrawLine(Vertices[i], Vertices[Next], 5.0f, DARKGRAY);
        }
    }
}

void Simulation::ReportSummary() const
{
    std::cout << "Simulation complete" << std::endl;
    std::cout << "Updates completed: " << mUpdateCount << std::endl;
    std::cout << "Wall Follower collisions: " << mWallFollower.GetCollisionCount() << std::endl;
    std::cout << "Line Follower collisions: " << mLineFollower.GetCollisionCount() << std::endl;
}