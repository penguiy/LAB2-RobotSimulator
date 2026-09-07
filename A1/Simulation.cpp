#include "Simulation.h"

#include <iostream>
#include <vector>


Simulation::Simulation()
    :
        mMapFilename( "SimpleWalls.map" ),
        mTimeStep( 0.02 ),
        mMaximumUpdates( 3000 ),
        mUpdatesPerFrame( 8 ),
        mRender(),
        mWallReader(),
        mWallFollower( 0.0, 0.0, 0.0 ),
        mUpdateCount( 0 ),
        mMapLoaded( false )
{
    mMapLoaded = mWallReader.ReadFile( mMapFilename );

    if( mMapLoaded )
    {
        const CPose& StartPose = mWallReader.GetStartPose();

        mWallFollower.SetPose(
            StartPose.mPosition.x,
            StartPose.mPosition.y,
            StartPose.mHeading );
    }
}


void Simulation::Run()
{
    if( !mMapLoaded )
    {
        std::cout << "Simulation could not start because the map "
                  << "failed to load." << std::endl;
    }
    else
    {
        // Several fixed simulation updates are performed for each displayed frame, simulated timestep independent of frame rate.
        while( !mRender.WindowShouldClose() &&
               mUpdateCount < mMaximumUpdates )
        {
            int FrameUpdateCount = 0;

            while( FrameUpdateCount < mUpdatesPerFrame &&
                   mUpdateCount < mMaximumUpdates )
            {
                Update();
                ++FrameUpdateCount;
            }

            Draw();
        }

        ReportSummary();
    }

    mRender.CloseWindow();
}


void Simulation::Update()
{
    mWallFollower.Update(
        mTimeStep,
        mWallReader.GetVertices() );

    ++mUpdateCount;
}


void Simulation::Draw()
{
    mRender.BeginDrawing();

    DrawWalls();
    mWallFollower.Draw( mRender );

    mRender.EndDrawing();
}


void Simulation::DrawWalls()
{
    const std::vector<Vec2D>& Vertices =
        mWallReader.GetVertices();

    if( Vertices.size() >= 2 )
    {
        // Each vertex joins the next one; modulo closes the final vertex back
        // to the first without needing a separate DrawLine call.
        for( std::vector<Vec2D>::size_type i = 0;
             i < Vertices.size();
             ++i )
        {
            const std::vector<Vec2D>::size_type Next =
                ( i + 1 ) % Vertices.size();

            mRender.DrawLine(
                Vertices[i],
                Vertices[Next],
                4.0f,
                WHITE );
        }
    }
}


void Simulation::ReportSummary() const
{
    std::cout << "Wall-following simulation complete" << std::endl;
    std::cout << "Updates completed: "
              << mUpdateCount << std::endl;
    std::cout << "Total collisions: "
              << mWallFollower.GetCollisionCount() << std::endl;
}
