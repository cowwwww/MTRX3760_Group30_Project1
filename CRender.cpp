//-----------------------------------------------------------------------------
// CRender.cpp
//
// This file does the raylib drawing.
//-----------------------------------------------------------------------------

#include "CRender.h"

#include "raylib.h"

CRender::CRender()
    : mScreenWidth( 800 ),
      mScreenHeight( 600 )
{
    InitWindow( mScreenWidth, mScreenHeight, "MTRX3760 Lab 2 - Robot Simulator" );
    SetTargetFPS( 60 );
}

int CRender::GetScreenWidth() const
{
    return mScreenWidth;
}

int CRender::GetScreenHeight() const
{
    return mScreenHeight;
}

bool CRender::WindowShouldClose()
{
    bool shouldClose = ::WindowShouldClose();
    return shouldClose;
}

void CRender::CloseWindow()
{
    ::CloseWindow();
}

void CRender::BeginDrawing()
{
    ::BeginDrawing();
    ::ClearBackground( BLACK );
}

void CRender::EndDrawing()
{
    ::EndDrawing();
}

void CRender::DrawCircle( Vec2D aPosition, int aRadius, EStyle aStyle )
{
    Color colour = RAYWHITE;

    if( aStyle == WALL_ROBOT )
    {
        colour = ORANGE;
    }
    else if( aStyle == LINE_ROBOT )
    {
        colour = SKYBLUE;
    }
    else if( aStyle == SENSOR )
    {
        colour = GREEN;
    }

    ::DrawCircle( aPosition.x, aPosition.y, float( aRadius ), colour );
}

void CRender::DrawLine( Vec2D aStart, Vec2D aEnd, float aThickness, EStyle aStyle )
{
    Color colour = RAYWHITE;

    if( aStyle == FLOOR_LINE )
    {
        colour = YELLOW;
    }
    else if( aStyle == WALL_TRAIL )
    {
        colour = ORANGE;
    }
    else if( aStyle == LINE_TRAIL )
    {
        colour = SKYBLUE;
    }
    else if( aStyle == SENSOR )
    {
        colour = GREEN;
    }

    Vector2 start = { aStart.x, aStart.y };
    Vector2 end = { aEnd.x, aEnd.y };
    ::DrawLineEx( start, end, aThickness, colour );
}
