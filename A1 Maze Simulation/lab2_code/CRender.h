//-----------------------------------------------------------------------------
// CRender.h
//
// Small wrapper around raylib.
// Other files use this class for drawing.
//-----------------------------------------------------------------------------

#ifndef CRENDER_H
#define CRENDER_H

// Simple 2D point.
struct Vec2D
{
    float x;
    float y;
};

class CRender
{
    public:
        // Drawing styles used by CRender.
        enum EStyle
        {
            WALL,
            FLOOR_LINE,
            WALL_ROBOT,
            LINE_ROBOT,
            WALL_TRAIL,
            LINE_TRAIL,
            SENSOR
        };

        CRender();

        bool WindowShouldClose();
        void CloseWindow();

        void BeginDrawing();
        void EndDrawing();

        void DrawCircle( Vec2D aPosition, int aRadius, EStyle aStyle );
        void DrawLine( Vec2D aStart, Vec2D aEnd, float aThickness, EStyle aStyle );

        int GetScreenWidth() const;
        int GetScreenHeight() const;

    private:
        const int mScreenWidth;
        const int mScreenHeight;
};

#endif
