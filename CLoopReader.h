//-----------------------------------------------------------------------------
// CLoopReader.h
//
// Reads one loop from a .map file.
// It uses the Lab 2 file format.
//-----------------------------------------------------------------------------

#ifndef CLOOPREADER_H
#define CLOOPREADER_H

#include "CRender.h"

#include <string>
#include <vector>

struct CPose
{
    Vec2D mPosition;
    float mHeading;
};

class CLoopReader
{
    public:
        CLoopReader();

        bool ReadFile( const std::string& arFilename );

        const std::string& GetName() const;
        const CPose& GetStartPose() const;
        const std::vector<Vec2D>& GetVertices() const;

    private:
        std::string mName;
        CPose mStartPose;
        std::vector<Vec2D> mVertices;
};

#endif
