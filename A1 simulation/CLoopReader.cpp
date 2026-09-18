//-----------------------------------------------------------------------------
// CLoopReader.cpp
//-----------------------------------------------------------------------------

#include "CLoopReader.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

CLoopReader::CLoopReader()
    : mStartPose( { { 0.0f, 0.0f }, 0.0f } )
{
}

const std::string& CLoopReader::GetName() const
{
    return mName;
}

const CPose& CLoopReader::GetStartPose() const
{
    return mStartPose;
}

const std::vector<Vec2D>& CLoopReader::GetVertices() const
{
    return mVertices;
}

bool CLoopReader::ReadFile( const std::string& arFilename )
{
    bool okay = true;
    bool haveLoop = false;
    int lineNumber = 0;

    mName.clear();
    mVertices.clear();
    mStartPose = CPose{ { 0.0f, 0.0f }, 0.0f };

    std::ifstream file( arFilename.c_str() );
    if( !file )
    {
        std::cout << "CLoopReader: could not open file '" << arFilename << "'" << std::endl;
        okay = false;
    }

    std::string line;
    while( okay && std::getline( file, line ) )
    {
        ++lineNumber;

        std::string::size_type hash = line.find( '#' );
        if( hash != std::string::npos )
        {
            line = line.substr( 0, hash );
        }

        std::istringstream words( line );
        std::string keyword;

        if( words >> keyword )
        {
            if( keyword == "loop" )
            {
                if( haveLoop )
                {
                    std::cout << "CLoopReader: second loop on line " << lineNumber << std::endl;
                    okay = false;
                }
                else if( !( words >> mName ) )
                {
                    std::cout << "CLoopReader: loop needs a name on line " << lineNumber << std::endl;
                    okay = false;
                }
                else
                {
                    haveLoop = true;
                }
            }
            else if( keyword == "startpose" )
            {
                float x = 0.0f;
                float y = 0.0f;
                float headingDegrees = 0.0f;

                if( !( words >> x >> y >> headingDegrees ) || !haveLoop )
                {
                    std::cout << "CLoopReader: invalid startpose on line " << lineNumber << std::endl;
                    okay = false;
                }
                else
                {
                    const float pi = 3.14159265f;
                    mStartPose = CPose{ { x, y }, headingDegrees * pi / 180.0f };
                }
            }
            else if( keyword == "vertex" )
            {
                float x = 0.0f;
                float y = 0.0f;

                if( !( words >> x >> y ) || !haveLoop )
                {
                    std::cout << "CLoopReader: invalid vertex on line " << lineNumber << std::endl;
                    okay = false;
                }
                else
                {
                    mVertices.push_back( { x, y } );
                }
            }
            else
            {
                std::cout << "CLoopReader: unknown keyword '" << keyword
                          << "' on line " << lineNumber << std::endl;
                okay = false;
            }
        }
    }

    if( okay && mVertices.size() < 3 )
    {
        std::cout << "CLoopReader: loop needs at least three vertices" << std::endl;
        okay = false;
    }

    return okay;
}
