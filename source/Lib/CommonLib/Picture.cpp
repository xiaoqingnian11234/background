/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2010-2017, ITU/ISO/IEC
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
*    be used to endorse or promote products derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*/

/** \file     Picture.cpp
 *  \brief    Description of a coded picture
 */

#include "Picture.h"
#include "SEI.h"
#include "ChromaFormat.h"
#if ENABLE_WPP_PARALLELISM
#if ENABLE_WPP_STATIC_LINK
#include <atomic>
#else
#include <condition_variable>
#endif
#endif


#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
#if ENABLE_WPP_PARALLELISM
#if ENABLE_WPP_STATIC_LINK
class SyncObj
{
public:
  SyncObj() : m_Val(-1) {}
  ~SyncObj()            {}

  void reset()
  {
    m_Val = -1;
  }

  bool isReady( int64_t val ) const
  {
//    std::cout << "is ready m_Val " << m_Val << " val " << val << std::endl;
    return m_Val >= val;
  }

  void wait( int64_t idx, int ctuPosY  )
  {
    while( ! isReady( idx ) )
    {
    }
  }

  void set( int64_t val, int ctuPosY)
  {
    m_Val = val;
  }

private:
  std::atomic<int>         m_Val;
};
#else
class SyncObj
{
public:
  SyncObj() : m_Val(-1) {}
  ~SyncObj()            {}

  void reset()
  {
    std::unique_lock< std::mutex > lock( m_mutex );

    m_Val = -1;
  }

  bool isReady( int64_t val ) const
  {
    return m_Val >= val;
  }

  void wait( int64_t idx, int ctuPosY  )
  {
    std::unique_lock< std::mutex > lock( m_mutex );

    while( ! isReady( idx ) )
    {
      m_cv.wait( lock );
    }
  }

  void set( int64_t val, int ctuPosY)
  {
    std::unique_lock< std::mutex > lock( m_mutex );
    m_Val = val;
    m_cv.notify_all();
  }

private:
  int64_t                 m_Val;
  std::condition_variable m_cv;
  std::mutex              m_mutex;
};
#endif
#endif

int g_wppThreadId( 0 );
#pragma omp threadprivate(g_wppThreadId)

#if ENABLE_SPLIT_PARALLELISM
int g_splitThreadId( 0 );
#pragma omp threadprivate(g_splitThreadId)

int g_splitJobId( 0 );
#pragma omp threadprivate(g_splitJobId)
#endif

Scheduler::Scheduler() :
#if ENABLE_WPP_PARALLELISM
  m_numWppThreads( 1 ),
  m_numWppDataInstances( 1 )
#endif
#if ENABLE_SPLIT_PARALLELISM && ENABLE_WPP_PARALLELISM
  ,
#endif
#if ENABLE_SPLIT_PARALLELISM
  m_numSplitThreads( 1 )
#endif
{
}

Scheduler::~Scheduler()
{
#if ENABLE_WPP_PARALLELISM
  for( auto & so : m_SyncObjs )
  {
    delete so;
  }
  m_SyncObjs.clear();
#endif
}

#if ENABLE_SPLIT_PARALLELISM
unsigned Scheduler::getSplitDataId( int jobId ) const
{
  if( m_numSplitThreads > 1 && m_hasParallelBuffer )
  {
    int splitJobId = jobId == CURR_THREAD_ID ? g_splitJobId : jobId;

    return ( g_wppThreadId * NUM_RESERVERD_SPLIT_JOBS ) + splitJobId;
  }
  else
  {
    return 0;
  }
}

unsigned Scheduler::getSplitPicId( int tId /*= CURR_THREAD_ID */ ) const
{
  if( m_numSplitThreads > 1 && m_hasParallelBuffer )
  {
    int threadId = tId == CURR_THREAD_ID ? g_splitThreadId : tId;

    return ( g_wppThreadId * m_numSplitThreads ) + threadId;
  }
  else
  {
    return 0;
  }
}

unsigned Scheduler::getSplitJobId() const
{
  if( m_numSplitThreads > 1 )
  {
    return g_splitJobId;
  }
  else
  {
    return 0;
  }
}

void Scheduler::setSplitJobId( const int jobId )
{
  CHECK( g_splitJobId != 0 && jobId != 0, "Need to reset the jobId after usage!" );
  g_splitJobId = jobId;
}

void Scheduler::startParallel()
{
  m_hasParallelBuffer = true;
}

void Scheduler::finishParallel()
{
  m_hasParallelBuffer = false;
}

void Scheduler::setSplitThreadId( const int tId )
{
  g_splitThreadId = tId == CURR_THREAD_ID ? omp_get_thread_num() : tId;
}

#endif


#if ENABLE_WPP_PARALLELISM
unsigned Scheduler::getWppDataId( int lID ) const
{
  const int tId = lID == CURR_THREAD_ID ? g_wppThreadId : lID;

#if ENABLE_SPLIT_PARALLELISM
  if( m_numSplitThreads > 1 )
  {
    return tId * NUM_RESERVERD_SPLIT_JOBS;
  }
  else
  {
    return tId;
  }
#else
  return tId;
#endif
}

unsigned Scheduler::getWppThreadId() const
{
  return g_wppThreadId;
}

void Scheduler::setWppThreadId( const int tId )
{
  g_wppThreadId = tId == CURR_THREAD_ID ? omp_get_thread_num() : tId;

  CHECK( g_wppThreadId >= PARL_WPP_MAX_NUM_THREADS, "The WPP thread ID " << g_wppThreadId << " is invalid!" );
}
#endif

unsigned Scheduler::getDataId() const
{
#if ENABLE_SPLIT_PARALLELISM
  if( m_numSplitThreads > 1 )
  {
    return getSplitDataId();
  }
#endif
#if ENABLE_WPP_PARALLELISM
  if( m_numWppThreads > 1 )
  {
    return getWppDataId();
  }
#endif
  return 0;
}

bool Scheduler::init( const int ctuYsize, const int ctuXsize, const int numWppThreadsRunning, const int numWppExtraLines, const int numSplitThreads )
{
#if ENABLE_SPLIT_PARALLELISM
  m_numSplitThreads = numSplitThreads;
#endif
#if ENABLE_WPP_PARALLELISM
  m_firstNonFinishedLine    = 0;
  m_numWppThreadsRunning    = 1;
  m_numWppDataInstances     = numWppThreadsRunning+numWppExtraLines;
  m_numWppThreads           = numWppThreadsRunning;
  m_ctuYsize                = ctuYsize;
  m_ctuXsize                = ctuXsize;

  if( m_SyncObjs.size() == 0 )
  {
    m_SyncObjs.reserve( ctuYsize );
    for( int i = (int)m_SyncObjs.size(); i < ctuYsize; i++ )
    {
      m_SyncObjs.push_back( new SyncObj );
    }
  }
  else
  {
    CHECK( m_SyncObjs.size() != ctuYsize, "");
  }

  for( int i = 0; i < ctuYsize; i++ )
  {
    m_SyncObjs[i]->reset();
  }

  if( m_numWppThreads != m_numWppDataInstances )
  {
    m_LineDone.clear();
    m_LineDone.resize(ctuYsize, -1);

    m_LineProc.clear();
    m_LineProc.resize(ctuYsize, false);

    m_SyncObjs[0]->set(0,0);
    m_LineProc[0]=true;
  }
#endif

  return true;
}


int Scheduler::getNumPicInstances() const
{
#if !ENABLE_SPLIT_PARALLELISM
  return 1;
#elif !ENABLE_WPP_PARALLELISM
  return ( m_numSplitThreads > 1 ? m_numSplitThreads : 1 );
#else
  return m_numSplitThreads > 1 ? m_numWppDataInstances * m_numSplitThreads : 1;
#endif
}

#if ENABLE_WPP_PARALLELISM
void Scheduler::wait( const int ctuPosX, const int ctuPosY )
{
  if( m_numWppThreads == m_numWppDataInstances )
  {
    if( ctuPosY > 0 && ctuPosX+1 < m_ctuXsize)
    {
      m_SyncObjs[ctuPosY-1]->wait( ctuPosX+1, ctuPosY-1 );
    }
    return;
  }

  m_SyncObjs[ctuPosY]->wait( ctuPosX, ctuPosY );
}

void Scheduler::setReady(const int ctuPosX, const int ctuPosY)
{
  if( m_numWppThreads == m_numWppDataInstances )
  {
    m_SyncObjs[ctuPosY]->set( ctuPosX, ctuPosY);
    return;
  }

  std::unique_lock< std::mutex > lock( m_mutex );

  if( ctuPosX+1 == m_ctuXsize )
  {
    m_LineProc[ctuPosY] = true; //prevent line from be further evaluated
    m_LineDone[ctuPosY] = std::numeric_limits<int>::max();
    m_firstNonFinishedLine = ctuPosY+1;
  }
  else
  {
    m_LineDone[ctuPosY] = ctuPosX;
    m_LineProc[ctuPosY] = false;    // mark currently not processed
  }

  int lastLine = m_firstNonFinishedLine + m_numWppDataInstances;
  lastLine = std::min( m_ctuYsize, lastLine )-1-m_firstNonFinishedLine;

  m_numWppThreadsRunning--;

  Position pos;
  //if the current encoder is the last
  const bool c1 = (ctuPosY == m_firstNonFinishedLine + m_numWppThreads - 1);
  const bool c2 = (ctuPosY+1 <= m_firstNonFinishedLine+lastLine);
  const bool c3 = (ctuPosX >= m_ctuXsize/4);
  if( c1 && c2 && c3 && getNextCtu( pos, ctuPosY+1, 4 ) )
  {
    //  try to continue in the next row
    // go on in the current line
    m_SyncObjs[pos.y]->set(pos.x, pos.y);
    m_numWppThreadsRunning++;
  }
  else if( getNextCtu( pos, ctuPosY, 1 ) )
  {
    //  try to continue in the same row
    // go on in the current line
    m_SyncObjs[pos.y]->set(pos.x, pos.y);
    m_numWppThreadsRunning++;
  }
  for( int i = m_numWppThreadsRunning; i < m_numWppThreads; i++ )
  {
   // just go and get a job
    for( int y = 0; y <= lastLine; y++ )
    {
      if( getNextCtu( pos, m_firstNonFinishedLine+y, 1 ))
      {
        m_SyncObjs[pos.y]->set(pos.x, pos.y);
        m_numWppThreadsRunning++;
        break;
      }
    }
  }
}


bool Scheduler::getNextCtu( Position& pos, int ctuLine, int offset)
{
  int x = m_LineDone[ctuLine] + 1;
  if( ! m_LineProc[ctuLine] )
  {
    int maxXOffset = x+offset >= m_ctuXsize ? m_ctuXsize-1 : x+offset;
    if( (ctuLine == 0 || m_LineDone[ctuLine-1]>=maxXOffset) && (x==0 || m_LineDone[ctuLine]>=+x-1))
    {
      m_LineProc[ctuLine] = true;
      pos.x = x; pos.y = ctuLine;
      return true;
    }
  }
  return false;
}

#endif
#endif


// ---------------------------------------------------------------------------
// picture methods
// ---------------------------------------------------------------------------

#if HEVC_TILES_WPP

Tile::Tile()
: m_tileWidthInCtus     (0)
, m_tileHeightInCtus    (0)
, m_rightEdgePosInCtus  (0)
, m_bottomEdgePosInCtus (0)
, m_firstCtuRsAddr      (0)
{
}

Tile::~Tile()
{
}


TileMap::TileMap()
  : pcv(nullptr)
  , tiles(0)
  , numTiles(0)
  , numTileColumns(0)
  , numTileRows(0)
  , tileIdxMap(nullptr)
  , ctuTsToRsAddrMap(nullptr)
  , ctuRsToTsAddrMap(nullptr)
{
}

Void TileMap::create( const SPS& sps, const PPS& pps )
{
  pcv = pps.pcv;

  numTileColumns = pps.getNumTileColumnsMinus1() + 1;
  numTileRows    = pps.getNumTileRowsMinus1() + 1;
  numTiles       = numTileColumns * numTileRows;
  tiles.resize( numTiles );

  const UInt numCtusInFrame = pcv->sizeInCtus;
  tileIdxMap       = new UInt[numCtusInFrame];
  ctuTsToRsAddrMap = new UInt[numCtusInFrame+1];
  ctuRsToTsAddrMap = new UInt[numCtusInFrame+1];

  initTileMap( sps, pps );
  initCtuTsRsAddrMap();
}

Void TileMap::destroy()
{
  tiles.clear();

  if ( tileIdxMap )
  {
    delete[] tileIdxMap;
    tileIdxMap = nullptr;
  }

  if ( ctuTsToRsAddrMap )
  {
    delete[] ctuTsToRsAddrMap;
    ctuTsToRsAddrMap = nullptr;
  }

  if ( ctuRsToTsAddrMap )
  {
    delete[] ctuRsToTsAddrMap;
    ctuRsToTsAddrMap = nullptr;
  }
}

void TileMap::initTileMap( const SPS& sps, const PPS& pps )
{
  const UInt frameWidthInCtus  = pcv->widthInCtus;
  const UInt frameHeightInCtus = pcv->heightInCtus;

  if( pps.getTileUniformSpacingFlag() )
  {
    //set width and height for each (uniform) tile
    for(Int row=0; row < numTileRows; row++)
    {
      for(Int col=0; col < numTileColumns; col++)
      {
        const Int tileIdx = row * numTileColumns + col;
        tiles[tileIdx].setTileWidthInCtus(  (col+1)*frameWidthInCtus/numTileColumns - (col*frameWidthInCtus)/numTileColumns );
        tiles[tileIdx].setTileHeightInCtus( (row+1)*frameHeightInCtus/numTileRows   - (row*frameHeightInCtus)/numTileRows );
      }
    }
  }
  else
  {
    //set the width for each tile
    for(Int row=0; row < numTileRows; row++)
    {
      Int cumulativeTileWidth = 0;
      for(Int col=0; col < numTileColumns - 1; col++)
      {
        tiles[row * numTileColumns + col].setTileWidthInCtus( pps.getTileColumnWidth(col) );
        cumulativeTileWidth += pps.getTileColumnWidth(col);
      }
      tiles[row * numTileColumns + numTileColumns - 1].setTileWidthInCtus( frameWidthInCtus-cumulativeTileWidth );
    }

    //set the height for each tile
    for(Int col=0; col < numTileColumns; col++)
    {
      Int cumulativeTileHeight = 0;
      for(Int row=0; row < numTileRows - 1; row++)
      {
        tiles[row * numTileColumns + col].setTileHeightInCtus( pps.getTileRowHeight(row) );
        cumulativeTileHeight += pps.getTileRowHeight(row);
      }
      tiles[(numTileRows - 1) * numTileColumns + col].setTileHeightInCtus( frameHeightInCtus-cumulativeTileHeight );
    }
  }

  // Tile size check
  Int minWidth  = 1;
  Int minHeight = 1;
  const Int profileIdc = sps.getPTL()->getGeneralPTL()->getProfileIdc();
  if (  profileIdc == Profile::MAIN || profileIdc == Profile::MAIN10)
  {
    if (pps.getTilesEnabledFlag())
    {
      minHeight = 64  / sps.getMaxCUHeight();
      minWidth  = 256 / sps.getMaxCUWidth();
    }
  }
  for(Int row=0; row < numTileRows; row++)
  {
    for(Int col=0; col < numTileColumns; col++)
    {
      const Int tileIdx = row * numTileColumns + col;
      if(tiles[tileIdx].getTileWidthInCtus() < minWidth)   { THROW("Invalid tile size"); }
      if(tiles[tileIdx].getTileHeightInCtus() < minHeight) { THROW("Invalid tile size"); }
    }
  }

  //initialize each tile of the current picture
  for( Int row=0; row < numTileRows; row++ )
  {
    for( Int col=0; col < numTileColumns; col++ )
    {
      const Int tileIdx = row * numTileColumns + col;

      //initialize the RightEdgePosInCU for each tile
      Int rightEdgePosInCTU = 0;
      for( Int i=0; i <= col; i++ )
      {
        rightEdgePosInCTU += tiles[row * numTileColumns + i].getTileWidthInCtus();
      }
      tiles[tileIdx].setRightEdgePosInCtus(rightEdgePosInCTU-1);

      //initialize the BottomEdgePosInCU for each tile
      Int bottomEdgePosInCTU = 0;
      for( Int i=0; i <= row; i++ )
      {
        bottomEdgePosInCTU += tiles[i * numTileColumns + col].getTileHeightInCtus();
      }
      tiles[tileIdx].setBottomEdgePosInCtus(bottomEdgePosInCTU-1);

      //initialize the FirstCUAddr for each tile
      tiles[tileIdx].setFirstCtuRsAddr( (tiles[tileIdx].getBottomEdgePosInCtus() - tiles[tileIdx].getTileHeightInCtus() + 1) * frameWidthInCtus +
                                         tiles[tileIdx].getRightEdgePosInCtus()  - tiles[tileIdx].getTileWidthInCtus()  + 1);
    }
  }

  Int  columnIdx = 0;
  Int  rowIdx = 0;

  //initialize the TileIdxMap
  const UInt numCtusInFrame = pcv->sizeInCtus;
  for( Int i=0; i<numCtusInFrame; i++)
  {
    for( Int col=0; col < numTileColumns; col++)
    {
      if(i % frameWidthInCtus <= tiles[col].getRightEdgePosInCtus())
      {
        columnIdx = col;
        break;
      }
    }
    for(Int row=0; row < numTileRows; row++)
    {
      if(i / frameWidthInCtus <= tiles[row*numTileColumns].getBottomEdgePosInCtus())
      {
        rowIdx = row;
        break;
      }
    }
    tileIdxMap[i] = rowIdx * numTileColumns + columnIdx;
  }
}

void TileMap::initCtuTsRsAddrMap()
{
  //generate the Coding Order Map and Inverse Coding Order Map
  const UInt numCtusInFrame = pcv->sizeInCtus;
  for(Int ctuTsAddr=0, ctuRsAddr=0; ctuTsAddr<numCtusInFrame; ctuTsAddr++, ctuRsAddr = calculateNextCtuRSAddr(ctuRsAddr))
  {
    ctuTsToRsAddrMap[ctuTsAddr] = ctuRsAddr;
    ctuRsToTsAddrMap[ctuRsAddr] = ctuTsAddr;
  }
  ctuTsToRsAddrMap[numCtusInFrame] = numCtusInFrame;
  ctuRsToTsAddrMap[numCtusInFrame] = numCtusInFrame;
}

UInt TileMap::calculateNextCtuRSAddr( const UInt currCtuRsAddr ) const
{
  const UInt frameWidthInCtus = pcv->widthInCtus;
  UInt  nextCtuRsAddr;

  //get the tile index for the current CTU
  const UInt uiTileIdx = getTileIdxMap(currCtuRsAddr);

  //get the raster scan address for the next CTU
  if( currCtuRsAddr % frameWidthInCtus == tiles[uiTileIdx].getRightEdgePosInCtus() && currCtuRsAddr / frameWidthInCtus == tiles[uiTileIdx].getBottomEdgePosInCtus() )
  //the current CTU is the last CTU of the tile
  {
    if(uiTileIdx+1 == numTiles)
    {
      nextCtuRsAddr = pcv->sizeInCtus;
    }
    else
    {
      nextCtuRsAddr = tiles[uiTileIdx+1].getFirstCtuRsAddr();
    }
  }
  else //the current CTU is not the last CTU of the tile
  {
    if( currCtuRsAddr % frameWidthInCtus == tiles[uiTileIdx].getRightEdgePosInCtus() )  //the current CTU is on the rightmost edge of the tile
    {
      nextCtuRsAddr = currCtuRsAddr + frameWidthInCtus - tiles[uiTileIdx].getTileWidthInCtus() + 1;
    }
    else
    {
      nextCtuRsAddr = currCtuRsAddr + 1;
    }
  }

  return nextCtuRsAddr;
}

UInt TileMap::getSubstreamForCtuAddr(const UInt ctuAddr, const Bool bAddressInRaster, Slice *pcSlice) const
{
  const bool bWPPEnabled = pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag();
  UInt subStrm;

  if( (bWPPEnabled && pcv->heightInCtus > 1) || (numTiles > 1) ) // wavefronts, and possibly tiles being used.
  {
    const UInt ctuRsAddr = bAddressInRaster ? ctuAddr : getCtuTsToRsAddrMap(ctuAddr);
    const UInt tileIndex = getTileIdxMap(ctuRsAddr);

    if (bWPPEnabled)
    {
      const UInt firstCtuRsAddrOfTile     = tiles[tileIndex].getFirstCtuRsAddr();
      const UInt tileYInCtus              = firstCtuRsAddrOfTile / pcv->widthInCtus;
      const UInt ctuLine                  = ctuRsAddr / pcv->widthInCtus;
      const UInt startingSubstreamForTile = (tileYInCtus * numTileColumns) + (tiles[tileIndex].getTileHeightInCtus() * (tileIndex % numTileColumns));

      subStrm = startingSubstreamForTile + (ctuLine - tileYInCtus);
    }
    else
    {
      subStrm = tileIndex;
    }
  }
  else
  {
    subStrm = 0;
  }
  return subStrm;
}
#endif

Picture::Picture()
{
#if HEVC_TILES_WPP
  tileMap              = nullptr;
#endif
  cs                   = nullptr;
  m_bIsBorderExtended  = false;
  usedByCurr           = false;
  longTerm             = false;
  reconstructed        = false;
  neededForOutput      = false;
  referenced           = false;
  layer                = std::numeric_limits<UInt>::max();
  fieldPic             = false;
  topField             = false;
  for( int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    m_prevQP[i] = -1;
  }
}

Void Picture::create(const ChromaFormat &_chromaFormat, const Size &size, const unsigned _maxCUSize, const unsigned _margin, const bool _decoder)
{
  UnitArea::operator=( UnitArea( _chromaFormat, Area( Position{ 0, 0 }, size ) ) );
  margin            =  _margin;
  const Area a      = Area( Position(), size );
  M_BUFS( 0, PIC_RECONSTRUCTION ).create( _chromaFormat, a, _maxCUSize, _margin, MEMORY_ALIGN_DEF_SIZE );

  if( !_decoder )
  {
    M_BUFS( 0, PIC_ORIGINAL ).    create( _chromaFormat, a );
  }
#if !KEEP_PRED_AND_RESI_SIGNALS

  m_ctuArea = UnitArea( _chromaFormat, Area( Position{ 0, 0 }, Size( _maxCUSize, _maxCUSize ) ) );
#endif
}

Void Picture::destroy()
{
#if ENABLE_SPLIT_PARALLELISM
#if ENABLE_WPP_PARALLELISM
  for( int jId = 0; jId < ( PARL_SPLIT_MAX_NUM_THREADS * PARL_WPP_MAX_NUM_THREADS ); jId++ )
#else
  for( int jId = 0; jId < PARL_SPLIT_MAX_NUM_THREADS; jId++ )
#endif
#endif
  for (UInt t = 0; t < NUM_PIC_TYPES; t++)
  {
    M_BUFS( jId, t ).destroy();
  }

  if( cs )
  {
    cs->destroy();
    delete cs;
    cs = nullptr;
  }

  for( auto &ps : slices )
  {
    delete ps;
  }
  slices.clear();

  for( auto &psei : SEIs )
  {
    delete psei;
  }
  SEIs.clear();

#if HEVC_TILES_WPP
  if ( tileMap )
  {
    tileMap->destroy();
    delete tileMap;
    tileMap = nullptr;
  }
#endif
}

Void Picture::createTempBuffers( const unsigned _maxCUSize )
{
#if KEEP_PRED_AND_RESI_SIGNALS
  const Area a( Position{ 0, 0 }, lumaSize() );
#else
  const Area a = m_ctuArea.Y();
#endif

#if ENABLE_SPLIT_PARALLELISM
  scheduler.startParallel();

  for( int jId = 0; jId < scheduler.getNumPicInstances(); jId++ )
#endif
  {
    M_BUFS( jId, PIC_PREDICTION                   ).create( chromaFormat, a,   _maxCUSize );
    M_BUFS( jId, PIC_RESIDUAL                     ).create( chromaFormat, a,   _maxCUSize );
#if ENABLE_SPLIT_PARALLELISM
    if( jId > 0 ) M_BUFS( jId, PIC_RECONSTRUCTION ).create( chromaFormat, Y(), _maxCUSize, margin, MEMORY_ALIGN_DEF_SIZE );
#endif
  }

  if( cs ) cs->rebindPicBufs();
}

Void Picture::destroyTempBuffers()
{
#if ENABLE_SPLIT_PARALLELISM
  scheduler.finishParallel();

  for( int jId = 0; jId < scheduler.getNumPicInstances(); jId++ )
#endif
  for( UInt t = 0; t < NUM_PIC_TYPES; t++ )
  {
    if( t == PIC_RESIDUAL || t == PIC_PREDICTION ) M_BUFS( jId, t ).destroy();
#if ENABLE_SPLIT_PARALLELISM
    if( t == PIC_RECONSTRUCTION &&       jId > 0 ) M_BUFS( jId, t ).destroy();
#endif
  }

  if( cs ) cs->rebindPicBufs();
}

       PelBuf     Picture::getOrigBuf(const CompArea &blk)        { return getBuf(blk,  PIC_ORIGINAL); }
const CPelBuf     Picture::getOrigBuf(const CompArea &blk)  const { return getBuf(blk,  PIC_ORIGINAL); }
       PelUnitBuf Picture::getOrigBuf(const UnitArea &unit)       { return getBuf(unit, PIC_ORIGINAL); }
const CPelUnitBuf Picture::getOrigBuf(const UnitArea &unit) const { return getBuf(unit, PIC_ORIGINAL); }
       PelUnitBuf Picture::getOrigBuf()                           { return M_BUFS(0,    PIC_ORIGINAL); }
const CPelUnitBuf Picture::getOrigBuf()                     const { return M_BUFS(0,    PIC_ORIGINAL); }

       PelBuf     Picture::getPredBuf(const CompArea &blk)        { return getBuf(blk,  PIC_PREDICTION); }
const CPelBuf     Picture::getPredBuf(const CompArea &blk)  const { return getBuf(blk,  PIC_PREDICTION); }
       PelUnitBuf Picture::getPredBuf(const UnitArea &unit)       { return getBuf(unit, PIC_PREDICTION); }
const CPelUnitBuf Picture::getPredBuf(const UnitArea &unit) const { return getBuf(unit, PIC_PREDICTION); }

       PelBuf     Picture::getResiBuf(const CompArea &blk)        { return getBuf(blk,  PIC_RESIDUAL); }
const CPelBuf     Picture::getResiBuf(const CompArea &blk)  const { return getBuf(blk,  PIC_RESIDUAL); }
       PelUnitBuf Picture::getResiBuf(const UnitArea &unit)       { return getBuf(unit, PIC_RESIDUAL); }
const CPelUnitBuf Picture::getResiBuf(const UnitArea &unit) const { return getBuf(unit, PIC_RESIDUAL); }

       PelBuf     Picture::getRecoBuf(const ComponentID compID)       { return getBuf(compID,                    PIC_RECONSTRUCTION); }
const CPelBuf     Picture::getRecoBuf(const ComponentID compID) const { return getBuf(compID,                    PIC_RECONSTRUCTION); }
       PelBuf     Picture::getRecoBuf(const CompArea &blk)            { return getBuf(blk,                       PIC_RECONSTRUCTION); }
const CPelBuf     Picture::getRecoBuf(const CompArea &blk)      const { return getBuf(blk,                       PIC_RECONSTRUCTION); }
       PelUnitBuf Picture::getRecoBuf(const UnitArea &unit)           { return getBuf(unit,                      PIC_RECONSTRUCTION); }
const CPelUnitBuf Picture::getRecoBuf(const UnitArea &unit)     const { return getBuf(unit,                      PIC_RECONSTRUCTION); }
       PelUnitBuf Picture::getRecoBuf()                               { return M_BUFS(scheduler.getSplitPicId(), PIC_RECONSTRUCTION); }
const CPelUnitBuf Picture::getRecoBuf()                         const { return M_BUFS(scheduler.getSplitPicId(), PIC_RECONSTRUCTION); }

Void Picture::finalInit( const SPS& sps, const PPS& pps )
{
  for( auto &sei : SEIs )
  {
    delete sei;
  }
  SEIs.clear();
  clearSliceBuffer();

#if HEVC_TILES_WPP
  if( tileMap )
  {
    tileMap->destroy();
    delete tileMap;
    tileMap = nullptr;
  }
#endif

  const ChromaFormat chromaFormatIDC = sps.getChromaFormatIdc();
  const Int          iWidth = sps.getPicWidthInLumaSamples();
  const Int          iHeight = sps.getPicHeightInLumaSamples();

  if( cs )
  {
    cs->initStructData();
  }
  else
  {
    cs = new CodingStructure( g_globalUnitCache.cuCache, g_globalUnitCache.puCache, g_globalUnitCache.tuCache );
    cs->sps = &sps;
    cs->create( chromaFormatIDC, Area( 0, 0, iWidth, iHeight ), true );
  }

  cs->picture = this;
  cs->slice   = nullptr;  // the slices for this picture have not been set at this point. update cs->slice after swapSliceObject()
  cs->pps     = &pps;
#if HEVC_VPS
  cs->vps     = nullptr;
#endif
  cs->pcv     = pps.pcv;

#if HEVC_TILES_WPP
  tileMap = new TileMap;
  tileMap->create( sps, pps );
#endif
}

Void Picture::allocateNewSlice()
{
  slices.push_back(new Slice);
  Slice& slice = *slices.back();

  slice.setPPS( cs->pps);
  slice.setSPS( cs->sps);
  if(slices.size()>=2)
  {
    slice.copySliceInfo( slices[slices.size()-2] );
    slice.initSlice();
  }
}

Slice *Picture::swapSliceObject(Slice * p, UInt i)
{
  p->setSPS(cs->sps);
  p->setPPS(cs->pps);

  Slice * pTmp = slices[i];
  slices[i] = p;
  pTmp->setSPS(0);
  pTmp->setPPS(0);
  return pTmp;
}

void Picture::clearSliceBuffer()
{
  for (UInt i = 0; i < UInt(slices.size()); i++)
  {
    delete slices[i];
  }
  slices.clear();
}

#if ENABLE_SPLIT_PARALLELISM

void Picture::finishParallelPart( const UnitArea& area )
{
  const UnitArea clipdArea = clipArea( area, *this );
  const int      sourceID  = scheduler.getSplitPicId( 0 );
  CHECK( scheduler.getSplitJobId() > 0, "Finish-CU cannot be called from within a mode- or split-parallelized block!" );

  // distribute the reconstruction across all of the parallel workers
  for( int tId = 1; tId < scheduler.getNumSplitThreads(); tId++ )
  {
    const int destID = scheduler.getSplitPicId( tId );

    M_BUFS( destID, PIC_RECONSTRUCTION ).subBuf( clipdArea ).copyFrom( M_BUFS( sourceID, PIC_RECONSTRUCTION ).subBuf( clipdArea ) );
  }
}

#if ENABLE_WPP_PARALLELISM
void Picture::finishCtuPart( const UnitArea& ctuArea )
{
  const UnitArea clipdArea = clipArea( ctuArea, *this );
  const int      sourceID  = scheduler.getSplitPicId( 0 );
  // distribute the reconstruction across all of the parallel workers
  for( int dataId = 0; dataId < scheduler.getNumPicInstances(); dataId++ )
  {
    if( dataId == sourceID ) continue;

    M_BUFS( dataId, PIC_RECONSTRUCTION ).subBuf( clipdArea ).copyFrom( M_BUFS( sourceID, PIC_RECONSTRUCTION ).subBuf( clipdArea ) );
  }
}
#endif

#endif

void Picture::extendPicBorder()
{
  if ( m_bIsBorderExtended )
  {
    return;
  }

  for(Int comp=0; comp<getNumberValidComponents( cs->area.chromaFormat ); comp++)
  {
    ComponentID compID = ComponentID( comp );
    PelBuf p = M_BUFS( 0, PIC_RECONSTRUCTION ).get( compID );
    Pel *piTxt = p.bufAt(0,0);
    int xmargin = margin >> getComponentScaleX( compID, cs->area.chromaFormat );
    int ymargin = margin >> getComponentScaleY( compID, cs->area.chromaFormat );

    Pel*  pi = piTxt;
    // do left and right margins
    for (Int y = 0; y < p.height; y++)
    {
      for (Int x = 0; x < xmargin; x++ )
      {
        pi[ -xmargin + x ] = pi[0];
        pi[  p.width + x ] = pi[p.width-1];
      }
      pi += p.stride;
    }

    // pi is now the (0,height) (bottom left of image within bigger picture
    pi -= (p.stride + xmargin);
    // pi is now the (-marginX, height-1)
    for (Int y = 0; y < ymargin; y++ )
    {
      ::memcpy( pi + (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin << 1)));
    }

    // pi is still (-marginX, height-1)
    pi -= ((p.height-1) * p.stride);
    // pi is now (-marginX, 0)
    for (Int y = 0; y < ymargin; y++ )
    {
      ::memcpy( pi - (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin<<1)) );
    }
  }

  m_bIsBorderExtended = true;
}

PelBuf Picture::getBuf( const ComponentID compID, const PictureType &type )
{
  return M_BUFS( type == PIC_ORIGINAL ? 0 : scheduler.getSplitPicId(), type ).getBuf( compID );
}

const CPelBuf Picture::getBuf( const ComponentID compID, const PictureType &type ) const
{
  return M_BUFS( type == PIC_ORIGINAL ? 0 : scheduler.getSplitPicId(), type ).getBuf( compID );
}

PelBuf Picture::getBuf( const CompArea &blk, const PictureType &type )
{
  if( !blk.valid() )
  {
    return PelBuf();
  }

#if ENABLE_SPLIT_PARALLELISM
  const int jId = type == PIC_ORIGINAL ? 0 : scheduler.getSplitPicId();

#endif
#if !KEEP_PRED_AND_RESI_SIGNALS
  if( type == PIC_RESIDUAL || type == PIC_PREDICTION )
  {
    CompArea localBlk = blk;
    localBlk.x &= ( cs->pcv->maxCUWidthMask  >> getComponentScaleX( blk.compID, blk.chromaFormat ) );
    localBlk.y &= ( cs->pcv->maxCUHeightMask >> getComponentScaleY( blk.compID, blk.chromaFormat ) );

    return M_BUFS( jId, type ).getBuf( localBlk );
  }
#endif

  return M_BUFS( jId, type ).getBuf( blk );
}

const CPelBuf Picture::getBuf( const CompArea &blk, const PictureType &type ) const
{
  if( !blk.valid() )
  {
    return PelBuf();
  }

#if ENABLE_SPLIT_PARALLELISM
  const int jId = type == PIC_ORIGINAL ? 0 : scheduler.getSplitPicId();

#endif
#if !KEEP_PRED_AND_RESI_SIGNALS
  if( type == PIC_RESIDUAL || type == PIC_PREDICTION )
  {
    CompArea localBlk = blk;
    localBlk.x &= ( cs->pcv->maxCUWidthMask  >> getComponentScaleX( blk.compID, blk.chromaFormat ) );
    localBlk.y &= ( cs->pcv->maxCUHeightMask >> getComponentScaleY( blk.compID, blk.chromaFormat ) );

    return M_BUFS( jId, type ).getBuf( localBlk );
  }
#endif

  return M_BUFS( jId, type ).getBuf( blk );
}

PelUnitBuf Picture::getBuf( const UnitArea &unit, const PictureType &type )
{
  if( chromaFormat == CHROMA_400 )
  {
    return PelUnitBuf( chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return PelUnitBuf( chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}

const CPelUnitBuf Picture::getBuf( const UnitArea &unit, const PictureType &type ) const
{
  if( chromaFormat == CHROMA_400 )
  {
    return CPelUnitBuf( chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return CPelUnitBuf( chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}
#if ENCODE_BGPIC
Void Picture::SetOrg0(Picture* pcPic)
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piOrg;
		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;
		switch (compId)
		{
		case 0:
			
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			break;
		case 1:
			
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			break;
		case 2:
			
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			break;
		default:
			break;
		}
		for (Int i = 0; i < uiPicHeight; i++)
		{
			for (Int j = 0; j < uiPicWidth; j++)
			{
				piOrg[i * uiStride + j] = 0;
			}
		}
	}
}
Void Picture::SetReco0( Picture* pcPic)
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piOrg;
		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;
		switch (compId)
		{
		case 0:
			
			piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Y().stride;
			uiPicHeight = pcPic->getRecoBuf().Y().height;
			uiPicWidth = pcPic->getRecoBuf().Y().width;
			break;
		case 1:
			
			piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cb().stride;
			uiPicHeight = pcPic->getRecoBuf().Cb().height;
			uiPicWidth = pcPic->getRecoBuf().Cb().width;
			break;
		case 2:
			
			piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiPicHeight = pcPic->getRecoBuf().Cr().height;
			uiPicWidth = pcPic->getRecoBuf().Cr().width;
			break;
		default:
			break;
		}
		for (Int i = 0; i < uiPicHeight; i++)
		{
			for (Int j = 0; j < uiPicWidth; j++)
			{
				piOrg[i * uiStride + j] = 0;
			}
		}
	}
}
Void Picture::CopyOrg(Picture* Back, Picture* pcPic)
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piBac;
		Pel* piOrg;
		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;
		switch (compId)
		{
		case 0:
			piBac = Back->getOrigBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			break;
		case 1:
			piBac = Back->getOrigBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cb().stride;   
			uiPicHeight = pcPic->getOrigBuf().Cb().height ;
			uiPicWidth = pcPic->getOrigBuf().Cb().width ;
			break;
		case 2:
			piBac = Back->getOrigBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height ;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			break;
		default:
			break;
		}
		for (Int i = 0; i < uiPicHeight; i++)
		{
			for (Int j = 0; j < uiPicWidth; j++)
			{
				piOrg[i * uiStride + j] = piBac[i * uiStride + j];
			}
		}
	}
}
Void Picture::CopyReco2Org(Picture* Back, Picture* pcPic)
{//第二个的Org等于第一个的Reco
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piBac;
		Pel* piOrg;
		UInt uiStride,uiStride2;
		UInt uiPicHeight;
		UInt uiPicWidth;
		switch (compId)
		{
		case 0:
			piBac = Back->getRecoBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Y().stride;
			uiStride2 = Back->getRecoBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			break;
		case 1:
			piBac = Back->getRecoBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiStride2 = Back->getRecoBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			break;
		case 2:
			piBac = Back->getRecoBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiStride2 = Back->getRecoBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			break;
		default:
			break;
		}
		for (Int i = 0; i < uiPicHeight; i++)
		{
			for (Int j = 0; j < uiPicWidth; j++)
			{
				piOrg[i * uiStride + j] = piBac[i * uiStride2 + j];
			}
		}
	}
}
Void Picture::CompPicOrgDiff(Picture* Back, Picture* pcPic,Double& diff)
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piBac;
		Pel* piOrg;
		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;
		switch (compId)
		{
		case 0:
			piBac = Back->getOrigBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			break;
		case 1:
			piBac = Back->getOrigBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			break;
		case 2:
			piBac = Back->getOrigBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			break;
		default:
			break;
		}
		for (Int i = 0; i < uiPicHeight; i++)
		{
			for (Int j = 0; j < uiPicWidth; j++)
			{
				diff=(piOrg[i * uiStride + j] - piBac[i * uiStride + j])>diff?
				(piOrg[i * uiStride + j] - piBac[i * uiStride + j]):diff;
			}
		}
	}
}

Void Picture::CopyReco(Picture* Back, Picture* pcPic)
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piBac;
		Pel* piOrg;
		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;
		switch (compId)
		{
		case 0:
			piBac = Back->getRecoBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Y().stride;
			uiPicHeight = pcPic->getRecoBuf().Y().height;
			uiPicWidth = pcPic->getRecoBuf().Y().width;
			break;
		case 1:
			piBac = Back->getRecoBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cb().stride;   
			uiPicHeight = pcPic->getRecoBuf().Cb().height;
			uiPicWidth = pcPic->getRecoBuf().Cb().width ;
			break;
		case 2:
			piBac = Back->getRecoBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiPicHeight = pcPic->getRecoBuf().Cr().height;
			uiPicWidth = pcPic->getRecoBuf().Cr().width ;
			break;
		default:
			break;
		}
		for (Int i = 0; i < uiPicHeight; i++)
		{
			for (Int j = 0; j < uiPicWidth; j++)
			{
				piOrg[i * uiStride + j] = piBac[i * uiStride + j];
			}
		}
	}
}
#endif // ENCODE_BGP

#if BG_REFERENCE_SUBSTITUTION
Void Picture::CopyBack(Picture* TempPicYuv,Picture* pcPic)
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piTemp;
		Pel* PiOrg;
		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;

		switch (compId)
		{
		case 0:
			piTemp = TempPicYuv->getRecoBuf().Y().bufAt(0, 0);
			PiOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Y().stride;
			uiPicHeight = pcPic->getRecoBuf().Y().height;
			uiPicWidth = pcPic->getRecoBuf().Y().width;
			break;
		case 1:
			piTemp = TempPicYuv->getRecoBuf().Cb().bufAt(0, 0);
			PiOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cb().stride;   
			uiPicHeight = pcPic->getRecoBuf().Cb().height ;
			uiPicWidth = pcPic->getRecoBuf().Cb().width ;
			break;
		case 2:
			piTemp = TempPicYuv->getRecoBuf().Cr().bufAt(0, 0);
			PiOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiPicHeight = pcPic->getRecoBuf().Cr().height;
			uiPicWidth = pcPic->getRecoBuf().Cr().width ;
			break;
		default:
			break;
		}
		for (Int i = 0; i < uiPicHeight; i++)
		{
			for (Int j = 0; j < uiPicWidth; j++)
			{
				PiOrg[i * uiStride + j] = piTemp[i * uiStride + j];
			}
		}


	}
}
#endif

#if BG_REFERENCE_SUBSTITUTION
Void Picture::CopyBGYuv(Picture* bgPicYuv, Picture* pcPic)
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piBac;
		Pel* piOrg;
		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;
		switch (compId)
		{
		case 0:
			piBac = bgPicYuv->getRecoBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Y().stride;
			uiPicHeight = pcPic->getRecoBuf().Y().height;
			uiPicWidth = pcPic->getRecoBuf().Y().width;
			break;
		case 1:
			piBac = bgPicYuv->getRecoBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cb().stride;   
			uiPicHeight = pcPic->getRecoBuf().Cb().height ;
			uiPicWidth = pcPic->getRecoBuf().Cb().width ;
			break;
		case 2:
			piBac = bgPicYuv->getRecoBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiPicHeight = pcPic->getRecoBuf().Cr().height ;
			uiPicWidth = pcPic->getRecoBuf().Cr().width ;
			break;
		default:
			break;
		}
		for (Int i = 0; i < uiPicHeight; i++)
		{
			for (Int j = 0; j < uiPicWidth; j++)
			{			
				piOrg[i * uiStride + j] = piBac[i * uiStride + j];
			}
		}
	
	}
}
#endif

#if BG_REFERENCE_SUBSTITUTION
Void Picture::Copy2Temp(Picture* TempPicYuv, Picture* pcPic)
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piTemp;
		Pel* PiOrg;
		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;

		switch (compId)
		{
		case 0:
			piTemp = TempPicYuv->getRecoBuf().Y().bufAt(0, 0);
			PiOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Y().stride;
			uiPicHeight = pcPic->getRecoBuf().Y().height;
			uiPicWidth = pcPic->getRecoBuf().Y().width;
			break;
		case 1:
			piTemp = TempPicYuv->getRecoBuf().Cb().bufAt(0, 0);
			PiOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cb().stride;   
			uiPicHeight = pcPic->getRecoBuf().Cb().height;
			uiPicWidth = pcPic->getRecoBuf().Cb().width;
			break;
		case 2:
			piTemp = TempPicYuv->getRecoBuf().Cr().bufAt(0, 0);
			PiOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiPicHeight = pcPic->getRecoBuf().Cr().height ;
			uiPicWidth = pcPic->getRecoBuf().Cr().width ;
			break;
		default:
			break;
		}
		
		for (Int i = 0; i < uiPicHeight; i++)
		{
			for (Int j = 0; j < uiPicWidth; j++)
			{
				piTemp[i * uiStride + j] = PiOrg[i * uiStride + j];
			}
		}
	}
}
#endif
#if ENCODE_BGPIC
Void Picture::CopyPic2Reco(Picture* TempPicYuv, Picture* pcPic)
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piTemp;
		Pel* PiOrg;
		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;

		switch (compId)
		{
		case 0:
			piTemp = TempPicYuv->getRecoBuf().Y().bufAt(0, 0);
			PiOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Y().stride;
			uiPicHeight = pcPic->getRecoBuf().Y().height;
			uiPicWidth = pcPic->getRecoBuf().Y().width;
			break;
		case 1:
			piTemp = TempPicYuv->getRecoBuf().Cb().bufAt(0, 0);
			PiOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cb().stride;
			uiPicHeight = pcPic->getRecoBuf().Cb().height;
			uiPicWidth = pcPic->getRecoBuf().Cb().width;
			break;
		case 2:
			piTemp = TempPicYuv->getRecoBuf().Cr().bufAt(0, 0);
			PiOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiPicHeight = pcPic->getRecoBuf().Cr().height;
			uiPicWidth = pcPic->getRecoBuf().Cr().width;
			break;
		default:
			break;
		}

		for (Int i = 0; i < uiPicHeight; i++)
		{
			for (Int j = 0; j < uiPicWidth; j++)
			{
				piTemp[i * uiStride + j] = PiOrg[i * uiStride + j];
			}
		}
	}
}
#endif
#if HIERARCHY_GENETATE_OrgBGP
Void Picture::Copy2OrgBackPic(Picture* backPic, UInt uiW, UInt uiH, Picture *pcPic, Int level)
{
	Pel* piBac;
	Pel* piOrg;
	const Pel* piRef;
	UInt uiStride;
	UInt uiPicHeight;
	UInt uiPicWidth;
	UInt unit_len;
	for (Int compId = 0; compId < 3; compId++)
	{
		switch (compId)
		{
		case 0:
			piBac = backPic->getOrigBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
			/*if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getOrigBuf().Y().bufAt(0, 0);
			else*/
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getOrigBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			unit_len = UNIT_LEN >> level;
			break;
		case 1:
			piBac = backPic->getOrigBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);
			/*if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getOrigBuf().Y().bufAt(0, 0);
			else*/
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getOrigBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = UNIT_LEN >> (level + 1);
			uiH >>= 1;
			uiW >>= 1;
			break;
		case 2:
			piBac = backPic->getOrigBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			/*if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getOrigBuf().Y().bufAt(0, 0);
			else*/
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getOrigBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = UNIT_LEN >> (level + 1);
			//uiH >>= 1;
			//uiW >>= 1;
			break;

		default:
			break;
		}
		for (Int unit_i = 0; unit_i < unit_len && (uiH + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (uiW + unit_j < uiPicWidth); unit_j++)
			{
				piBac[uiH * uiStride + uiW + unit_j] = (piRef[uiH * uiStride + uiW + unit_j] + piOrg[uiH * uiStride + uiW + unit_j]) / 2;
			}
			piOrg += uiStride;
			piRef += uiStride;
			piBac += uiStride;
		}
	}
}
Void Picture::CopyOrgPicMean(Picture* backPic, Picture* backPic2, Picture *pcPic, UInt uiW, UInt uiH,  Int level)
{//backPic=(backPic2+pic+picref)/3
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piBac;
		Pel* piBac2;
		Pel* piOrg;
		const Pel* piRef;
		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;
		UInt unit_len;
		switch (compId)
		{
		case 0:
			piBac = backPic->getOrigBuf().Y().bufAt(0, 0);
			piBac2 = backPic2->getOrigBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getOrigBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			unit_len = UNIT_LEN >> level;
			break;
		case 1:
			piBac = backPic->getOrigBuf().Cb().bufAt(0, 0);
			piBac2 = backPic2->getOrigBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);

			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getOrigBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = UNIT_LEN >> (level + 1);
			uiH >>= 1;
			uiW >>= 1;
			break;
		case 2:
			piBac = backPic->getOrigBuf().Cr().bufAt(0, 0);
			piBac2 = backPic2->getOrigBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);

			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getOrigBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = UNIT_LEN >> (level + 1);
			break;

		default:
			break;
		}
		for (Int unit_i = 0; unit_i < unit_len && (uiH + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (uiW + unit_j < uiPicWidth); unit_j++)
			{
				piBac[uiH * uiStride + uiW + unit_j] = (piBac2[uiH * uiStride + uiW + unit_j] + piRef[uiH * uiStride + uiW + unit_j] + piOrg[uiH * uiStride + uiW + unit_j]) / 3;
			}
			piOrg += uiStride;
			piBac += uiStride;
			piBac2 += uiStride;
			piRef += uiStride;
		}
	}
}
Void Picture::CopyRecPicMean(Picture* backPic, Picture* backPic2, Picture *pcPic, UInt uiW, UInt uiH, Int level)
{//backPic=(backPic2+pic+picref)/3
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piBac;
		Pel* piBac2;
		Pel* piOrg;
		const Pel* piRef;
		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;
		UInt unit_len;
		switch (compId)
		{
		case 0:
			piBac = backPic->getRecoBuf().Y().bufAt(0, 0);
			piBac2 = backPic2->getRecoBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);
			/*if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getRecoBuf().Y().bufAt(0, 0);
			else*/
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Y().stride;
			uiPicHeight = pcPic->getRecoBuf().Y().height;
			uiPicWidth = pcPic->getRecoBuf().Y().width;
			unit_len = UNIT_LEN >> level;
			break;
		case 1:
			piBac = backPic->getRecoBuf().Cb().bufAt(0, 0);
			piBac2 = backPic2->getRecoBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);
			/*if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getRecoBuf().Y().bufAt(0, 0);
			else*/
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cb().stride;
			uiPicHeight = pcPic->getRecoBuf().Cb().height;
			uiPicWidth = pcPic->getRecoBuf().Cb().width;
			unit_len = UNIT_LEN >> (level + 1);
			uiH >>= 1;
			uiW >>= 1;
			break;
		case 2:
			piBac = backPic->getRecoBuf().Cr().bufAt(0, 0);
			piBac2 = backPic2->getRecoBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);
			/*if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getRecoBuf().Y().bufAt(0, 0);
			else*/
			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiPicHeight = pcPic->getRecoBuf().Cr().height;
			uiPicWidth = pcPic->getRecoBuf().Cr().width;
			unit_len = UNIT_LEN >> (level + 1);
			//uiH >>= 1;
			//uiW >>= 1;
			break;

		default:
			break;
		}
		for (Int unit_i = 0; unit_i < unit_len && (uiH + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (uiW + unit_j < uiPicWidth); unit_j++)
			{
				piBac[uiH * uiStride + uiW + unit_j] = (piBac2[uiH * uiStride + uiW + unit_j] + piRef[uiH * uiStride + uiW + unit_j] + piOrg[uiH * uiStride + uiW + unit_j]) / 3;
			}
			piOrg += uiStride;
			piRef += uiStride;
			piBac += uiStride;
			piBac2 += uiStride;
		}
	}
}
Bool Picture::IsEmpty(Picture* pcPic, UInt uiW, UInt uiH, Int level)/*315*/
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piPic;
		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;
		UInt unit_len;
		switch (compId)
		{
		case 0:
			piPic = pcPic->getOrigBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			unit_len = UNIT_LEN >> level;
			break;
		case 1:
			piPic = pcPic->getOrigBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = UNIT_LEN >> (level + 1);
			uiH >>= 1;
			uiW >>= 1;
			break;
		case 2:
			piPic = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = UNIT_LEN >> (level + 1);
			//uiH >>= 1;
			//uiW >>= 1;
			break;

		default:
			break;
		}
		for (Int unit_i = 0; unit_i < unit_len && (uiH + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (uiW + unit_j < uiPicWidth); unit_j++)
			{
				if (piPic[uiH * uiStride + uiW + unit_j] != 0)
					return false;
			}
			piPic += uiStride;
		}
	}
	return true;
}
#endif

#if BLOCK_GEN
Void Picture::CopyOrg2CTU(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic)
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piBac;
		Pel* piOrg;
		UInt uiStride;
		UInt uiHeight;
		UInt uiWidth;
		UInt unit_len;
		UInt H, W;
		switch (compId)
		{
		case 0:
			piBac = backPic->getOrigBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Y().stride;
			uiHeight = pcPic->getOrigBuf().Y().height;
			uiWidth = pcPic->getOrigBuf().Y().width;
			unit_len = BLOCK_CTU;
			H = uiH;
			W = uiW;
			break;
		case 1:
			piBac = backPic->getOrigBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiHeight = pcPic->getOrigBuf().Cb().height;
			uiWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = BLOCK_CTU >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		case 2:
			piBac = backPic->getOrigBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiHeight = pcPic->getOrigBuf().Cr().height;
			uiWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = BLOCK_CTU >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		default:
			break;
		}

		for (UInt uiY = 0; uiY < unit_len && (H + uiY < uiHeight); uiY++)
		{
			for (UInt uiX = 0; uiX < unit_len && (W + uiX < uiWidth); uiX++)
			{
				piBac[H * uiStride + W + uiX] = piOrg[H * uiStride + W + uiX];
			}
			piOrg += uiStride;
			piBac += uiStride;
		}
	}
}
Void Picture::CopyOrg2Block(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic)
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piBac;
		Pel* piOrg;
		UInt uiStride;
		UInt uiHeight;
		UInt uiWidth;
		UInt unit_len;
		UInt H, W;
		switch (compId)
		{
		case 0:
			piBac = backPic->getOrigBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Y().stride;
			uiHeight = pcPic->getOrigBuf().Y().height;
			uiWidth = pcPic->getOrigBuf().Y().width;
			unit_len = BLOCK_GEN_LEN;
			H = uiH;
			W = uiW;
			break;		
		case 1:
			piBac = backPic->getOrigBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiHeight = pcPic->getOrigBuf().Cb().height;
			uiWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		case 2:
			piBac = backPic->getOrigBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiHeight = pcPic->getOrigBuf().Cr().height;
			uiWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		default:
			break;
		}

		for (UInt uiY = 0; uiY < unit_len && (H + uiY < uiHeight); uiY++)
		{
			for (UInt uiX = 0; uiX < unit_len && (W + uiX < uiWidth); uiX++)
			{
				piBac[H * uiStride + W + uiX] = piOrg[H * uiStride + W + uiX];
			}
			piOrg += uiStride;
			piBac += uiStride;
		}
	}
}
Void Picture::CopyPreReco2Block(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic)
{
	Int SetZero = 0;
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piBac;
		Pel* piOrg;
		UInt uiStride;
		UInt uiHeight;
		UInt uiWidth;
		UInt unit_len;
		UInt H, W;
		switch (compId)
		{
		case 0:
			piBac = backPic->getOrigBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Y().stride;
			uiHeight = pcPic->getOrigBuf().Y().height;
			uiWidth = pcPic->getOrigBuf().Y().width;
			unit_len = BLOCK_GEN_LEN;
			H = uiH;
			W = uiW;
			break;
		case 1:
			piBac = backPic->getOrigBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiHeight = pcPic->getOrigBuf().Cb().height;
			uiWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		case 2:
			piBac = backPic->getOrigBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiHeight = pcPic->getOrigBuf().Cr().height;
			uiWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		default:
			break;
		}

		for (UInt uiY = 0; uiY < unit_len && (H + uiY < uiHeight); uiY++)
		{
			for (UInt uiX = 0; uiX < unit_len && (W + uiX < uiWidth); uiX++)
			{
				piBac[H * uiStride + W + uiX] = piOrg[H * uiStride + W + uiX]-1;
				if (SetZero <= 10)
				{
					//piBac[H * uiStride + W + uiX] = piOrg[H * uiStride + W + uiX]-1;
					//cout << "setzert" << SetZero;
				}
				SetZero++;
			}
			piOrg += uiStride;
			piBac += uiStride;
		}
	}
}
Void Picture::DrawRef(UInt uiW, UInt uiH, Picture* pcPic,Int Refnum)
{
	Int SetZero = 0;
	for (Int compId = 0; compId < 3/*只改亮度*/; compId++)
	{
		Pel* piOrg;
		UInt uiStride;
		UInt uiHeight;
		UInt uiWidth;
		UInt unit_len;
		UInt H, W;
		switch (compId)
		{
		case 0:
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Y().stride;
			uiHeight = pcPic->getOrigBuf().Y().height;
			uiWidth = pcPic->getOrigBuf().Y().width;
			unit_len = BLOCK_GEN_LEN;
			H = uiH;
			W = uiW;
			break;
		case 1:
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiHeight = pcPic->getOrigBuf().Cb().height;
			uiWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		case 2:
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiHeight = pcPic->getOrigBuf().Cr().height;
			uiWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		default:
			break;
		}

		for (UInt uiY = 0; uiY < unit_len && (H + uiY < uiHeight); uiY++)
		{
			for (UInt uiX = 0; uiX < unit_len && (W + uiX < uiWidth); uiX++)
			{
				if(Refnum>0)
					piOrg[H * uiStride + W + uiX] = 0;
				else
					piOrg[H * uiStride + W + uiX] = 255;
				if (compId != 0)
					piOrg[H * uiStride + W + uiX] = 128;
			}
			piOrg += uiStride;
		}
	}
}
Void Picture::CopyReco2CTU(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic)
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piBac;
		Pel* piOrg;
		UInt uiStride;
		UInt uiHeight;
		UInt uiWidth;
		UInt unit_len;
		UInt W, H;
		switch (compId)
		{
		case 0:
			piBac = backPic->getRecoBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Y().stride;
			uiHeight = pcPic->getRecoBuf().Y().height;
			uiWidth = pcPic->getRecoBuf().Y().width;
			unit_len = BLOCK_CTU;
			H = uiH;
			W = uiW;
			break;
		case 1:
			piBac = backPic->getRecoBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Cb().stride;
			uiHeight = pcPic->getRecoBuf().Cb().height;
			uiWidth = pcPic->getRecoBuf().Cb().width;
			unit_len = BLOCK_CTU >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		case 2:
			piBac = backPic->getRecoBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiHeight = pcPic->getRecoBuf().Cr().height;
			uiWidth = pcPic->getRecoBuf().Cr().width;
			unit_len = BLOCK_CTU >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		default:
			break;
		}

		for (UInt uiY = 0; uiY < unit_len && (H + uiY < uiHeight); uiY++)
		{

			for (UInt uiX = 0; uiX < unit_len && (W + uiX < uiWidth); uiX++)
			{
				piBac[H * uiStride + W + uiX] = piOrg[H * uiStride + W + uiX];
			}
			piOrg += uiStride;
			piBac += uiStride;
		}
	}
}
Void Picture::CopyReco2Block(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic)
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* piBac;
		Pel* piOrg;
		UInt uiStride;
		UInt uiHeight;
		UInt uiWidth;
		UInt unit_len;
		UInt W, H;
		switch (compId)
		{
		case 0:
			piBac = backPic->getRecoBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Y().stride;
			uiHeight = pcPic->getRecoBuf().Y().height;
			uiWidth = pcPic->getRecoBuf().Y().width;
			unit_len = BLOCK_GEN_LEN;
			H = uiH;
			W = uiW;
			break;
		case 1:
			piBac = backPic->getRecoBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Cb().stride;
			uiHeight = pcPic->getRecoBuf().Cb().height;
			uiWidth = pcPic->getRecoBuf().Cb().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		case 2:
			piBac = backPic->getRecoBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiHeight = pcPic->getRecoBuf().Cr().height;
			uiWidth = pcPic->getRecoBuf().Cr().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		default:
			break;
		}

		for (UInt uiY = 0; uiY < unit_len && (H + uiY < uiHeight); uiY++)
		{
			
			for (UInt uiX = 0; uiX < unit_len && (W + uiX < uiWidth); uiX++)
			{
				if(piOrg[H * uiStride + W + uiX]!=0)
					piBac[H * uiStride + W + uiX] = piOrg[H * uiStride + W + uiX];
			}
			piOrg += uiStride;
			piBac += uiStride;
		}
	}
}
Void Picture::DeleteOrg(Picture* pcPic)
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* PiOrg;
		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;

		switch (compId)
		{
		case 0:
			PiOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			break;
		case 1:
			PiOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			break;
		case 2:
			PiOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			break;
		default:
			break;
		}

		for (Int i = 0; i < uiPicHeight; i++)
		{
			for (Int j = 0; j < uiPicWidth; j++)
			{
				PiOrg[i * uiStride + j] = 0;
			}
		}
	}
}
Void Picture::DeleteReco(Picture* pcPic)
{
	for (Int compId = 0; compId < 3; compId++)
	{
		Pel* PiOrg;
		UInt uiStride;
		UInt uiPicHeight;
		UInt uiPicWidth;

		switch (compId)
		{
		case 0:
			PiOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Y().stride;
			uiPicHeight = pcPic->getRecoBuf().Y().height;
			uiPicWidth = pcPic->getRecoBuf().Y().width;
			break;
		case 1:
			PiOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cb().stride;
			uiPicHeight = pcPic->getRecoBuf().Cb().height;
			uiPicWidth = pcPic->getRecoBuf().Cb().width;
			break;
		case 2:
			PiOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiPicHeight = pcPic->getRecoBuf().Cr().height;
			uiPicWidth = pcPic->getRecoBuf().Cr().width;
			break;
		default:
			break;
		}

		for (Int i = 0; i < uiPicHeight; i++)
		{
			for (Int j = 0; j < uiPicWidth; j++)
			{
				PiOrg[i * uiStride + j] = 0;
			}
		}
	}
}

#endif

#if OrgBG_BLOCK_SUBSTITUTION
Void Picture::Copy2OrgBackPic(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic)
{
	Pel* piBac;
	Pel* piOrg;
	UInt uiStride;
	UInt uiHeight;
	UInt uiWidth;
	UInt unit_len;
	for (Int compId = 0; compId < 3; compId++)
	{
		switch (compId)
		{
		case 0:
			piBac = backPic->getOrigBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Y().stride;
			uiHeight = pcPic->getOrigBuf().Y().height;
			uiWidth = pcPic->getOrigBuf().Y().width;
			unit_len = BLOCK_LEN;
			break;
		case 1:
			piBac = backPic->getOrigBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiHeight = pcPic->getOrigBuf().Cb().height;
			uiWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = BLOCK_LEN >> 1;  //>>1
			uiH >>= 1;
			uiW >>= 1;
			break;
		case 2:
			piBac = backPic->getOrigBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiHeight = pcPic->getOrigBuf().Cr().height;
			uiWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = BLOCK_LEN >> 1;
			//uiH >>= 1;
			//uiW >>= 1;
			break;
		default:
			break;
		}

		for (UInt uiY = 0; uiY < unit_len && (uiH + uiY < uiHeight); uiY++)
		{
			for (UInt uiX = 0; uiX < unit_len && (uiW + uiX < uiWidth); uiX++)
			{
				piBac[uiH * uiStride + uiW + uiX] = piOrg[uiH * uiStride + uiW + uiX];
			}
			piOrg += uiStride;
			piBac += uiStride;
		}
	}
}

#endif

#if BG_BLOCK_SUBSTITUTION
Void Picture::Copy2BackPic(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic)
{//backPic = pcPic in compId
	Pel* piBac;
	Pel* piOrg;
	UInt uiStride;
	UInt uiHeight;
	UInt uiWidth;
	UInt unit_len;
	for (Int compId = 0; compId < 3; compId++)
	{
		switch (compId)
		{
		case 0:
			piBac = backPic->getRecoBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Y().stride;
			uiHeight = pcPic->getRecoBuf().Y().height;
			uiWidth = pcPic->getRecoBuf().Y().width;
			unit_len = BLOCK_LEN;
			break;
		case 1:
			piBac = backPic->getRecoBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Cb().stride;
			uiHeight = pcPic->getRecoBuf().Cb().height;
			uiWidth = pcPic->getRecoBuf().Cb().width;
			unit_len = BLOCK_LEN >> 1;
			uiH >>= 1;
			uiW >>= 1;
			break;
		case 2:
			piBac = backPic->getRecoBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiHeight = pcPic->getRecoBuf().Cr().height;
			uiWidth = pcPic->getRecoBuf().Cr().width;
			unit_len = BLOCK_LEN >> 1;
			//uiH >>= 1;
			//uiW >>= 1;
			break;
		default:
			break;
		}

		for (UInt uiY = 0; uiY < unit_len && (uiH + uiY < uiHeight); uiY++)
		{
			for (UInt uiX = 0; uiX < unit_len && (uiW + uiX < uiWidth); uiX++)
			{
				piBac[uiH * uiStride + uiW + uiX] = piOrg[uiH * uiStride + uiW + uiX];
			}
			piOrg += uiStride;
			piBac += uiStride;
		}
	}
}

#endif

#if HIERARCHY_GENETATE_BGP
Void Picture::Copy2BackPic(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic, Int level)
{ //Bac = ( Ref + Org )/2
	Pel* piBac;
	Pel* piOrg;
	const Pel* piRef;
	UInt uiStride;
	UInt uiPicHeight;
	UInt uiPicWidth;
	UInt unit_len;
	for (Int compId = 0; compId < 3; compId++)
	{
		switch (compId)
		{
		case 0:
			piBac = backPic->getRecoBuf().Y().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);

			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Y().stride;
			uiPicHeight = pcPic->getRecoBuf().Y().height;
			uiPicWidth = pcPic->getRecoBuf().Y().width;
			unit_len = UNIT_LEN >> level;
			break;
		case 1:
			piBac = backPic->getRecoBuf().Cb().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);

			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Cb().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Cb().stride;
			uiPicHeight = pcPic->getRecoBuf().Cb().height;
			uiPicWidth = pcPic->getRecoBuf().Cb().width;
			unit_len = UNIT_LEN >> (level + 1);
			uiH >>= 1;
			uiW >>= 1;
			break;
		case 2:
			piBac = backPic->getRecoBuf().Cr().bufAt(0, 0);
			piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);

			piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Cr().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiPicHeight = pcPic->getRecoBuf().Cr().height;
			uiPicWidth = pcPic->getRecoBuf().Cr().width;
			unit_len = UNIT_LEN >> (level + 1);
			//uiH >>= 1;
			//uiW >>= 1;
			break;
		default:
			break;
		}

		for (Int unit_i = 0; unit_i < unit_len && (uiH + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (uiW + unit_j < uiPicWidth); unit_j++)
			{
				piBac[uiH * uiStride + uiW + unit_j] = (piRef[uiH * uiStride + uiW + unit_j] + piOrg[uiH * uiStride + uiW + unit_j]) / 2;
			}
			piOrg += uiStride;
			piRef += uiStride;
			piBac += uiStride;
		}
	}
}
#endif

#if HIERARCHY_GENETATE_OrgBGP
Void Picture::xCompDiffOrg(UInt uiW, UInt uiH, Picture* pcPic, double& diff, Int level)
{
	Pel* piOrg;
	const Pel* piRef;
	UInt uiStride;
	UInt uiPicHeight;
	UInt uiPicWidth;
	UInt unit_len;
	for (Int compId = 0; compId < 3; compId++)
	{
		switch (compId)
		{
		case 0:
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);

			if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getOrigBuf().Y().bufAt(0, 0);
			else
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getOrigBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			unit_len = UNIT_LEN >> level;
			break;
		case 1:
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);

			if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getOrigBuf().Cb().bufAt(0, 0);
			else
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getOrigBuf().Cb().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = UNIT_LEN >> (level + 1);
			uiH >>= 1;
			uiW >>= 1;
			break;
		case 2:
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getOrigBuf().Cr().bufAt(0, 0);
			else
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getOrigBuf().Cr().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = UNIT_LEN >> (level + 1);
			//uiH >>= 1;
			//uiW >>= 1;
			break;
		default:
			break;
		}

		for (Int unit_i = 0; unit_i < unit_len && (uiH + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (uiW + unit_j < uiPicWidth); unit_j++)
			{
				diff = abs(piRef[uiH * uiStride + uiW + unit_j] - piOrg[uiH * uiStride + uiW + unit_j]) > diff ?
					abs(piRef[uiH * uiStride + uiW + unit_j] - piOrg[uiH * uiStride + uiW + unit_j]) : diff;
			}
			piOrg += uiStride;
			piRef += uiStride;
		}
	}
}
Void Picture::xCompDiffBlock(Picture* PicYuvOrg, UInt uiW, UInt uiH, Picture* pcPic, double& diff, Int level)
{//生成背景Org时 计算块的差值
	Pel* piOrg;
	Pel* YuvOrg;
	UInt uiStride;
	UInt uiPicHeight;
	UInt uiPicWidth;
	UInt unit_len;
	for (Int compId = 0; compId < 3; compId++)
	{
		switch (compId)
		{
		case 0:
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
			YuvOrg = PicYuvOrg->getOrigBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			unit_len = UNIT_LEN >> level;
			break;
		case 1:
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);
			YuvOrg = PicYuvOrg->getOrigBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = UNIT_LEN >> (level + 1);
			uiH >>= 1;
			uiW >>= 1;
			break;
		case 2:
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			YuvOrg = PicYuvOrg->getOrigBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = UNIT_LEN >> (level + 1);
			//uiH >>= 1;
			//uiW >>= 1;
			break;
		default:
			break;
		}

		for (Int unit_i = 0; unit_i < unit_len && (uiH + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (uiW + unit_j < uiPicWidth); unit_j++)
			{
				diff = abs(YuvOrg[uiH * uiStride + uiW + unit_j] - piOrg[uiH * uiStride + uiW + unit_j]) > diff ?
					abs(YuvOrg[uiH * uiStride + uiW + unit_j] - piOrg[uiH * uiStride + uiW + unit_j]) : diff;
			}
			piOrg += uiStride;
			YuvOrg += uiStride;
		}
	}
}
#endif

#if HIERARCHY_GENETATE_BGP
Void Picture::xCompDiff(UInt uiW, UInt uiH, Picture* pcPic, double& diff, Int level)

{
	Pel* piOrg;
	const Pel* piRef;
	UInt uiStride;
	UInt uiPicHeight;
	UInt uiPicWidth;
	UInt unit_len;
	for (Int compId = 0; compId < 3; compId++)
	{
		switch (compId)
		{
		case 0:
			piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);
			if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
			{
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getRecoBuf().Y().bufAt(0, 0);
			}
			else
			{
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Y().bufAt(0, 0);
			}

			uiStride = pcPic->getRecoBuf().Y().stride;
			uiPicHeight = pcPic->getRecoBuf().Y().height;
			uiPicWidth = pcPic->getRecoBuf().Y().width;
			unit_len = UNIT_LEN >> level;
			break;
		case 1:
			piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);
			if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getRecoBuf().Cb().bufAt(0, 0);
			else
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Cb().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Cb().stride;
			uiPicHeight = pcPic->getRecoBuf().Cb().height;
			uiPicWidth = pcPic->getRecoBuf().Cb().width;
			unit_len = UNIT_LEN >> (level + 1);
			uiH >>= 1;
			uiW >>= 1;
			break;
		case 2:
			piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);
			if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getRecoBuf().Cr().bufAt(0, 0);
			else
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Cr().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiPicHeight = pcPic->getRecoBuf().Cr().height;
			uiPicWidth = pcPic->getRecoBuf().Cr().width;
			unit_len = UNIT_LEN >> (level + 1);
			//uiH >>= 1;
			//uiW >>= 1;
			break;
		default:
			break;
		}

		for (Int unit_i = 0; unit_i < unit_len && (uiH + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (uiW + unit_j < uiPicWidth); unit_j++)
			{
				diff = abs(piRef[uiH * uiStride + uiW + unit_j] - piOrg[uiH * uiStride + uiW + unit_j]) > diff ?
					abs(piRef[uiH * uiStride + uiW + unit_j] - piOrg[uiH * uiStride + uiW + unit_j]) : diff;
			}
			piOrg += uiStride;
			piRef += uiStride;
		}
	}
}
#endif
#if BLOCK_GEN
Void Picture::CompBlockDiff(UInt uiW, UInt uiH, Picture* pcPic, double& diff)
{
	
	Pel* piOrg;
	const Pel* piRef;
	UInt uiStride;
	UInt uiPicHeight;
	UInt uiPicWidth;
	UInt unit_len;
	for (Int compId = 0; compId < 3; compId++)
	{
		switch (compId)
		{
		case 0:
			piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);
			if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getRecoBuf().Y().bufAt(0, 0);
			else
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Y().stride;
			uiPicHeight = pcPic->getRecoBuf().Y().height;
			uiPicWidth = pcPic->getRecoBuf().Y().width;
			unit_len = BLOCK_GEN_LEN;
			break;
		case 1:
			piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);
			if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getRecoBuf().Cb().bufAt(0, 0);
			else
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Cb().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Cb().stride;
			uiPicHeight = pcPic->getRecoBuf().Cb().height;
			uiPicWidth = pcPic->getRecoBuf().Cb().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			uiH >>= 1;
			uiW >>= 1;
			break;
		case 2:
			piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);
			if (pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2))
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 2)->getRecoBuf().Cr().bufAt(0, 0);
			else
				piRef = pcPic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf().Cr().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiPicHeight = pcPic->getRecoBuf().Cr().height;
			uiPicWidth = pcPic->getRecoBuf().Cr().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			//uiH >>= 1;
			//uiW >>= 1;
			break;
		default:
			break;
		}

		for (Int unit_i = 0; unit_i < unit_len && (uiH + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (uiW + unit_j < uiPicWidth); unit_j++)
			{
				diff = abs(piRef[uiH * uiStride + uiW + unit_j] - piOrg[uiH * uiStride + uiW + unit_j]) > diff ?
					abs(piRef[uiH * uiStride + uiW + unit_j] - piOrg[uiH * uiStride + uiW + unit_j]) : diff;
			}
			piOrg += uiStride;
			piRef += uiStride;
		}
	}
}
Void Picture::CopyBlock( UInt uiW, UInt uiH, Picture* PicYuvOrg, Picture* pcPic)
{

	Pel* piOrg;
	Pel* YuvOrg;
	UInt uiStride;
	UInt uiPicHeight;
	UInt uiPicWidth;
	UInt unit_len;
	for (Int compId = 0; compId < 3; compId++)
	{
		switch (compId)
		{
		case 0:
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
			YuvOrg = PicYuvOrg->getOrigBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			unit_len = BLOCK_GEN_LEN;
			break;
		case 1:
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);
			YuvOrg = PicYuvOrg->getOrigBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			uiH >>= 1;
			uiW >>= 1;
			break;
		case 2:
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			YuvOrg = PicYuvOrg->getOrigBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			//uiH >>= 1;
			//uiW >>= 1;
			break;
		default:
			break;
		}

		for (Int unit_i = 0; unit_i < unit_len && (uiH + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (uiW + unit_j < uiPicWidth); unit_j++)
			{
				YuvOrg[uiH * uiStride + uiW + unit_j] = piOrg[uiH * uiStride + uiW + unit_j];
			}
			piOrg += uiStride;
			YuvOrg += uiStride;
		}
	}
}
Void Picture::CompBlockPicBgPdpp(UInt uiW, UInt uiH, Picture* pcPic, Picture* PicYuvOrg, Double dpp, Double& P)
{

	Pel* piOrg;
	Pel* piRef;
	UInt uiStride;
	UInt uiPicHeight;
	UInt uiPicWidth;
	UInt unit_len;
	UInt H, W;
	Double numslowerdpp = 0, nums = 0;
	for (Int compId = 0; compId < 3; compId++)
	{
		switch (compId)
		{
		case 0:
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
			piRef = PicYuvOrg->getOrigBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			unit_len = BLOCK_GEN_LEN;
			H = uiH;
			W = uiW;
			break;
		case 1:
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);
			piRef = PicYuvOrg->getOrigBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		case 2:
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			piRef = PicYuvOrg->getOrigBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		default:
			break;
		}

		for (Int unit_i = 0; unit_i < unit_len && (H + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (W + unit_j < uiPicWidth); unit_j++)
			{
				double diff = (piRef[H * uiStride + W + unit_j] - piOrg[H * uiStride + W + unit_j])*
					(piRef[H * uiStride + W + unit_j] - piOrg[H * uiStride + W + unit_j]);
				if (diff <= dpp)
				{
					numslowerdpp++;
				}
				nums++;
			}
			piOrg += uiStride;
			piRef += uiStride;
		}
	}
	P = numslowerdpp / nums;
}
Void Picture::CompBlockPicdpp(UInt uiW, UInt uiH, Picture* pcPic, double& dpp)
{
	Pel* piOrg;
	Pel* piReco;
	UInt uiStride1,uiStride2;
	UInt uiPicHeight;
	UInt uiPicWidth;
	UInt unit_len;
	UInt H, W;
	Int nums = 0;
	for (Int compId = 0; compId < 3; compId++)
	{
		switch (compId)
		{
		case 0:
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
			piReco = pcPic->getRecoBuf().Y().bufAt(0, 0);

			uiStride1 = pcPic->getOrigBuf().Y().stride;
			uiStride2 = pcPic->getRecoBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			unit_len = BLOCK_GEN_LEN;
			H = uiH;
			W = uiW;
			break;
		case 1:
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);
			piReco = pcPic->getRecoBuf().Cb().bufAt(0, 0);

			uiStride1 = pcPic->getOrigBuf().Cb().stride;
			uiStride2 = pcPic->getRecoBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		case 2:
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			piReco = pcPic->getRecoBuf().Cr().bufAt(0, 0);

			uiStride1 = pcPic->getOrigBuf().Cr().stride;
			uiStride2 = pcPic->getRecoBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		default:
			break;
		}

		for (Int unit_i = 0; unit_i < unit_len && (H + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (W + unit_j < uiPicWidth); unit_j++)
			{
				dpp = dpp + (piReco[H * uiStride2 + W + unit_j] - piOrg[H * uiStride1 + W + unit_j])*
					(piReco[H * uiStride2 + W + unit_j] - piOrg[H * uiStride1 + W + unit_j]);
				nums++;
			}
			piOrg += uiStride1;
			piReco += uiStride2;
		}
	}
	dpp = dpp / nums;
}

Void Picture::CompBlockPicbgdpp(UInt uiW, UInt uiH, Picture* pcPic, Picture* bgpic, double& dpp)
{

	Pel* piOrg;
	Pel* PiBg;
	UInt uiStride;
	UInt uiPicHeight;
	UInt uiPicWidth;
	UInt unit_len;
	UInt H, W;
	Int nums = 0;
	for (Int compId = 0; compId < 1; compId++)
	{
		switch (compId)
		{
		case 0:
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
			PiBg = bgpic->getOrigBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			unit_len = BLOCK_GEN_LEN;
			H = uiH;
			W = uiW;
			break;
		case 1:
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);
			PiBg = bgpic->getOrigBuf().Cb().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		case 2:
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			PiBg = bgpic->getOrigBuf().Cr().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		default:
			break;
		}

		for (Int unit_i = 0; unit_i < unit_len && (H + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (W + unit_j < uiPicWidth); unit_j++)
			{
				dpp = dpp + abs(PiBg[H * uiStride + W + unit_j] - piOrg[H * uiStride + W + unit_j]);
				nums++;
			}
			piOrg += uiStride;
			PiBg += uiStride;
		}
	}
	dpp = dpp / nums;
}

Void Picture::CompBlockPicOrgDiff(UInt uiW, UInt uiH, Picture* pcPic,Picture* PicYuvOrg, double& diff)
{
	Pel* piOrg;
	Pel* piRef;
	UInt uiStride;
	UInt uiPicHeight;
	UInt uiPicWidth;
	UInt unit_len;
	UInt H, W;
	for (Int compId = 0; compId < 3; compId++)
	{
		switch (compId)
		{
		case 0:
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
			piRef = PicYuvOrg->getOrigBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			unit_len = BLOCK_GEN_LEN;
			H = uiH;
			W = uiW;
			break;
		case 1:
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);
			piRef = PicYuvOrg->getOrigBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		case 2:
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			piRef = PicYuvOrg->getOrigBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		default:
			break;
		}

		for (Int unit_i = 0; unit_i < unit_len && (H + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (W + unit_j < uiPicWidth); unit_j++)
			{
				diff = abs(piRef[H * uiStride + W + unit_j] - piOrg[H * uiStride + W + unit_j]) > diff ?
					abs(piRef[H * uiStride + W + unit_j] - piOrg[H * uiStride + W + unit_j]) : diff;
			}
			piOrg += uiStride;
			piRef += uiStride;
		}
	}
}

Void Picture::CompBlockPicRecoDiff(UInt uiW, UInt uiH, Picture* pcPic, Picture* PicYuvOrg, double& diff)
{

	Pel* piOrg;
	Pel* BgOrg;
	UInt uiStride;
	UInt uiPicHeight;
	UInt uiPicWidth;
	UInt unit_len;
	UInt H, W;
	for (Int compId = 0; compId < 3; compId++)
	{
		switch (compId)
		{
		case 0:
			piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);
			BgOrg = PicYuvOrg->getRecoBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Y().stride;
			uiPicHeight = pcPic->getRecoBuf().Y().height;
			uiPicWidth = pcPic->getRecoBuf().Y().width;
			unit_len = BLOCK_GEN_LEN;
			H = uiH;
			W = uiW;
			break;
		case 1:
			piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);
			BgOrg = PicYuvOrg->getRecoBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cb().stride;
			uiPicHeight = pcPic->getRecoBuf().Cb().height;
			uiPicWidth = pcPic->getRecoBuf().Cb().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		case 2:
			piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);
			BgOrg = PicYuvOrg->getRecoBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiPicHeight = pcPic->getRecoBuf().Cr().height;
			uiPicWidth = pcPic->getRecoBuf().Cr().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		default:
			break;
		}

		for (Int unit_i = 0; unit_i < unit_len && (H + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (W + unit_j < uiPicWidth); unit_j++)
			{
				diff = abs(BgOrg[H * uiStride + W + unit_j] - piOrg[H * uiStride + W + unit_j]) > diff ?
					abs(BgOrg[H * uiStride + W + unit_j] - piOrg[H * uiStride + W + unit_j]) : diff;
			}
			piOrg += uiStride;
			BgOrg += uiStride;
		}
	}
}
Bool Picture::CompBlockRecoIsSimilar(UInt uiW, UInt uiH, Picture* pcPic, Picture* PicYuvOrg)
{

	Pel* piOrg;
	Pel* BgOrg;
	UInt uiStride;
	UInt uiPicHeight;
	UInt uiPicWidth;
	UInt unit_len;
	UInt H, W;
	Int Sum = 0;
	Int Zero = 0;
	Int SetZero = 0;
	for (Int compId = 0; compId < 3; compId++)
	{
		
		switch (compId)
		{
		case 0:
			piOrg = pcPic->getRecoBuf().Y().bufAt(0, 0);
			BgOrg = PicYuvOrg->getRecoBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getRecoBuf().Y().stride;
			uiPicHeight = pcPic->getRecoBuf().Y().height;
			uiPicWidth = pcPic->getRecoBuf().Y().width;
			unit_len = BLOCK_GEN_LEN;
			H = uiH;
			W = uiW;
			break;
		case 1:
			piOrg = pcPic->getRecoBuf().Cb().bufAt(0, 0);
			BgOrg = PicYuvOrg->getRecoBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cb().stride;
			uiPicHeight = pcPic->getRecoBuf().Cb().height;
			uiPicWidth = pcPic->getRecoBuf().Cb().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		case 2:
			piOrg = pcPic->getRecoBuf().Cr().bufAt(0, 0);
			BgOrg = PicYuvOrg->getRecoBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getRecoBuf().Cr().stride;
			uiPicHeight = pcPic->getRecoBuf().Cr().height;
			uiPicWidth = pcPic->getRecoBuf().Cr().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		default:
			break;
		}

		for (Int unit_i = 0; unit_i < unit_len && (H + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (W + unit_j < uiPicWidth); unit_j++)
			{
				if(BgOrg[H * uiStride + W + unit_j] == piOrg[H * uiStride + W + unit_j])
					Zero++;
				Sum++;
			}
			piOrg += uiStride;
			BgOrg += uiStride;
		}
	}
	cout << "Zero" << Zero<<"SuM"<<Sum;
	if (Zero>=900)
		return true;
	return false;
}
Bool Picture::CompBlockOrgIsSimilar(UInt uiW, UInt uiH, Picture* pcPic, Picture* PicYuvOrg)
{

	Pel* piOrg;
	Pel* BgOrg;
	UInt uiStride;
	UInt uiPicHeight;
	UInt uiPicWidth;
	UInt unit_len;
	UInt H, W;
	for (Int compId = 0; compId < 3; compId++)
	{
		Int Sum = 0;
		Int Zero = 0;
		switch (compId)
		{
		case 0:
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
			BgOrg = PicYuvOrg->getOrigBuf().Y().bufAt(0, 0);

			uiStride = pcPic->getOrigBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			unit_len = BLOCK_GEN_LEN;
			H = uiH;
			W = uiW;
			break;
		case 1:
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);
			BgOrg = PicYuvOrg->getOrigBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		case 2:
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			BgOrg = PicYuvOrg->getOrigBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		default:
			break;
		}

		for (Int unit_i = 0; unit_i < unit_len && (H + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (W + unit_j < uiPicWidth); unit_j++)
			{
				if (abs(BgOrg[H * uiStride + W + unit_j] - piOrg[H * uiStride + W + unit_j]) >= 50)
				{
					Zero++;
				}
				Sum++;
			}
			piOrg += uiStride;
			BgOrg += uiStride;
		}
		if (Zero / Sum > 0.5)
			return true;
	}
	return false;
}
Bool Picture::CompBlockOrgIsFull(UInt uiW, UInt uiH, Picture* pcPic)
{

	Pel* piOrg;
	UInt uiStride;
	UInt uiPicHeight;
	UInt uiPicWidth;
	UInt unit_len;
	UInt H, W;
	for (Int compId = 0; compId < 3; compId++)
	{
		switch (compId)
		{
		case 0:
			piOrg = pcPic->getOrigBuf().Y().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Y().stride;
			uiPicHeight = pcPic->getOrigBuf().Y().height;
			uiPicWidth = pcPic->getOrigBuf().Y().width;
			unit_len = BLOCK_GEN_LEN;
			H = uiH;
			W = uiW;
			break;
		case 1:
			piOrg = pcPic->getOrigBuf().Cb().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cb().stride;
			uiPicHeight = pcPic->getOrigBuf().Cb().height;
			uiPicWidth = pcPic->getOrigBuf().Cb().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		case 2:
			piOrg = pcPic->getOrigBuf().Cr().bufAt(0, 0);
			uiStride = pcPic->getOrigBuf().Cr().stride;
			uiPicHeight = pcPic->getOrigBuf().Cr().height;
			uiPicWidth = pcPic->getOrigBuf().Cr().width;
			unit_len = BLOCK_GEN_LEN >> 1;
			H = uiH >> 1;
			W = uiW >> 1;
			break;
		default:
			break;
		}

		for (Int unit_i = 0; unit_i < unit_len && (H + unit_i < uiPicHeight); unit_i++)
		{
			for (Int unit_j = 0; unit_j < unit_len && (W + unit_j < uiPicWidth); unit_j++)
			{
				if (piOrg[H * uiStride + W + unit_j] == 0)
				{
					return false;
				}
			}
			piOrg += uiStride;
		}
	}
	return true;
}
#endif