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

/** \file     UnitTool.cpp
 *  \brief    defines operations for basic units
 */

#include "UnitTools.h"

#include "dtrace_next.h"

#include "Unit.h"
#include "Slice.h"
#include "Picture.h"

#include <utility>
#include <algorithm>

// CS tools


UInt64 CS::getEstBits(const CodingStructure &cs)
{
  return cs.fracBits >> SCALE_BITS;
}



bool CS::isDualITree( const CodingStructure &cs )
{
  return cs.slice->isIntra() && !cs.pcv->ISingleTree;
}

UnitArea CS::getArea( const CodingStructure &cs, const UnitArea &area, const ChannelType chType )
{
  return isDualITree( cs ) ? area.singleChan( chType ) : area;
}

// CU tools

bool CU::isIntra(const CodingUnit &cu)
{
  return cu.predMode == MODE_INTRA;
}

bool CU::isInter(const CodingUnit &cu)
{
  return cu.predMode == MODE_INTER;
}

bool CU::isRDPCMEnabled(const CodingUnit& cu)
{
  return cu.cs->sps->getSpsRangeExtension().getRdpcmEnabledFlag(cu.predMode == MODE_INTRA ? RDPCM_SIGNAL_IMPLICIT : RDPCM_SIGNAL_EXPLICIT);
}

#if HEVC_USE_RQT
UInt CU::getQuadtreeTULog2MinSizeInCU(const CodingUnit& cu)
{
  const SPS &sps = *cu.cs->sps;

  const UInt log2CbSize         = g_aucLog2[cu.lumaSize().width];
  const UInt quadtreeTUMaxDepth =  isIntra(cu) ? sps.getQuadtreeTUMaxDepthIntra() : sps.getQuadtreeTUMaxDepthInter();
#if HEVC_USE_PART_SIZE
  const Int intraSplitFlag      = (isIntra(cu) && cu.partSize == SIZE_NxN) ? 1 : 0;
#else
  const Int intraSplitFlag      = 0;
#endif
  const Int interSplitFlag      = ((quadtreeTUMaxDepth == 1) && isInter(cu) && (cu.partSize != SIZE_2Nx2N));

  UInt log2MinTUSizeInCU = 0;

  if (log2CbSize < (sps.getQuadtreeTULog2MinSize() + quadtreeTUMaxDepth - 1 + interSplitFlag + intraSplitFlag))
  {
    // when fully making use of signaled TUMaxDepth + inter/intraSplitFlag, resulting luma TB size is < QuadtreeTULog2MinSize
    log2MinTUSizeInCU = sps.getQuadtreeTULog2MinSize();
  }
  else
  {
    // when fully making use of signaled TUMaxDepth + inter/intraSplitFlag, resulting luma TB size is still >= QuadtreeTULog2MinSize
    log2MinTUSizeInCU = log2CbSize - (quadtreeTUMaxDepth - 1 + interSplitFlag + intraSplitFlag); // stop when trafoDepth == hierarchy_depth = splitFlag

    if (log2MinTUSizeInCU > sps.getQuadtreeTULog2MaxSize())
    {
      // when fully making use of signaled TUMaxDepth + inter/intraSplitFlag, resulting luma TB size is still > QuadtreeTULog2MaxSize
      log2MinTUSizeInCU = sps.getQuadtreeTULog2MaxSize();
    }
  }
  return log2MinTUSizeInCU;
}

#endif
bool CU::isLosslessCoded(const CodingUnit &cu)
{
  return cu.cs->pps->getTransquantBypassEnabledFlag() && cu.transQuantBypass;
}

bool CU::isSameSlice(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.slice->getIndependentSliceIdx() == cu2.slice->getIndependentSliceIdx();
}

#if HEVC_TILES_WPP
bool CU::isSameTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.tileIdx == cu2.tileIdx;
}

bool CU::isSameSliceAndTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return ( cu.slice->getIndependentSliceIdx() == cu2.slice->getIndependentSliceIdx() ) && ( cu.tileIdx == cu2.tileIdx );
}
#endif

bool CU::isSameCtu(const CodingUnit& cu, const CodingUnit& cu2)
{
  UInt ctuSizeBit = g_aucLog2[cu.cs->sps->getMaxCUWidth()];

  Position pos1Ctu(cu.lumaPos().x  >> ctuSizeBit, cu.lumaPos().y  >> ctuSizeBit);
  Position pos2Ctu(cu2.lumaPos().x >> ctuSizeBit, cu2.lumaPos().y >> ctuSizeBit);

  return pos1Ctu.x == pos2Ctu.x && pos1Ctu.y == pos2Ctu.y;
}

UInt CU::getIntraSizeIdx(const CodingUnit &cu)
{
#if HEVC_USE_PART_SIZE
  UInt  uiShift = ( cu.partSize == SIZE_NxN ? 1 : 0 );
  UChar uiWidth = cu.lumaSize().width >> uiShift;
#else
  UChar uiWidth = cu.lumaSize().width;
#endif

  UInt  uiCnt   = 0;

  while (uiWidth)
  {
    uiCnt++;
    uiWidth >>= 1;
  }

  uiCnt -= 2;

  return uiCnt > 6 ? 6 : uiCnt;
}

bool CU::isLastSubCUOfCtu( const CodingUnit &cu )
{
  const SPS &sps      = *cu.cs->sps;
  const Area cuAreaY = CS::isDualITree( *cu.cs ) ? Area( recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() ), recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() ) ) : ( const Area& ) cu.Y();

  return ( ( ( ( cuAreaY.x + cuAreaY.width  ) & cu.cs->pcv->maxCUWidthMask  ) == 0 || cuAreaY.x + cuAreaY.width  == sps.getPicWidthInLumaSamples()  ) &&
           ( ( ( cuAreaY.y + cuAreaY.height ) & cu.cs->pcv->maxCUHeightMask ) == 0 || cuAreaY.y + cuAreaY.height == sps.getPicHeightInLumaSamples() ) );
}

UInt CU::getCtuAddr( const CodingUnit &cu )
{
  return getCtuAddr( cu.blocks[cu.chType].lumaPos(), *cu.cs->pcv );
}

int CU::predictQP( const CodingUnit& cu, const int prevQP )
{
  const CodingStructure &cs = *cu.cs;

#if ENABLE_WPP_PARALLELISM
  if( cs.sps->getSpsNext().getUseNextDQP() )
  {
    // Inter-CTU 2D "planar"   c(orner)  a(bove)
    // predictor arrangement:  b(efore)  p(rediction)

    // restrict the lookup, as it might cross CTU/slice/tile boundaries
    const CodingUnit *cuA = cs.getCURestricted( cu.blocks[cu.chType].pos().offset(  0, -1 ), cu, cu.chType );
    const CodingUnit *cuB = cs.getCURestricted( cu.blocks[cu.chType].pos().offset( -1,  0 ), cu, cu.chType );
    const CodingUnit *cuC = cs.getCURestricted( cu.blocks[cu.chType].pos().offset( -1, -1 ), cu, cu.chType );

    const int a = cuA ? cuA->qp : cs.slice->getSliceQpBase();
    const int b = cuB ? cuB->qp : cs.slice->getSliceQpBase();
    const int c = cuC ? cuC->qp : cs.slice->getSliceQpBase();

    return Clip3( ( a < b ? a : b ), ( a > b ? a : b ), a + b - c ); // derived from Martucci's Median Adaptive Prediction, 1990
  }

#endif
  // only predict within the same CTU, use HEVC's above+left prediction
  const int a = ( cu.blocks[cu.chType].y & ( cs.pcv->maxCUHeightMask >> getChannelTypeScaleY( cu.chType, cu.chromaFormat ) ) ) ? ( cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType ) )->qp : prevQP;
  const int b = ( cu.blocks[cu.chType].x & ( cs.pcv->maxCUWidthMask  >> getChannelTypeScaleX( cu.chType, cu.chromaFormat ) ) ) ? ( cs.getCU( cu.blocks[cu.chType].pos().offset( -1, 0 ), cu.chType ) )->qp : prevQP;

  return ( a + b + 1 ) >> 1;
}

bool CU::isQGStart( const CodingUnit& cu )
{
  const SPS &sps = *cu.cs->sps;
  const PPS &pps = *cu.cs->pps;

  return ( cu.blocks[cu.chType].x % ( ( 1 << ( g_aucLog2[sps.getMaxCUWidth()]  - pps.getMaxCuDQPDepth() ) ) >> getChannelTypeScaleX( cu.chType, cu.chromaFormat ) ) ) == 0 &&
         ( cu.blocks[cu.chType].y % ( ( 1 << ( g_aucLog2[sps.getMaxCUHeight()] - pps.getMaxCuDQPDepth() ) ) >> getChannelTypeScaleY( cu.chType, cu.chromaFormat ) ) ) == 0;
}

UInt CU::getNumPUs( const CodingUnit& cu )
{
  UInt cnt = 0;
  PredictionUnit *pu = cu.firstPU;

  do
  {
    cnt++;
  } while( ( pu != cu.lastPU ) && ( pu = pu->next ) );

  return cnt;
}

void CU::addPUs( CodingUnit& cu )
{
#if HEVC_USE_PART_SIZE
  const auto puAreas = PartitionerImpl::getPUPartitioning( cu );

  for( const auto &puArea : puAreas )
  {
    cu.cs->addPU( CS::getArea( *cu.cs, puArea, cu.chType ), cu.chType );
  }
#else
  cu.cs->addPU( CS::getArea( *cu.cs, cu, cu.chType ), cu.chType );
#endif
}


PartSplit CU::getSplitAtDepth( const CodingUnit& cu, const unsigned depth )
{
  if( depth >= cu.depth ) return CU_DONT_SPLIT;

  const PartSplit cuSplitType = PartSplit( ( cu.splitSeries >> ( depth * SPLIT_DMULT ) ) & SPLIT_MASK );

  if     ( cuSplitType == CU_QUAD_SPLIT    ) return CU_QUAD_SPLIT;

  else if( cuSplitType == CU_HORZ_SPLIT    ) return CU_HORZ_SPLIT;

  else if( cuSplitType == CU_VERT_SPLIT    ) return CU_VERT_SPLIT;

  else if( cuSplitType == CU_TRIH_SPLIT    ) return CU_TRIH_SPLIT;
  else if( cuSplitType == CU_TRIV_SPLIT    ) return CU_TRIV_SPLIT;
  else   { THROW( "Unknown split mode"    ); return CU_QUAD_SPLIT; }
}

bool CU::hasNonTsCodedBlock( const CodingUnit& cu )
{
  bool hasAnyNonTSCoded = false;

  for( auto &currTU : traverseTUs( cu ) )
  {
    for( UInt i = 0; i < ::getNumberValidTBlocks( *cu.cs->pcv ); i++ )
    {
      hasAnyNonTSCoded |= ( currTU.blocks[i].valid() && !currTU.transformSkip[i] && TU::getCbf( currTU, ComponentID( i ) ) );
    }
  }

  return hasAnyNonTSCoded;
}

UInt CU::getNumNonZeroCoeffNonTs( const CodingUnit& cu )
{
  UInt count = 0;
  for( auto &currTU : traverseTUs( cu ) )
  {
    count += TU::getNumNonZeroCoeffsNonTS( currTU );
  }

  return count;
}




PUTraverser CU::traversePUs( CodingUnit& cu )
{
  return PUTraverser( cu.firstPU, cu.lastPU->next );
}

TUTraverser CU::traverseTUs( CodingUnit& cu )
{
  return TUTraverser( cu.firstTU, cu.lastTU->next );
}

cPUTraverser CU::traversePUs( const CodingUnit& cu )
{
  return cPUTraverser( cu.firstPU, cu.lastPU->next );
}

cTUTraverser CU::traverseTUs( const CodingUnit& cu )
{
  return cTUTraverser( cu.firstTU, cu.lastTU->next );
}

// PU tools

int PU::getIntraMPMs( const PredictionUnit &pu, unsigned* mpm, const ChannelType &channelType /*= CHANNEL_TYPE_LUMA*/ )
{
  const unsigned numMPMs = pu.cs->pcv->numMPMs;
  {
    Int numCand = -1;
    Int leftIntraDir = DC_IDX, aboveIntraDir = DC_IDX;

    const CompArea& area = pu.block( getFirstComponentOfChannel( channelType ) );
    const Position& pos  = area.pos();

    // Get intra direction of left PU
    const PredictionUnit *puLeft = pu.cs->getPURestricted( pos.offset( -1, 0 ), pu, channelType );

    if( puLeft && CU::isIntra( *puLeft->cu ) )
    {
      leftIntraDir = puLeft->intraDir[channelType];

      if( isChroma( channelType ) && leftIntraDir == DM_CHROMA_IDX )
      {
        leftIntraDir = puLeft->intraDir[0];
      }
    }

    // Get intra direction of above PU
    const PredictionUnit* puAbove = pu.cs->getPURestricted( pos.offset( 0, -1 ), pu, channelType );

    if( puAbove && CU::isIntra( *puAbove->cu ) && CU::isSameCtu( *pu.cu, *puAbove->cu ) )
    {
      aboveIntraDir = puAbove->intraDir[channelType];

      if( isChroma( channelType ) && aboveIntraDir == DM_CHROMA_IDX )
      {
        aboveIntraDir = puAbove->intraDir[0];
      }
    }

    CHECK( 2 >= numMPMs, "Invalid number of most probable modes" );

    const Int offset = 29;

    const Int mod    = offset + 3;

    if( leftIntraDir == aboveIntraDir )
    {
      numCand = 1;

      if( leftIntraDir > DC_IDX ) // angular modes
      {
        mpm[0] =   g_intraMode65to33AngMapping[leftIntraDir];
        mpm[1] = ((g_intraMode65to33AngMapping[leftIntraDir] + offset) % mod) + 2;
        mpm[2] = ((g_intraMode65to33AngMapping[leftIntraDir] - 1)      % mod) + 2;
      }
      else //non-angular
      {
        mpm[0] = g_intraMode65to33AngMapping[PLANAR_IDX];
        mpm[1] = g_intraMode65to33AngMapping[DC_IDX];
        mpm[2] = g_intraMode65to33AngMapping[VER_IDX];
      }
    }
    else
    {
      numCand = 2;

      mpm[0] = g_intraMode65to33AngMapping[leftIntraDir];
      mpm[1] = g_intraMode65to33AngMapping[aboveIntraDir];

      if( leftIntraDir && aboveIntraDir ) //both modes are non-planar
      {
        mpm[2] = g_intraMode65to33AngMapping[PLANAR_IDX];
      }
      else
      {
        mpm[2] = g_intraMode65to33AngMapping[(leftIntraDir + aboveIntraDir) < 2 ? VER_IDX : DC_IDX];
      }
    }
    for( UInt i = 0; i < numMPMs; i++ )
    {
      mpm[i] = g_intraMode33to65AngMapping[mpm[i]];
      CHECK( mpm[i] >= NUM_LUMA_MODE, "Invalid MPM" );
    }
    CHECK( numCand == 0, "No candidates found" );
    return numCand;
  }
}

void PU::getIntraChromaCandModes( const PredictionUnit &pu, unsigned modeList[NUM_CHROMA_MODE] )
{
  {
    modeList[  0 ] = PLANAR_IDX;
    modeList[  1 ] = VER_IDX;
    modeList[  2 ] = HOR_IDX;
    modeList[  3 ] = DC_IDX;
    modeList[  4 ] = DM_CHROMA_IDX;

    const PredictionUnit *lumaPU = CS::isDualITree( *pu.cs ) ? pu.cs->picture->cs->getPU( pu.blocks[pu.chType].lumaPos(), CHANNEL_TYPE_LUMA ) : &pu;
    const UInt lumaMode = lumaPU->intraDir[CHANNEL_TYPE_LUMA];
    for( Int i = 0; i < 4; i++ )
    {
      if( lumaMode == modeList[i] )
      {
        modeList[i] = VDIA_IDX;
        break;
      }
    }
  }
}


bool PU::isChromaIntraModeCrossCheckMode( const PredictionUnit &pu )
{
  return pu.intraDir[CHANNEL_TYPE_CHROMA] == DM_CHROMA_IDX;
}

UInt PU::getFinalIntraMode( const PredictionUnit &pu, const ChannelType &chType )
{
  UInt uiIntraMode = pu.intraDir[chType];

  if( uiIntraMode == DM_CHROMA_IDX && !isLuma( chType ) )
  {
    const PredictionUnit &lumaPU = CS::isDualITree( *pu.cs ) ? *pu.cs->picture->cs->getPU( pu.blocks[chType].lumaPos(), CHANNEL_TYPE_LUMA ) : *pu.cs->getPU( pu.blocks[chType].lumaPos(), CHANNEL_TYPE_LUMA );
    uiIntraMode = lumaPU.intraDir[0];
  }
  if( pu.chromaFormat == CHROMA_422 && !isLuma( chType ) )
  {
    uiIntraMode = g_chroma422IntraAngleMappingTable[uiIntraMode];
  }
  return uiIntraMode;
}


void PU::getInterMergeCandidates( const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx )
{
  const CodingStructure &cs  = *pu.cs;
  const Slice &slice         = *pu.cs->slice;
  const UInt maxNumMergeCand = slice.getMaxNumMergeCand();
  const bool canFastExit     = pu.cs->pps->getLog2ParallelMergeLevelMinus2() == 0;

  Bool isCandInter[MRG_MAX_NUM_CANDS];

  for (UInt ui = 0; ui < maxNumMergeCand; ++ui)
  {
    isCandInter[ui] = false;
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mrgTypeNeighbours [ui] = MRG_TYPE_DEFAULT_N;
    mrgCtx.mvFieldNeighbours[(ui << 1)    ].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  // compute the location of the current PU

  Int cnt = 0;

  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();

  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  //left
  const PredictionUnit* puLeft = cs.getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );

#if HEVC_USE_PART_SIZE
  const Bool isAvailableA1 = puLeft && isDiffMER( pu, *puLeft ) && ( pu.cu != puLeft->cu || pu.cu->partSize == SIZE_NxN ) && CU::isInter( *puLeft->cu );
#else
  const Bool isAvailableA1 = puLeft && isDiffMER( pu, *puLeft ) && pu.cu != puLeft->cu && CU::isInter( *puLeft->cu );
#endif

  if( isAvailableA1 )
  {
    miLeft = puLeft->getMotionInfo( posLB.offset(-1, 0) );

    isCandInter[cnt] = true;

    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;

    // get Mv from Left
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);

    if (slice.isInterB())
    {
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miLeft.mv[1], miLeft.refIdx[1]);
    }

    if( mrgCandIdx == cnt && canFastExit )
    {
      return;
    }

    cnt++;
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }


  // above
  const PredictionUnit *puAbove = cs.getPURestricted( posRT.offset( 0, -1 ), pu, pu.chType );

#if HEVC_USE_PART_SIZE
  Bool isAvailableB1 = puAbove && isDiffMER( pu, *puAbove ) && ( pu.cu != puAbove->cu || pu.cu->partSize == SIZE_NxN ) && CU::isInter( *puAbove->cu );
#else
  Bool isAvailableB1 = puAbove && isDiffMER( pu, *puAbove ) && pu.cu != puAbove->cu && CU::isInter( *puAbove->cu );
#endif

  if( isAvailableB1 )
  {
    miAbove = puAbove->getMotionInfo( posRT.offset( 0, -1 ) );

    if( !isAvailableA1 || ( miAbove != miLeft ) )
    {
      isCandInter[cnt] = true;

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
      // get Mv from Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAbove.mv[0], miAbove.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAbove.mv[1], miAbove.refIdx[1] );
      }

      if( mrgCandIdx == cnt && canFastExit )
      {
        return;
      }

      cnt++;
    }
  }

  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }

  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );

  Bool isAvailableB0 = puAboveRight && isDiffMER( pu, *puAboveRight ) && CU::isInter( *puAboveRight->cu );

  if( isAvailableB0 )
  {
    miAboveRight = puAboveRight->getMotionInfo( posRT.offset( 1, -1 ) );

#if HM_JEM_MERGE_CANDS
    if( ( !isAvailableB1 || ( miAbove != miAboveRight ) ) && ( !isAvailableA1 || ( miLeft != miAboveRight ) ) )
#else
    if( !isAvailableB1 || ( miAbove != miAboveRight ) )
#endif
    {
      isCandInter[cnt] = true;

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;
      // get Mv from Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveRight.mv[0], miAboveRight.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveRight.mv[1], miAboveRight.refIdx[1] );
      }

      if( mrgCandIdx == cnt && canFastExit )
      {
        return;
      }

      cnt++;
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );

  Bool isAvailableA0 = puLeftBottom && isDiffMER( pu, *puLeftBottom ) && CU::isInter( *puLeftBottom->cu );

  if( isAvailableA0 )
  {
    miBelowLeft = puLeftBottom->getMotionInfo( posLB.offset( -1, 1 ) );

#if HM_JEM_MERGE_CANDS
    if( ( !isAvailableA1 || ( miBelowLeft != miLeft ) ) && ( !isAvailableB1 || ( miBelowLeft != miAbove ) ) && ( !isAvailableB0 || ( miBelowLeft != miAboveRight ) ) )
#else
    if( !isAvailableA1 || ( miBelowLeft != miLeft ) )
#endif
    {
      isCandInter[cnt] = true;

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
      // get Mv from Bottom-Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miBelowLeft.mv[0], miBelowLeft.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miBelowLeft.mv[1], miBelowLeft.refIdx[1] );
      }

      if( mrgCandIdx == cnt && canFastExit )
      {
        return;
      }

      cnt++;
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }


  // above left
  if( cnt < 4 )
  {
    const PredictionUnit *puAboveLeft = cs.getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );

    Bool isAvailableB2 = puAboveLeft && isDiffMER( pu, *puAboveLeft ) && CU::isInter( *puAboveLeft->cu );

    if( isAvailableB2 )
    {
      miAboveLeft = puAboveLeft->getMotionInfo( posLT.offset( -1, -1 ) );

#if HM_JEM_MERGE_CANDS
      if( ( !isAvailableA1 || ( miLeft != miAboveLeft ) ) && ( !isAvailableB1 || ( miAbove != miAboveLeft ) ) && ( !isAvailableA0 || ( miBelowLeft != miAboveLeft ) ) && ( !isAvailableB0 || ( miAboveRight != miAboveLeft ) ) )
#else
      if( ( !isAvailableA1 || ( miLeft != miAboveLeft ) ) && ( !isAvailableB1 || ( miAbove != miAboveLeft ) ) )
#endif
      {
        isCandInter[cnt] = true;

        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveLeft.mv[0], miAboveLeft.refIdx[0] );

        if( slice.isInterB() )
        {
          mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveLeft.mv[1], miAboveLeft.refIdx[1] );
        }

        if( mrgCandIdx == cnt && canFastExit )
        {
          return;
        }

        cnt++;
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  if (slice.getEnableTMVPFlag())
  {
    //>> MTK colocated-RightBottom
    // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
    Position posRB = pu.Y().bottomRight().offset(-3, -3);

    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    Position posC1 = pu.Y().center();
    Bool C0Avail = false;

    if (((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight))
    {
      {
        Position posInCtu( posRB.x & pcv.maxCUWidthMask, posRB.y & pcv.maxCUHeightMask );

        if( ( posInCtu.x + 4 < pcv.maxCUWidth ) &&           // is not at the last column of CTU
            ( posInCtu.y + 4 < pcv.maxCUHeight ) )           // is not at the last row    of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          C0Avail = true;
        }
        else if( posInCtu.x + 4 < pcv.maxCUWidth )           // is not at the last column of CTU But is last row of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          // in the reference the CTU address is not set - thus probably resulting in no using this C0 possibility
        }
        else if( posInCtu.y + 4 < pcv.maxCUHeight )          // is not at the last row of CTU But is last column of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          C0Avail = true;
        }
        else //is the right bottom corner of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          // same as for last column but not last row
        }
      }
    }

    Mv        cColMv;
    int       iRefIdx     = 0;
    int       dir         = 0;
    unsigned  uiArrayAddr = cnt;
    bool      bExistMV    = ( C0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, cColMv, iRefIdx ) )
                                      || getColocatedMVP(pu, REF_PIC_LIST_0, posC1, cColMv, iRefIdx );

    if (bExistMV)
    {
      dir     |= 1;
      mrgCtx.mvFieldNeighbours[2 * uiArrayAddr].setMvField(cColMv, iRefIdx);
    }

    if (slice.isInterB())
    {
      bExistMV = ( C0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, cColMv, iRefIdx ) )
                           || getColocatedMVP(pu, REF_PIC_LIST_1, posC1, cColMv, iRefIdx );
      if (bExistMV)
      {
        dir     |= 2;
        mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 1].setMvField(cColMv, iRefIdx);
      }
    }

    if( dir != 0 )
    {
      bool addTMvp = true;
#if HM_JEM_MERGE_CANDS
      int iSpanCand = cnt;
      for (int i = 0; i < iSpanCand; i++)
      {
        if (mrgCtx.interDirNeighbours[i] == dir &&
          mrgCtx.mvFieldNeighbours[i << 1] == mrgCtx.mvFieldNeighbours[uiArrayAddr << 1] &&
          mrgCtx.mvFieldNeighbours[(i << 1) + 1] == mrgCtx.mvFieldNeighbours[(uiArrayAddr << 1) + 1])
        {
          addTMvp = false;
        }
      }
#endif
      if (addTMvp)
      {
        mrgCtx.interDirNeighbours[uiArrayAddr] = dir;
        isCandInter              [uiArrayAddr] = true;

        if( mrgCandIdx == cnt && canFastExit )
        {
          return;
        }

        cnt++;
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  UInt uiArrayAddr = cnt;
  UInt uiCutoff    = std::min( uiArrayAddr, 4u );

  if (slice.isInterB())
  {
    static const UInt NUM_PRIORITY_LIST = 12;
    static const UInt uiPriorityList0[NUM_PRIORITY_LIST] = { 0 , 1, 0, 2, 1, 2, 0, 3, 1, 3, 2, 3 };
    static const UInt uiPriorityList1[NUM_PRIORITY_LIST] = { 1 , 0, 2, 0, 2, 1, 3, 0, 3, 1, 3, 2 };

    for (Int idx = 0; idx < uiCutoff * (uiCutoff - 1) && uiArrayAddr != maxNumMergeCand; idx++)
    {
      CHECK( idx >= NUM_PRIORITY_LIST, "Invalid priority list number" );
      Int i = uiPriorityList0[idx];
      Int j = uiPriorityList1[idx];
      if (isCandInter[i] && isCandInter[j] && (mrgCtx.interDirNeighbours[i] & 0x1) && (mrgCtx.interDirNeighbours[j] & 0x2))
      {
        isCandInter[uiArrayAddr] = true;
        mrgCtx.interDirNeighbours[uiArrayAddr] = 3;

        // get Mv from cand[i] and cand[j]
        mrgCtx.mvFieldNeighbours[ uiArrayAddr << 1     ].setMvField(mrgCtx.mvFieldNeighbours[ i << 1     ].mv, mrgCtx.mvFieldNeighbours[ i << 1     ].refIdx);
        mrgCtx.mvFieldNeighbours[(uiArrayAddr << 1) + 1].setMvField(mrgCtx.mvFieldNeighbours[(j << 1) + 1].mv, mrgCtx.mvFieldNeighbours[(j << 1) + 1].refIdx);

        Int iRefPOCL0 = slice.getRefPOC(REF_PIC_LIST_0, mrgCtx.mvFieldNeighbours[(uiArrayAddr << 1)    ].refIdx);
        Int iRefPOCL1 = slice.getRefPOC(REF_PIC_LIST_1, mrgCtx.mvFieldNeighbours[(uiArrayAddr << 1) + 1].refIdx);

        if( iRefPOCL0 == iRefPOCL1 && mrgCtx.mvFieldNeighbours[( uiArrayAddr << 1 )].mv == mrgCtx.mvFieldNeighbours[( uiArrayAddr << 1 ) + 1].mv )
        {
          isCandInter[uiArrayAddr] = false;
        }
        else
        {
          uiArrayAddr++;
        }
      }
    }
  }

  // early termination
  if (uiArrayAddr == maxNumMergeCand)
  {
    return;
  }

  Int iNumRefIdx = slice.isInterB() ? std::min(slice.getNumRefIdx(REF_PIC_LIST_0), slice.getNumRefIdx(REF_PIC_LIST_1)) : slice.getNumRefIdx(REF_PIC_LIST_0);

  Int r = 0;
  Int refcnt = 0;
  while (uiArrayAddr < maxNumMergeCand)
  {
    isCandInter               [uiArrayAddr     ] = true;
    mrgCtx.interDirNeighbours [uiArrayAddr     ] = 1;
    mrgCtx.mvFieldNeighbours  [uiArrayAddr << 1].setMvField(Mv(0, 0), r);

    if (slice.isInterB())
    {
      mrgCtx.interDirNeighbours [ uiArrayAddr          ] = 3;
      mrgCtx.mvFieldNeighbours  [(uiArrayAddr << 1) + 1].setMvField(Mv(0, 0), r);
    }

    uiArrayAddr++;

    if (refcnt == iNumRefIdx - 1)
    {
      r = 0;
    }
    else
    {
      ++r;
      ++refcnt;
    }
  }
  mrgCtx.numValidMergeCand = uiArrayAddr;
}



static int xGetDistScaleFactor(const int &iCurrPOC, const int &iCurrRefPOC, const int &iColPOC, const int &iColRefPOC)
{
  int iDiffPocD = iColPOC - iColRefPOC;
  int iDiffPocB = iCurrPOC - iCurrRefPOC;

  if (iDiffPocD == iDiffPocB)
  {
    return 4096;
  }
  else
  {
    int iTDB = Clip3(-128, 127, iDiffPocB);
    int iTDD = Clip3(-128, 127, iDiffPocD);
    int iX = (0x4000 + abs(iTDD / 2)) / iTDD;
    int iScale = Clip3(-4096, 4095, (iTDB * iX + 32) >> 6);
    return iScale;
  }
}

bool PU::getColocatedMVP(const PredictionUnit &pu, const RefPicList &eRefPicList, const Position &_pos, Mv& rcMv, const int &refIdx )
{
  // don't perform MV compression when generally disabled or subPuMvp is used
  const unsigned scale = ( pu.cs->pcv->noMotComp ? 1 : 4 * std::max<Int>(1, 4 * AMVP_DECIMATION_FACTOR / 4) );
  const unsigned mask  = ~( scale - 1 );

  const Position pos = Position{ PosType( _pos.x & mask ), PosType( _pos.y & mask ) };

  const Slice &slice = *pu.cs->slice;

  // use coldir.
  const Picture* const pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());

  if( !pColPic )
  {
    return false;
  }

  RefPicList eColRefPicList = slice.getCheckLDC() ? eRefPicList : RefPicList(slice.getColFromL0Flag());

  const MotionInfo& mi = pColPic->cs->getMotionInfo( pos );

  if( !mi.isInter )
  {
    return false;
  }

  int iColRefIdx = mi.refIdx[eColRefPicList];

  if (iColRefIdx < 0)
  {
    eColRefPicList = RefPicList(1 - eColRefPicList);
    iColRefIdx = mi.refIdx[eColRefPicList];

    if (iColRefIdx < 0)
    {
      return false;
    }
  }

  const Slice *pColSlice = nullptr;

  for( const auto s : pColPic->slices )
  {
    if( s->getIndependentSliceIdx() == mi.sliceIdx )
    {
      pColSlice = s;
      break;
    }
  }

  CHECK( pColSlice == nullptr, "Slice segment not found" );

  const Slice &colSlice = *pColSlice;

  const bool bIsCurrRefLongTerm = slice.getRefPic(eRefPicList, refIdx)->longTerm;
  const bool bIsColRefLongTerm  = colSlice.getIsUsedAsLongTerm(eColRefPicList, iColRefIdx);

  if (bIsCurrRefLongTerm != bIsColRefLongTerm)
  {
    return false;
  }


  // Scale the vector.
  Mv cColMv = mi.mv[eColRefPicList];

  if (bIsCurrRefLongTerm /*|| bIsColRefLongTerm*/)
  {
    rcMv = cColMv;
  }
  else
  {
    const Int currPOC    = slice.getPOC();
    const Int colPOC     = colSlice.getPOC();
    const Int colRefPOC  = colSlice.getRefPOC(eColRefPicList, iColRefIdx);
    const Int currRefPOC = slice.getRefPic(eRefPicList, refIdx)->getPOC();
    const Int distscale  = xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);

    if (distscale == 4096)
    {
      rcMv = cColMv;
    }
    else
    {
      rcMv = cColMv.scaleMv(distscale);
    }
  }

  return true;
}

bool PU::isDiffMER(const PredictionUnit &pu1, const PredictionUnit &pu2)
{
  const unsigned xN = pu1.lumaPos().x;
  const unsigned yN = pu1.lumaPos().y;
  const unsigned xP = pu2.lumaPos().x;
  const unsigned yP = pu2.lumaPos().y;

  unsigned plevel = pu1.cs->pps->getLog2ParallelMergeLevelMinus2() + 2;

  if ((xN >> plevel) != (xP >> plevel))
  {
    return true;
  }

  if ((yN >> plevel) != (yP >> plevel))
  {
    return true;
  }

  return false;
}

/** Constructs a list of candidates for AMVP (See specification, section "Derivation process for motion vector predictor candidates")
* \param uiPartIdx
* \param uiPartAddr
* \param eRefPicList
* \param iRefIdx
* \param pInfo
*/
void PU::fillMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo)
{
  CodingStructure &cs = *pu.cs;

  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;

  if (refIdx < 0)
  {
    return;
  }

  //-- Get Spatial MV
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

  Bool isScaledFlagLX = false; /// variable name from specification; true when the PUs below left or left are available (availableA0 || availableA1).

  {
    const PredictionUnit* tmpPU = cs.getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType ); // getPUBelowLeft(idx, partIdxLB);
    isScaledFlagLX = tmpPU != NULL && CU::isInter( *tmpPU->cu );

    if( !isScaledFlagLX )
    {
      tmpPU = cs.getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );
      isScaledFlagLX = tmpPU != NULL && CU::isInter( *tmpPU->cu );
    }
  }

  // Left predictor search
  if( isScaledFlagLX )
  {
    Bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, *pInfo );

      if( !bAdded )
      {
        bAdded = addMVPCandWithScaling( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, *pInfo );

        if( !bAdded )
        {
          addMVPCandWithScaling( pu, eRefPicList, refIdx, posLB, MD_LEFT, *pInfo );
        }
      }
    }
  }

  // Above predictor search
  {
    Bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, *pInfo );

      if( !bAdded )
      {
        addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, *pInfo );
      }
    }
  }

  if( !isScaledFlagLX )
  {
    Bool bAdded = addMVPCandWithScaling( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandWithScaling( pu, eRefPicList, refIdx, posRT, MD_ABOVE, *pInfo );

      if( !bAdded )
      {
        addMVPCandWithScaling( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, *pInfo );
      }
    }
  }


  if( pInfo->numCand == 2 )
  {
    if( pInfo->mvCand[0] == pInfo->mvCand[1] )
    {
      pInfo->numCand = 1;
    }
  }

  if( cs.slice->getEnableTMVPFlag() )
  {
    // Get Temporal Motion Predictor
    const int refIdx_Col = refIdx;

    Position posRB = pu.Y().bottomRight().offset(-3, -3);

    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    bool C0Avail = false;
    Position posC1 = pu.Y().center();

    Mv cColMv;

    if( ( ( posRB.x + pcv.minCUWidth ) < pcv.lumaWidth ) && ( ( posRB.y + pcv.minCUHeight ) < pcv.lumaHeight ) )
    {
      Position posInCtu( posRB.x & pcv.maxCUWidthMask, posRB.y & pcv.maxCUHeightMask );

      if ((posInCtu.x + 4 < pcv.maxCUWidth) &&           // is not at the last column of CTU
          (posInCtu.y + 4 < pcv.maxCUHeight))             // is not at the last row    of CTU
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
      else if (posInCtu.x + 4 < pcv.maxCUWidth)           // is not at the last column of CTU But is last row of CTU
      {
        // in the reference the CTU address is not set - thus probably resulting in no using this C0 possibility
        posC0 = posRB.offset(4, 4);
      }
      else if (posInCtu.y + 4 < pcv.maxCUHeight)          // is not at the last row of CTU But is last column of CTU
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
      else //is the right bottom corner of CTU
      {
        // same as for last column but not last row
        posC0 = posRB.offset(4, 4);
      }
    }

    if ((C0Avail && getColocatedMVP(pu, eRefPicList, posC0, cColMv, refIdx_Col)) || getColocatedMVP(pu, eRefPicList, posC1, cColMv, refIdx_Col))
    {
      pInfo->mvCand[pInfo->numCand++] = cColMv;
    }
  }
  if (pInfo->numCand > AMVP_MAX_NUM_CANDS)
  {
    pInfo->numCand = AMVP_MAX_NUM_CANDS;
  }

  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand] = Mv( 0, 0 );
    pInfo->numCand++;
  }

}


bool PU::addMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &info )
{
        CodingStructure &cs    = *pu.cs;
  const PredictionUnit *neibPU = NULL;
        Position neibPos;

  switch (eDir)
  {
  case MD_LEFT:
    neibPos = pos.offset( -1,  0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset(  0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset(  1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1,  1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

  if( neibPU == NULL || !CU::isInter( *neibPU->cu ) )
  {
    return false;
  }

  const MotionInfo& neibMi        = neibPU->getMotionInfo( neibPos );

  const Int        currRefPOC     = cs.slice->getRefPic( eRefPicList, iRefIdx )->getPOC();
  const RefPicList eRefPicList2nd = ( eRefPicList == REF_PIC_LIST_0 ) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for( Int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = ( predictorSource == 0 ) ? eRefPicList : eRefPicList2nd;
    const Int        neibRefIdx       = neibMi.refIdx[eRefPicListIndex];

    if( neibRefIdx >= 0 && currRefPOC == cs.slice->getRefPOC( eRefPicListIndex, neibRefIdx ) )
    {
      {
        info.mvCand[info.numCand++] = neibMi.mv[eRefPicListIndex];
        return true;
      }
    }
  }


  return false;
}

/**
* \param pInfo
* \param eRefPicList
* \param iRefIdx
* \param uiPartUnitIdx
* \param eDir
* \returns Bool
*/
bool PU::addMVPCandWithScaling( const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &info )
{
        CodingStructure &cs    = *pu.cs;
  const Slice &slice           = *cs.slice;
  const PredictionUnit *neibPU = NULL;
        Position neibPos;

  switch( eDir )
  {
  case MD_LEFT:
    neibPos = pos.offset( -1,  0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset(  0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset(  1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1,  1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

  if( neibPU == NULL || !CU::isInter( *neibPU->cu ) )
  {
    return false;
  }

  const MotionInfo& neibMi        = neibPU->getMotionInfo( neibPos );

  const RefPicList eRefPicList2nd = ( eRefPicList == REF_PIC_LIST_0 ) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  const int  currPOC            = slice.getPOC();
  const int  currRefPOC         = slice.getRefPic( eRefPicList, iRefIdx )->poc;
  const bool bIsCurrRefLongTerm = slice.getRefPic( eRefPicList, iRefIdx )->longTerm;
  const int  neibPOC            = currPOC;

  for( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = (predictorSource == 0) ? eRefPicList : eRefPicList2nd;
    const int        neibRefIdx       = neibMi.refIdx[eRefPicListIndex];
    if( neibRefIdx >= 0 )
    {
      const bool bIsNeibRefLongTerm = slice.getRefPic(eRefPicListIndex, neibRefIdx)->longTerm;

      if (bIsCurrRefLongTerm == bIsNeibRefLongTerm)
      {
        Mv cMv = neibMi.mv[eRefPicListIndex];

        if( !( bIsCurrRefLongTerm /* || bIsNeibRefLongTerm*/) )
        {
          const int neibRefPOC = slice.getRefPOC( eRefPicListIndex, neibRefIdx );
          const int scale      = xGetDistScaleFactor( currPOC, currRefPOC, neibPOC, neibRefPOC );

          if( scale != 4096 )
          {
            cMv = cMv.scaleMv( scale );
          }
        }

        {
          info.mvCand[info.numCand++] = cMv;
          return true;
        }
      }
    }
  }


  return false;
}

bool PU::isBipredRestriction(const PredictionUnit &pu)
{
  if( !pu.cs->pcv->only2Nx2N && pu.cu->lumaSize().width == 8 && ( pu.lumaSize().width < 8 || pu.lumaSize().height < 8 ) )
  {
    return true;
  }
  return false;
}


void PU::spanMotionInfo( PredictionUnit &pu, const MergeCtx &mrgCtx )
{
  MotionBuf mb = pu.getMotionBuf();

  if( !pu.mergeFlag || pu.mergeType == MRG_TYPE_DEFAULT_N )
  {
    MotionInfo mi;

    mi.isInter  = CU::isInter( *pu.cu );
    mi.sliceIdx = pu.cu->slice->getIndependentSliceIdx();

    if( mi.isInter )
    {
      mi.interDir = pu.interDir;

      for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
      {
        mi.mv[i]     = pu.mv[i];
        mi.refIdx[i] = pu.refIdx[i];
      }
    }

    {
      mb.fill( mi );
    }
  }
  else
  {

    if( isBipredRestriction( pu ) )
    {
      for( int y = 0; y < mb.height; y++ )
      {
        for( int x = 0; x < mb.width; x++ )
        {
          MotionInfo &mi = mb.at( x, y );
          if( mi.interDir == 3 )
          {
            mi.interDir  = 1;
            mi.mv    [1] = Mv();
            mi.refIdx[1] = NOT_VALID;
          }
        }
      }
    }
  }
}


bool PU::isBiPredFromDifferentDir( const PredictionUnit& pu )
{
  if ( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
  {
    const int iPOC0 = pu.cu->slice->getRefPOC( REF_PIC_LIST_0, pu.refIdx[0] );
    const int iPOC1 = pu.cu->slice->getRefPOC( REF_PIC_LIST_1, pu.refIdx[1] );
    const int iPOC  = pu.cu->slice->getPOC();
    if ( (iPOC - iPOC0)*(iPOC - iPOC1) < 0 )
    {
      return true;
    }
  }

  return false;
}

void PU::restrictBiPredMergeCands( const PredictionUnit &pu, MergeCtx& mergeCtx )
{
  if( PU::isBipredRestriction( pu ) )
  {
    for( UInt mergeCand = 0; mergeCand < mergeCtx.numValidMergeCand; ++mergeCand )
    {
      if( mergeCtx.interDirNeighbours[ mergeCand ] == 3 )
      {
        mergeCtx.interDirNeighbours[ mergeCand ] = 1;
        mergeCtx.mvFieldNeighbours[( mergeCand << 1 ) + 1].setMvField( Mv( 0, 0 ), -1 );
      }
    }
  }
}






// TU tools

#if HEVC_USE_4x4_DSTVII
bool TU::useDST(const TransformUnit &tu, const ComponentID &compID)
{
  return isLuma(compID) && tu.cu->predMode == MODE_INTRA;
}

#endif

bool TU::isNonTransformedResidualRotated(const TransformUnit &tu, const ComponentID &compID)
{
  return tu.cs->sps->getSpsRangeExtension().getTransformSkipRotationEnabledFlag() && tu.blocks[compID].width == 4 && tu.cu->predMode == MODE_INTRA;
}

bool TU::getCbf( const TransformUnit &tu, const ComponentID &compID )
{
#if HEVC_USE_RQT || ENABLE_BMS
  return getCbfAtDepth( tu, compID, tu.depth );
#else
  return tu.cbf[compID];
#endif
}

#if HEVC_USE_RQT || ENABLE_BMS
bool TU::getCbfAtDepth(const TransformUnit &tu, const ComponentID &compID, const unsigned &depth)
{
  return ((tu.cbf[compID] >> depth) & 1) == 1;
}

void TU::setCbfAtDepth(TransformUnit &tu, const ComponentID &compID, const unsigned &depth, const bool &cbf)
{
  // first clear the CBF at the depth
  tu.cbf[compID] &= ~(1  << depth);
  // then set the CBF
  tu.cbf[compID] |= ((cbf ? 1 : 0) << depth);
}
#else
void TU::setCbf( TransformUnit &tu, const ComponentID &compID, const bool &cbf )
{
  tu.cbf[compID] = cbf;
}
#endif

bool TU::hasTransformSkipFlag(const CodingStructure& cs, const CompArea& area)
{
  UInt transformSkipLog2MaxSize = cs.pps->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize();

  if( cs.pcv->rectCUs )
  {
    return ( area.width * area.height <= (1 << ( transformSkipLog2MaxSize << 1 )) );
  }
  return ( area.width <= (1 << transformSkipLog2MaxSize) );
}

UInt TU::getGolombRiceStatisticsIndex(const TransformUnit &tu, const ComponentID &compID)
{
  const Bool transformSkip    = tu.transformSkip[compID];
  const Bool transquantBypass = tu.cu->transQuantBypass;

  //--------

  const UInt channelTypeOffset = isChroma(compID) ? 2 : 0;
  const UInt nonTransformedOffset = (transformSkip || transquantBypass) ? 1 : 0;

  //--------

  const UInt selectedIndex = channelTypeOffset + nonTransformedOffset;
  CHECK( selectedIndex >= RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS, "Invalid golomb rice adaptation statistics set" );

  return selectedIndex;
}

#if HEVC_USE_MDCS
UInt TU::getCoefScanIdx(const TransformUnit &tu, const ComponentID &compID)
{
  //------------------------------------------------

  //this mechanism is available for intra only

  if( !CU::isIntra( *tu.cu ) )
  {
    return SCAN_DIAG;
  }

  //------------------------------------------------

  //check that MDCS can be used for this TU


  const CompArea &area      = tu.blocks[compID];
  const SPS &sps            = *tu.cs->sps;
  const ChromaFormat format = sps.getChromaFormatIdc();


  const UInt maximumWidth  = MDCS_MAXIMUM_WIDTH  >> getComponentScaleX(compID, format);
  const UInt maximumHeight = MDCS_MAXIMUM_HEIGHT >> getComponentScaleY(compID, format);

  if ((area.width > maximumWidth) || (area.height > maximumHeight))
  {
    return SCAN_DIAG;
  }

  //------------------------------------------------

  //otherwise, select the appropriate mode

  const PredictionUnit &pu = *tu.cs->getPU( area.pos(), toChannelType( compID ) );

  UInt uiDirMode = PU::getFinalIntraMode(pu, toChannelType(compID));

  //------------------

       if (abs((Int) uiDirMode - VER_IDX) <= MDCS_ANGLE_LIMIT)
  {
    return SCAN_HOR;
  }
  else if (abs((Int) uiDirMode - HOR_IDX) <= MDCS_ANGLE_LIMIT)
  {
    return SCAN_VER;
  }
  else
  {
    return SCAN_DIAG;
  }
}

#endif
#if HEVC_USE_RQT
bool TU::isProcessingAllQuadrants(const UnitArea &tuArea)
{
  if (tuArea.chromaFormat == CHROMA_444)
  {
    return true;
  }
  else
  {
    return tuArea.Cb().valid() && tuArea.lumaSize().width != tuArea.chromaSize().width;
  }
}

#endif
bool TU::hasCrossCompPredInfo( const TransformUnit &tu, const ComponentID &compID )
{
  return ( isChroma(compID) && tu.cs->pps->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && TU::getCbf( tu, COMPONENT_Y ) &&
         ( CU::isInter(*tu.cu) || PU::isChromaIntraModeCrossCheckMode( *tu.cs->getPU( tu.blocks[compID].pos(), toChannelType( compID ) ) ) ) );
}

UInt TU::getNumNonZeroCoeffsNonTS( const TransformUnit& tu, const bool bLuma, const bool bChroma )
{
  UInt count = 0;
  for( UInt i = 0; i < ::getNumberValidTBlocks( *tu.cs->pcv ); i++ )
  {
    if( tu.blocks[i].valid() && !tu.transformSkip[i] && TU::getCbf( tu, ComponentID( i ) ) )
    {
      if( isLuma  ( tu.blocks[i].compID ) && !bLuma   ) continue;
      if( isChroma( tu.blocks[i].compID ) && !bChroma ) continue;

      UInt area = tu.blocks[i].area();
      const TCoeff* coeff = tu.getCoeffs( ComponentID( i ) ).buf;
      for( UInt j = 0; j < area; j++ )
      {
        count += coeff[j] != 0;
      }
    }
  }
  return count;
}

bool TU::needsSqrt2Scale( const Size& size )
{
  return (((g_aucLog2[size.width] + g_aucLog2[size.height]) & 1) == 1);
}

#if HM_QTBT_AS_IN_JEM_QUANT

bool TU::needsBlockSizeTrafoScale( const Size& size )
{
  return needsSqrt2Scale( size ) || isNonLog2BlockSize( size );
}
#else
Bool TU::needsQP3Offset(const TransformUnit &tu, const ComponentID &compID)
{
  if( tu.cs->pcv->rectCUs && !tu.transformSkip[compID] )
  {
    return ( ( ( g_aucLog2[tu.blocks[compID].width] + g_aucLog2[tu.blocks[compID].height] ) & 1 ) == 1 );
  }
  return false;
}
#endif





// other tools

UInt getCtuAddr( const Position& pos, const PreCalcValues& pcv )
{
  return ( pos.x >> pcv.maxCUWidthLog2 ) + ( pos.y >> pcv.maxCUHeightLog2 ) * pcv.widthInCtus;
}



