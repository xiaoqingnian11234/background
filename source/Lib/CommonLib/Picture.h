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

/** \file     Picture.h
 *  \brief    Description of a coded picture
 */

#ifndef __PICTURE__
#define __PICTURE__

#include "CommonDef.h"

#include "Common.h"
#include "Unit.h"
#include "Buffer.h"
#include "Unit.h"
#include "Slice.h"
#include "CodingStructure.h"
#include <iostream>
using namespace std;

#include <deque>

#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
#if ENABLE_WPP_PARALLELISM
#include <mutex>
class SyncObj;
#endif

#define CURR_THREAD_ID -1

class Scheduler
{
public:
  Scheduler();
  ~Scheduler();

#if ENABLE_SPLIT_PARALLELISM
  unsigned getSplitDataId( int jobId = CURR_THREAD_ID ) const;
  unsigned getSplitPicId ( int tId   = CURR_THREAD_ID ) const;
  unsigned getSplitJobId () const;
  void     setSplitJobId ( const int jobId );
  void     startParallel ();
  void     finishParallel();
  void     setSplitThreadId( const int tId = CURR_THREAD_ID );
  unsigned getNumSplitThreads() const { return m_numSplitThreads; };
#endif
#if ENABLE_WPP_PARALLELISM
  unsigned getWppDataId  ( int lId = CURR_THREAD_ID ) const;
  unsigned getWppThreadId() const;
  void     setWppThreadId( const int tId = CURR_THREAD_ID );
#endif
  unsigned getDataId     () const;
  bool init              ( const int ctuYsize, const int ctuXsize, const int numWppThreadsRunning, const int numWppExtraLines, const int numSplitThreads );
  int  getNumPicInstances() const;
#if ENABLE_WPP_PARALLELISM
  void setReady          ( const int ctuPosX, const int ctuPosY );
  void wait              ( const int ctuPosX, const int ctuPosY );

private:
  bool getNextCtu( Position& pos, int ctuLine, int offset );

private:
  int m_firstNonFinishedLine;
  int m_numWppThreads;
  int m_numWppThreadsRunning;
  int m_numWppDataInstances;
  int m_ctuYsize;
  int m_ctuXsize;

  std::vector<int>         m_LineDone;
  std::vector<bool>        m_LineProc;
  std::mutex               m_mutex;
  std::vector<SyncObj*>    m_SyncObjs;
#endif
#if ENABLE_SPLIT_PARALLELISM

  int   m_numSplitThreads;
  bool  m_hasParallelBuffer;
#endif
};
#endif

class SEI;
class AQpLayer;

typedef std::list<SEI*> SEIMessages;

#if HEVC_TILES_WPP
class Tile
{
private:
  UInt      m_tileWidthInCtus;
  UInt      m_tileHeightInCtus;
  UInt      m_rightEdgePosInCtus;
  UInt      m_bottomEdgePosInCtus;
  UInt      m_firstCtuRsAddr;

public:
  Tile();
  virtual ~Tile();

  Void      setTileWidthInCtus     ( UInt i )            { m_tileWidthInCtus = i; }
  UInt      getTileWidthInCtus     () const              { return m_tileWidthInCtus; }
  Void      setTileHeightInCtus    ( UInt i )            { m_tileHeightInCtus = i; }
  UInt      getTileHeightInCtus    () const              { return m_tileHeightInCtus; }
  Void      setRightEdgePosInCtus  ( UInt i )            { m_rightEdgePosInCtus = i; }
  UInt      getRightEdgePosInCtus  () const              { return m_rightEdgePosInCtus; }
  Void      setBottomEdgePosInCtus ( UInt i )            { m_bottomEdgePosInCtus = i; }
  UInt      getBottomEdgePosInCtus () const              { return m_bottomEdgePosInCtus; }
  Void      setFirstCtuRsAddr      ( UInt i )            { m_firstCtuRsAddr = i; }
  UInt      getFirstCtuRsAddr      () const              { return m_firstCtuRsAddr; }
};


struct TileMap
{
  TileMap();

  Void create( const SPS& sps, const PPS& pps );
  Void destroy();

  UInt getTileIdxMap( UInt ctuRsAddr )       const { return *(tileIdxMap + ctuRsAddr); }
  UInt getTileIdxMap( const Position& pos )  const { return getTileIdxMap( ( pos.x / pcv->maxCUWidth ) + ( pos.y / pcv->maxCUHeight ) * pcv->widthInCtus ); };
  UInt getCtuTsToRsAddrMap( UInt ctuTsAddr ) const { return *(ctuTsToRsAddrMap + (ctuTsAddr>=pcv->sizeInCtus ? pcv->sizeInCtus : ctuTsAddr)); }
  UInt getCtuRsToTsAddrMap( UInt ctuRsAddr ) const { return *(ctuRsToTsAddrMap + (ctuRsAddr>=pcv->sizeInCtus ? pcv->sizeInCtus : ctuRsAddr)); }
  UInt getSubstreamForCtuAddr(const UInt ctuAddr, const Bool bAddressInRaster, Slice *pcSlice) const;

  const PreCalcValues* pcv;
  std::vector<Tile> tiles;
  UInt  numTiles;
  UInt  numTileColumns;
  UInt  numTileRows;
  UInt* tileIdxMap;
  UInt* ctuTsToRsAddrMap;
  UInt* ctuRsToTsAddrMap;

  void initTileMap( const SPS& sps, const PPS& pps );
  void initCtuTsRsAddrMap();
  UInt calculateNextCtuRSAddr( const UInt currCtuRsAddr ) const;
};
#endif

#if ENABLE_SPLIT_PARALLELISM
#define M_BUFS(JID,PID) m_bufs[JID][PID]
#else
#define M_BUFS(JID,PID) m_bufs[PID]
#endif

struct Picture : public UnitArea
{
  UInt margin;
  Picture();

  Void create(const ChromaFormat &_chromaFormat, const Size &size, const unsigned _maxCUSize, const unsigned margin, const bool bDecoder);
  Void destroy();

  Void createTempBuffers( const unsigned _maxCUSize );
  Void destroyTempBuffers();

         PelBuf     getOrigBuf(const CompArea &blk);
  const CPelBuf     getOrigBuf(const CompArea &blk) const;
         PelUnitBuf getOrigBuf(const UnitArea &unit);
  const CPelUnitBuf getOrigBuf(const UnitArea &unit) const;
         PelUnitBuf getOrigBuf();
  const CPelUnitBuf getOrigBuf() const;

         PelBuf     getPredBuf(const CompArea &blk);
  const CPelBuf     getPredBuf(const CompArea &blk) const;
         PelUnitBuf getPredBuf(const UnitArea &unit);
  const CPelUnitBuf getPredBuf(const UnitArea &unit) const;

         PelBuf     getResiBuf(const CompArea &blk);
  const CPelBuf     getResiBuf(const CompArea &blk) const;
         PelUnitBuf getResiBuf(const UnitArea &unit);
  const CPelUnitBuf getResiBuf(const UnitArea &unit) const;

         PelBuf     getRecoBuf(const ComponentID compID);
  const CPelBuf     getRecoBuf(const ComponentID compID) const;
         PelBuf     getRecoBuf(const CompArea &blk);
  const CPelBuf     getRecoBuf(const CompArea &blk) const;
         PelUnitBuf getRecoBuf(const UnitArea &unit);
  const CPelUnitBuf getRecoBuf(const UnitArea &unit) const;
         PelUnitBuf getRecoBuf();
  const CPelUnitBuf getRecoBuf() const;

         PelBuf     getBuf(const ComponentID compID, const PictureType &type);
  const CPelBuf     getBuf(const ComponentID compID, const PictureType &type) const;
         PelBuf     getBuf(const CompArea &blk,      const PictureType &type);
  const CPelBuf     getBuf(const CompArea &blk,      const PictureType &type) const;
         PelUnitBuf getBuf(const UnitArea &unit,     const PictureType &type);
  const CPelUnitBuf getBuf(const UnitArea &unit,     const PictureType &type) const;

  void extendPicBorder();
  void finalInit( const SPS& sps, const PPS& pps );

  int  getPOC()                               const { return poc; }
  Void setBorderExtension( bool bFlag)              { m_bIsBorderExtended = bFlag;}

public:
  bool m_bIsBorderExtended;
  bool referenced;
  bool reconstructed;
  bool neededForOutput;
  bool usedByCurr;
  bool longTerm;
  bool topField;
  bool fieldPic;
  int  m_prevQP[MAX_NUM_CHANNEL_TYPE];

  Int  poc;
  UInt layer;
  UInt depth;
  //vector<int> BgBlock;

#if ENABLE_SPLIT_PARALLELISM
#if ENABLE_WPP_PARALLELISM
  PelStorage m_bufs[( PARL_SPLIT_MAX_NUM_JOBS * PARL_WPP_MAX_NUM_THREADS )][NUM_PIC_TYPES];
#else
  PelStorage m_bufs[PARL_SPLIT_MAX_NUM_JOBS][NUM_PIC_TYPES];
#endif
#else
  PelStorage m_bufs[NUM_PIC_TYPES];
#endif

  CodingStructure*   cs;
  std::deque<Slice*> slices;
  SEIMessages        SEIs;

  Void         allocateNewSlice();
  Slice        *swapSliceObject(Slice * p, UInt i);
  void         clearSliceBuffer();

#if HEVC_TILES_WPP
  TileMap*     tileMap;
#endif
  std::vector<AQpLayer*> aqlayer;

#if !KEEP_PRED_AND_RESI_SIGNALS
private:
  UnitArea m_ctuArea;
#endif

#if ENABLE_SPLIT_PARALLELISM
public:
  void finishParallelPart   ( const UnitArea& ctuArea );
#if ENABLE_WPP_PARALLELISM
  void finishCtuPart        ( const UnitArea& ctuArea );
#endif
#endif
#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
public:
  Scheduler                  scheduler;
#endif

public:
  SAOBlkParam    *getSAO(int id = 0)                        { return &m_sao[id][0]; };
  void            resizeSAO(unsigned numEntries, int dstid) { m_sao[dstid].resize(numEntries); }
  void            copySAO(const Picture& src, int dstid)    { std::copy(src.m_sao[0].begin(), src.m_sao[0].end(), m_sao[dstid].begin()); }

#if ENABLE_QPA
  std::vector<double>     m_uEnerHpCtu;                         ///< CTU-wise L2 or squared L1 norm of high-passed luma input
  std::vector<Pel>        m_iOffsetCtu;                         ///< CTU-wise DC offset (later QP index offset) of luma input
#endif

  std::vector<SAOBlkParam> m_sao[2];
  /*Void Picture::setBgblock(Int block[]) 
  {
	  for (int i = 0; block[i] != -1; i++)
	  {
		  BgBlock.push_back(block[i]);
	  }
  }
  vector<int> getBgblock() { return BgBlock; }
  */
#if GENERATE_BG_PIC
  Void Picture::Copy2BackPic(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic, Int level);
#endif
#if GENERATE_OrgBG_PIC
  Void Picture::Copy2OrgBackPic(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic, Int level);
  Void Picture::CopyOrgPicMean(Picture* backPic, Picture* backPic2, Picture *pcPic, UInt uiW, UInt uiH, Int level);
  Void Picture::CopyRecPicMean(Picture* backPic, Picture* backPic2, Picture *pcPic, UInt uiW, UInt uiH, Int level);
  Bool Picture::IsEmpty(Picture* pcPic, UInt uiW, UInt uiH, Int level);/*315*/
#endif

#if BLOCK_GEN
  Void Picture::CopyOrg2CTU(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic);
  Void Picture::CopyOrg2Block(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic);
  Void Picture::CopyPreReco2Block(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic);
  Void Picture::DrawRef(UInt uiW, UInt uiH, Picture* pcPic, Int Refnum);
  Void Picture::CopyReco2CTU(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic);
  Void Picture::CopyReco2Block(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic);
  Void Picture::DeleteOrg(Picture* backPic);
  Void Picture::DeleteReco(Picture* backPic);
#endif
#if OrgBG_BLOCK_SUBSTITUTION
  Void Picture::Copy2OrgBackPic(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic);
#endif
#if BG_BLOCK_SUBSTITUTION
  Void Picture::Copy2BackPic(Picture* backPic, UInt uiW, UInt uiH, Picture* pcPic);
#endif
#if BG_REFERENCE_SUBSTITUTION
  Void Picture::Copy2Temp(Picture* TempPicYuv, Picture* pcPic);  //Pic copy to Temp
#endif
#if ENCODE_BGPIC
  Void Picture::CopyPic2Reco(Picture* TempPicYuv, Picture* pcPic);  //Pic copy to Reco
#endif
#if BG_REFERENCE_SUBSTITUTION
  Void Picture::CopyBGYuv(Picture* bgPicYuv, Picture* pcPic);
#endif
#if BG_REFERENCE_SUBSTITUTION
  Void Picture::CopyBack(Picture* TempPicYuv, Picture* pcPic);
#endif
#if HIERARCHY_GENETATE_BGP
  Void Picture::xCompDiff(UInt uiW, UInt uiH, Picture* pcPic, double& diff, Int level);
#endif
#if BLOCK_GEN
  Void Picture::CompBlockDiff(UInt uiW, UInt uiH, Picture* pcPic, double& diff);
  Void Picture::CompBlockDiffOrg(UInt uiW, UInt uiH, Picture* PicYuvOrg, Picture* pcPic, double& diff);
  Void Picture::CopyBlock(UInt uiW, UInt uiH, Picture* PicYuvOrg, Picture* pcPic);
  Void Picture::CompBlockPicBgPdpp(UInt uiW, UInt uiH, Picture* pcPic, Picture* PicYuvOrg, double dpp, double& P);
  Void Picture::CompBlockPicdpp(UInt uiW, UInt uiH, Picture* pcPic, double& dpp);
  Void Picture::CompBlockPicbgdpp(UInt uiW, UInt uiH, Picture* pcPic, Picture* bgpic, double& dpp);
  Void Picture::CompBlockPicOrgDiff(UInt uiW, UInt uiH, Picture* pcPic, Picture* PicYuvOrg, double& diff);
  Void Picture::CompBlockPicRecoDiff(UInt uiW, UInt uiH, Picture* pcPic, Picture* PicYuvOrg, double& diff);
  Bool Picture::CompBlockRecoIsSimilar(UInt uiW, UInt uiH, Picture* pcPic, Picture* PicYuvOrg);
  Bool Picture::CompBlockOrgIsSimilar(UInt uiW, UInt uiH, Picture* pcPic, Picture* PicYuvOrg);
  Bool Picture::CompBlockOrgIsFull(UInt uiW, UInt uiH, Picture* pcPic);
#endif
#if HIERARCHY_GENETATE_OrgBGP
  Void Picture::xCompDiffOrg(UInt uiW, UInt uiH, Picture* pcPic, double& diff, Int level);
  Void Picture::xCompDiffBlock(Picture* PicYuvOrg, UInt uiW, UInt uiH, Picture* pcPic, double& diff, Int level);
#endif
#if ENCODE_BGPIC
  Void Picture::SetOrg0( Picture* pcPic);// pcPic->org = 0
  Void Picture::SetReco0( Picture* pcPic);// pcPic->Reco = 0
  Void Picture::CopyOrg(Picture* BackOrg, Picture* pcPic);// pcPic->org = BackOrg->org
  Void Picture::CopyReco2Org(Picture* Back, Picture* pcPic);// pcPic->org = BackOrg->Reco
  Void Picture::CopyReco(Picture* BackReco, Picture* pcPic);// pcPic->Reco = BackReco->Reco
  Void Picture::CompPicOrgDiff(Picture* Back, Picture* pcPic, Double& diff);
#endif // ENCODE_BGPIC


};

int calcAndPrintHashStatus(const CPelUnitBuf& pic, const class SEIDecodedPictureHash* pictureHashSEI, const BitDepths &bitDepths, const MsgLevel msgl);


typedef std::list<Picture*> PicList;

#endif
