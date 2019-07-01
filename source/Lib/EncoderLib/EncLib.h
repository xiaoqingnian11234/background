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

/** \file     EncLib.h
    \brief    encoder class (header)
*/

#ifndef __ENCTOP__
#define __ENCTOP__

// Include files
#include "CommonLib/TrQuant.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/NAL.h"

#include "Utilities/VideoIOYuv.h"

#include "EncCfg.h"
#include "EncGOP.h"
#include "EncSlice.h"
#include "VLCWriter.h"
#include "CABACWriter.h"
#include "InterSearch.h"
#include "IntraSearch.h"
#include "EncSampleAdaptiveOffset.h"
#include "RateCtrl.h"


//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder class
class EncLib : public EncCfg
{
private:

#if BLOCK_GEN
	Picture* m_bgNewBlocksOrg;
	Picture* m_bgNewBlocksRec;
	Picture* PrePicReco;
	Picture* m_bgNewBlockOrg;
	Picture* m_bgNewBlockRec;
	Picture* m_bgNewBlockReco;
#endif

#if GENERATE_OrgBG_PIC
	Picture* m_bgNewPicYuvOrg;
#endif

#if GENERATE_BG_PIC
	Picture* m_bgNewPicYuvRec;
#endif

#if GENERATE_RESI_PIC
	Picture* m_bgNewPicYuvResi;
#endif

#if GENERATE_UPDATE_RESI_PIC
	Picture* m_bgNewPicYuvUpdateResi;
#endif
#if GENERATE_TEMPRECO_PIC
	Picture* m_bgNewPicYuvReco;
#endif

#if GENERATE_RECO_PIC
	Picture* m_bgNewPicYuvTempUpdateReco;
#endif

#if BG_REFERENCE_SUBSTITUTION
	Picture* m_PicYuvTemp;
#endif
	

  // picture
  Int                       m_iPOCLast;                           ///< time index (POC)
  Int                       m_iNumPicRcvd;                        ///< number of received pictures
  UInt                      m_uiNumAllPicCoded;                   ///< number of coded pictures
  PicList                   m_cListPic;                           ///< dynamic list of pictures



  // encoder search
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  InterSearch              *m_cInterSearch;                       ///< encoder search class
  IntraSearch              *m_cIntraSearch;                       ///< encoder search class
#else
  InterSearch               m_cInterSearch;                       ///< encoder search class
  IntraSearch               m_cIntraSearch;                       ///< encoder search class
#endif
  // coding tool
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  TrQuant                  *m_cTrQuant;                           ///< transform & quantization class
#else
  TrQuant                   m_cTrQuant;                           ///< transform & quantization class
#endif
  LoopFilter                m_cLoopFilter;                        ///< deblocking filter class
  EncSampleAdaptiveOffset   m_cEncSAO;                            ///< sample adaptive offset class
  HLSWriter                 m_HLSWriter;                          ///< CAVLC encoder
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  CABACEncoder             *m_CABACEncoder;
#else
  CABACEncoder              m_CABACEncoder;
#endif

  // processing unit
  EncGOP                    m_cGOPEncoder;                        ///< GOP encoder
  EncSlice                  m_cSliceEncoder;                      ///< slice encoder
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  EncCu                    *m_cCuEncoder;                         ///< CU encoder
#else
  EncCu                     m_cCuEncoder;                         ///< CU encoder
#endif
  // SPS
  ParameterSetMap<SPS>      m_spsMap;                             ///< SPS. This is the base value. This is copied to PicSym
  ParameterSetMap<PPS>      m_ppsMap;                             ///< PPS. This is the base value. This is copied to PicSym
  // RD cost computation
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  RdCost                   *m_cRdCost;                            ///< RD cost computation class
  CtxCache                 *m_CtxCache;                           ///< buffer for temporarily stored context models
#else
  RdCost                    m_cRdCost;                            ///< RD cost computation class
  CtxCache                  m_CtxCache;                           ///< buffer for temporarily stored context models
#endif
  // quality control
  RateCtrl                  m_cRateCtrl;                          ///< Rate control class

  AUWriterIf*               m_AUWriterIf;

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  int                       m_numCuEncStacks;
#endif


public:
  Ctx                       m_entropyCodingSyncContextState;      ///< leave in addition to vector for compatibility
#if ENABLE_WPP_PARALLELISM
  std::vector<Ctx>          m_entropyCodingSyncContextStateVec;   ///< context storage for state of contexts at the wavefront/WPP/entropy-coding-sync second CTU of tile-row
#endif

protected:
  Void  xGetNewPicBuffer  ( std::list<PelUnitBuf*>& rcListPicYuvRecOut, Picture*& rpcPic, Int ppsId ); ///< get picture buffer which will be processed. If ppsId<0, then the ppsMap will be queried for the first match.
#if HEVC_VPS
  Void  xInitVPS          (VPS &vps, const SPS &sps); ///< initialize VPS from encoder options
#endif
  Void  xInitSPS          (SPS &sps);                 ///< initialize SPS from encoder options
  Void  xInitPPS          (PPS &pps, const SPS &sps); ///< initialize PPS from encoder options
#if HEVC_USE_SCALING_LISTS
  Void  xInitScalingLists (SPS &sps, PPS &pps);   ///< initialize scaling lists
#endif
  Void  xInitHrdParameters(SPS &sps);                 ///< initialize HRD parameters

#if HEVC_TILES_WPP
  Void  xInitPPSforTiles  (PPS &pps);
#endif
  Void  xInitRPS          (SPS &sps, Bool isFieldCoding);           ///< initialize PPS from encoder options

public:
  EncLib();
  virtual ~EncLib();

  Void      create          ();
  Void      destroy         ();
  Void      init            ( Bool isFieldCoding, AUWriterIf* auWriterIf );
  Void      deletePicBuffer ();

  // -------------------------------------------------------------------------------------------------------------------
  // member access functions
  // -------------------------------------------------------------------------------------------------------------------

#if BLOCK_GEN
  Void setbgNewBlocksOrg(Picture* m) { m_bgNewBlocksOrg = m; }
  Picture* getbgNewBlocksOrg() { return m_bgNewBlocksOrg; }

  Void setbgNewBlocksRec(Picture* m) { m_bgNewBlocksRec = m; }
  Picture* getbgNewBlocksRec() { return m_bgNewBlocksRec; }

  Void setPrePicReco(Picture* m) { PrePicReco = m; }
  Picture* getPrePicReco() { return PrePicReco; }

  Void setbgNewBlockOrg(Picture* m) { m_bgNewBlockOrg = m; }
  Picture* getbgNewBlockOrg() { return m_bgNewBlockOrg; }

  Void setbgNewBlockRec(Picture* m) { m_bgNewBlockRec = m; }
  Picture* getbgNewBlockRec() { return m_bgNewBlockRec; }

  Void setbgNewBlockReco(Picture* m) { m_bgNewBlockReco = m; }
  Picture* getbgNewBlockReco() { return m_bgNewBlockReco; }
#endif

#if GENERATE_OrgBG_PIC
  Void setbgNewPicYuvOrg(Picture* m) { m_bgNewPicYuvOrg = m; }
  Picture* getbgNewPicYuvOrg(){ return m_bgNewPicYuvOrg; }
#endif

#if GENERATE_BG_PIC
  Void setbgNewPicYuvRec(Picture* m) { m_bgNewPicYuvRec = m; }
  Picture* getbgNewPicYuvRec(){ return m_bgNewPicYuvRec; }
#endif

#if GENERATE_RESI_PIC
  Void setbgNewPicYuvResi(Picture* m) { m_bgNewPicYuvResi = m; }
  Picture* getbgNewPicYuvResi(){ return m_bgNewPicYuvResi; }
#endif

#if GENERATE_UPDATE_RESI_PIC
  Void setbgNewPicYuvUpdateResi(Picture* m) { m_bgNewPicYuvUpdateResi = m; }
  Picture* getbgNewPicYuvUpdateResi(){ return m_bgNewPicYuvUpdateResi; }
#endif

#if GENERATE_TEMPRECO_PIC
  Void setbgNewPicYuvReco(Picture* m) { m_bgNewPicYuvReco = m; }
  Picture* getbgNewPicYuvReco(){ return m_bgNewPicYuvReco; }
#endif

#if GENERATE_RECO_PIC
  Void setbgNewPicYuvTempUpdateReco(Picture* m) { m_bgNewPicYuvTempUpdateReco = m; }
  Picture* getbgNewPicYuvTempUpdateReco(){ return m_bgNewPicYuvTempUpdateReco; }
#endif

#if BG_REFERENCE_SUBSTITUTION
  Void setPicYuvTemp(Picture* m) { m_PicYuvTemp = m; }
  Picture* getPicYuvTemp(){ return m_PicYuvTemp; }
#endif


  AUWriterIf*             getAUWriterIf         ()              { return   m_AUWriterIf;           }
  PicList*                getListPic            ()              { return  &m_cListPic;             }
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  InterSearch*            getInterSearch        ( int jId = 0 ) { return  &m_cInterSearch[jId];    }
  IntraSearch*            getIntraSearch        ( int jId = 0 ) { return  &m_cIntraSearch[jId];    }

  TrQuant*                getTrQuant            ( int jId = 0 ) { return  &m_cTrQuant[jId];        }
#else
  InterSearch*            getInterSearch        ()              { return  &m_cInterSearch;         }
  IntraSearch*            getIntraSearch        ()              { return  &m_cIntraSearch;         }

  TrQuant*                getTrQuant            ()              { return  &m_cTrQuant;             }
#endif
  LoopFilter*             getLoopFilter         ()              { return  &m_cLoopFilter;          }
  EncSampleAdaptiveOffset* getSAO               ()              { return  &m_cEncSAO;              }
  EncGOP*                 getGOPEncoder         ()              { return  &m_cGOPEncoder;          }
  EncSlice*               getSliceEncoder       ()              { return  &m_cSliceEncoder;        }
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  EncCu*                  getCuEncoder          ( int jId = 0 ) { return  &m_cCuEncoder[jId];      }
#else
  EncCu*                  getCuEncoder          ()              { return  &m_cCuEncoder;           }
#endif
  HLSWriter*              getHLSWriter          ()              { return  &m_HLSWriter;            }
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  CABACEncoder*           getCABACEncoder       ( int jId = 0 ) { return  &m_CABACEncoder[jId];    }

  RdCost*                 getRdCost             ( int jId = 0 ) { return  &m_cRdCost[jId];         }
  CtxCache*               getCtxCache           ( int jId = 0 ) { return  &m_CtxCache[jId];        }
#else
  CABACEncoder*           getCABACEncoder       ()              { return  &m_CABACEncoder;         }

  RdCost*                 getRdCost             ()              { return  &m_cRdCost;              }
  CtxCache*               getCtxCache           ()              { return  &m_CtxCache;             }
#endif
  RateCtrl*               getRateCtrl           ()              { return  &m_cRateCtrl;            }

  Void selectReferencePictureSet(Slice* slice, Int POCCurr, Int GOPid );
  Int getReferencePictureSetIdxForSOP(Int POCCurr, Int GOPid );

  Bool                   PPSNeedsWriting(Int ppsId);
  Bool                   SPSNeedsWriting(Int spsId);
  const PPS* getPPS( int Id ) { return m_ppsMap.getPS( Id); }

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  void                   setNumCuEncStacks( int n )             { m_numCuEncStacks = n; }
  int                    getNumCuEncStacks()              const { return m_numCuEncStacks; }
#endif

  // -------------------------------------------------------------------------------------------------------------------
  // encoder function
  // -------------------------------------------------------------------------------------------------------------------

  /// encode several number of pictures until end-of-sequence
  Void encode( Bool bEos,
               PelStorage* pcPicYuvOrg,
               PelStorage* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
               std::list<PelUnitBuf*>& rcListPicYuvRecOut,
               Int& iNumEncoded );

  /// encode several number of pictures until end-of-sequence
  Void encode( Bool bEos,
               PelStorage* pcPicYuvOrg,
               PelStorage* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
               std::list<PelUnitBuf*>& rcListPicYuvRecOut,
               Int& iNumEncoded, Bool isTff );


  Void printSummary(Bool isField) { m_cGOPEncoder.printOutSummary (m_uiNumAllPicCoded, isField, m_printMSEBasedSequencePSNR, m_printSequenceMSE, m_spsMap.getFirstPS()->getBitDepths()); }

};

//! \}

#endif // __ENCTOP__

