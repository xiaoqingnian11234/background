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

/** \file     EncApp.h
    \brief    Encoder application class (header)
*/

#ifndef __ENCAPP__
#define __ENCAPP__

#include <list>
#include <ostream>

#include "EncoderLib/EncLib.h"
#include "Utilities/VideoIOYuv.h"
#include "CommonLib/NAL.h"
#include "EncAppCfg.h"

//! \ingroup EncoderApp
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder application class
class EncApp : public EncAppCfg, public AUWriterIf
{
private:
  // class interface
  EncLib            m_cEncLib;                    ///< encoder class
  VideoIOYuv        m_cVideoIOYuvInputFile;       ///< input YUV file
  VideoIOYuv        m_cVideoIOYuvReconFile;       ///< output reconstruction file
  Int               m_iFrameRcvd;                 ///< number of received frames
  UInt              m_essentialBytes;
  UInt              m_totalBytes;
  fstream           m_bitstream;

private:
  // initialization
  Void xCreateLib  ( std::list<PelUnitBuf*>& recBufList
                    );                           ///< create files & encoder class
  Void xInitLibCfg ();                           ///< initialize internal variables
  Void xInitLib    (Bool isFieldCoding);         ///< initialize encoder class
  Void xDestroyLib ();                           ///< destroy encoder class

  // file I/O
  Void xWriteOutput     ( Int iNumEncoded, std::list<PelUnitBuf*>& recBufList
                         );                      ///< write bitstream to file
  Void rateStatsAccum   ( const AccessUnit& au, const std::vector<UInt>& stats);
  Void printRateSummary ();
  Void printChromaFormat();
#if  PRINT_OrgBAC_PIC_FLAG
  TVideoIOYuv                m_cTVideoIOYuvOrgBgFile;       ///< output original background file
#endif

#if BLOCK_GEN
  Picture* bg_NewBlocksOrg; //Compose Org
  Picture* bg_NewBlocksRec; //Compose Rec
  Picture* PrePicReco;//skip
  Picture* bg_NewBlockOrg;  //Org
  Picture* bg_NewBlockRec;  //˫����֡Rec
  Picture* bg_NewBlockReco; //����Reco
#endif

#if GENERATE_OrgBG_PIC
  Picture* bg_NewPicYuvOrg;
#endif

#if GENERATE_BG_PIC
  Picture* bg_NewPicYuvRec;
#endif

#if GENERATE_RESI_PIC
  Picture* bg_NewPicYuvResi;
#endif

#if GENERATE_UPDATE_RESI_PIC
  Picture* bg_NewPicYuvUpdateResi;
#endif

#if GENERATE_TEMPRECO_PIC
  Picture* bg_NewPicYuvReco;
#endif

#if GENERATE_RECO_PIC
  Picture* bg_NewPicYuvTempUpdateReco;
#endif

#if BG_REFERENCE_SUBSTITUTION
  Picture*	 rcPicYuvTemp;
#endif

public:
  EncApp();
  virtual ~EncApp();

  Void  encode();                               ///< main encoding function

  void  outputAU( const AccessUnit& au );

};// END CLASS DEFINITION EncApp

//! \}

#endif // __ENCAPP__

