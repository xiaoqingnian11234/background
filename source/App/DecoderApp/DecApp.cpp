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

/** \file     DecApp.cpp
    \brief    Decoder application class
*/

#include <list>
#include <vector>
#include <stdio.h>
#include <fcntl.h>

#include "DecApp.h"
#include "DecoderLib/AnnexBread.h"
#include "DecoderLib/NALread.h"
#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif
#include "CommonLib/dtrace_codingstruct.h"


//! \ingroup DecoderApp
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

DecApp::DecApp()
: m_iPOCLastDisplay(-MAX_INT)
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 - create internal class
 - initialize internal class
 - until the end of the bitstream, call decoding function in DecApp class
 - delete allocated buffers
 - destroy internal class
 - returns the number of mismatching pictures
 */
UInt DecApp::decode()
{
  Int                 poc;
  PicList* pcListPic = NULL;

  ifstream bitstreamFile(m_bitstreamFileName.c_str(), ifstream::in | ifstream::binary);
  if (!bitstreamFile)
  {
    EXIT( "failed to open bitstream file " << m_bitstreamFileName.c_str() << " for reading" ) ;
  }

  InputByteStream bytestream(bitstreamFile);

  if (!m_outputDecodedSEIMessagesFilename.empty() && m_outputDecodedSEIMessagesFilename!="-")
  {
    m_seiMessageFileStream.open(m_outputDecodedSEIMessagesFilename.c_str(), std::ios::out);
    if (!m_seiMessageFileStream.is_open() || !m_seiMessageFileStream.good())
    {
      EXIT( "Unable to open file "<< m_outputDecodedSEIMessagesFilename.c_str() << " for writing decoded SEI messages");
    }
  }

  // create & initialize internal classes
  xCreateDecLib();

  m_iPOCLastDisplay += m_iSkipFrame;      // set the last displayed POC correctly for skip forward.

  // clear contents of colour-remap-information-SEI output file
  if (!m_colourRemapSEIFileName.empty())
  {
    std::ofstream ofile(m_colourRemapSEIFileName.c_str());
    if (!ofile.good() || !ofile.is_open())
    {
      EXIT( "Unable to open file " << m_colourRemapSEIFileName.c_str() << " for writing colour-remap-information-SEI video");
    }
  }

  // main decoder loop
  Bool openedReconFile = false; // reconstruction file not yet opened. (must be performed after SPS is seen)
  Bool loopFiltered = false;
  /*================================================*/

  const UInt uiWidth = 1280;//m_cDecLib.getpcPic()->getOrigBuf().Y().width;   //1280;
  const UInt uiHeight = 720;// m_cDecLib.getpcPic()->getOrigBuf().Y().height;

  ChromaFormat m_chromaFormatConstraint = CHROMA_420;
#if GENERATE_BG_PIC
  
  bg_NewPicYuvRec = new Picture;
  bg_NewPicYuvRec->create(m_chromaFormatConstraint, Size(uiWidth, uiHeight), MAX_CU_SIZE, MAX_CU_SIZE + 16, false);
  m_cDecLib.setbgNewPicYuvRec(bg_NewPicYuvRec);
#endif

#if BLOCK_GEN

  bg_NewBlocksRec = new Picture;
  bg_NewBlocksRec->create(m_chromaFormatConstraint, Size(uiWidth, uiHeight), MAX_CU_SIZE, MAX_CU_SIZE + 16, false);
  m_cDecLib.setbgNewBlocksRec(bg_NewBlocksRec);
#endif
  
#if GENERATE_RESI_PIC

  /*bg_NewPicYuvResi = new Picture;
  bg_NewPicYuvResi->create(m_chromaFormatConstraint, Size(uiWidth, uiHeight), MAX_CU_SIZE, MAX_CU_SIZE + 16, false);

  m_cDecLib.setbgNewPicYuvResi(bg_NewPicYuvResi);*/
#endif

#if GENERATE_UPDATE_RESI_PIC

  bg_NewPicYuvUpdateResi = new TComPicYuv;
  bg_NewPicYuvUpdateResi->create(uiWidth, uiHeight,
	  g_uiMaxCUWidth,
	  g_uiMaxCUHeight,
	  g_uiMaxCUDepth
	  );

  m_cTDecTop.setbgNewPicYuvUpdateResi(bg_NewPicYuvUpdateResi);
#endif

#if GENERATE_TEMPRECO_PIC

  bg_NewPicYuvReco = new Picture;
  bg_NewPicYuvReco->create(m_chromaFormatConstraint, Size(uiWidth, uiHeight), MAX_CU_SIZE, MAX_CU_SIZE + 16, true);

  m_cDecLib.setbgNewPicYuvReco(bg_NewPicYuvReco);
#endif

#if BG_REFERENCE_SUBSTITUTION
  Picture* rcPicYuvTemp = new Picture;

  rcPicYuvTemp->create(m_chromaFormatConstraint, Size(uiWidth, uiHeight), MAX_CU_SIZE, MAX_CU_SIZE + 16, false);

  m_cDecLib.setPicYuvTemp(rcPicYuvTemp);
#endif

#if ENCODE_BGPIC
  Bool isF;
  isF = true;
  Bool isO;   
  isO = true;
#endif

  while (!!bitstreamFile)
  {
    /* location serves to work around a design fault in the decoder, whereby
     * the process of reading a new slice that is the first slice of a new frame
     * requires the DecApp::decode() method to be called again with the same
     * nal unit. */
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatistics::CodingStatisticsData* backupStats = new CodingStatistics::CodingStatisticsData(CodingStatistics::GetStatistics());
    streampos location = bitstreamFile.tellg() - streampos(bytestream.GetNumBufferedBytes());
#else
    streampos location = bitstreamFile.tellg();
#endif
    AnnexBStats stats = AnnexBStats();

    InputNALUnit nalu;
    byteStreamNALUnit(bytestream, nalu.getBitstream().getFifo(), stats);

    // call actual decoding function
    Bool bNewPicture = false;
    if (nalu.getBitstream().getFifo().empty())
    {
      /* this can happen if the following occur:
       *  - empty input file
       *  - two back-to-back start_code_prefixes
       *  - start_code_prefix immediately followed by EOF
       */
      msg( ERROR, "Warning: Attempt to decode an empty NAL unit\n");
    }
    else
    {
      read(nalu);

      if( (m_iMaxTemporalLayer >= 0 && nalu.m_temporalId > m_iMaxTemporalLayer) || !isNaluWithinTargetDecLayerIdSet(&nalu)  )
      {
        bNewPicture = false;
      }
      else
      {
        bNewPicture = m_cDecLib.decode(nalu, m_iSkipFrame, m_iPOCLastDisplay
#if ENCODE_BGPIC
		,isO
#endif
		);
        if (bNewPicture)
        {
          bitstreamFile.clear();
          /* location points to the current nalunit payload[1] due to the
           * need for the annexB parser to read three extra bytes.
           * [1] except for the first NAL unit in the file
           *     (but bNewPicture doesn't happen then) */
#if RExt__DECODER_DEBUG_BIT_STATISTICS
          bitstreamFile.seekg(location);
          bytestream.reset();
          CodingStatistics::SetStatistics(*backupStats);
#else
          bitstreamFile.seekg(location-streamoff(3));
          bytestream.reset();
#endif
        }
      }
    }



    if( ( bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS ) && !m_cDecLib.getFirstSliceInSequence() )
    {
      if (!loopFiltered || bitstreamFile)
      {
        m_cDecLib.executeLoopFilters();
        m_cDecLib.finishPicture( poc, pcListPic );
      }
      loopFiltered = (nalu.m_nalUnitType == NAL_UNIT_EOS);
      if (nalu.m_nalUnitType == NAL_UNIT_EOS)
      {
        m_cDecLib.setFirstSliceInSequence(true);
      }

    }
    else if ( (bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS ) &&
              m_cDecLib.getFirstSliceInSequence () )
    {
      m_cDecLib.setFirstSliceInPicture (true);
    }

    if( pcListPic )
    {
      if ( (!m_reconFileName.empty()) && (!openedReconFile) )
      {
        const BitDepths &bitDepths=pcListPic->front()->cs->sps->getBitDepths(); // use bit depths of first reconstructed picture.
        for( UInt channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++ )
        {
            if( m_outputBitDepth[channelType] == 0 )
            {
                m_outputBitDepth[channelType] = bitDepths.recon[channelType];
            }
        }

        m_cVideoIOYuvReconFile.open( m_reconFileName, true, m_outputBitDepth, m_outputBitDepth, bitDepths.recon ); // write mode
        openedReconFile = true;
      }
      // write reconstruction to file
      if( bNewPicture )
      {
        xWriteOutput( pcListPic, nalu.m_temporalId );
      }
      if ( (bNewPicture || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA) && m_cDecLib.getNoOutputPriorPicsFlag() )
      {
        m_cDecLib.checkNoOutputPriorPics( pcListPic );
        m_cDecLib.setNoOutputPriorPicsFlag (false);
      }
      if ( bNewPicture &&
           (   nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL
            || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP
            || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP
            || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL
            || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP ) )
      {
        xFlushOutput( pcListPic );
      }
      if (nalu.m_nalUnitType == NAL_UNIT_EOS)
      {
        xWriteOutput( pcListPic, nalu.m_temporalId );
        m_cDecLib.setFirstSliceInPicture (false);
      }
      // write reconstruction to file -- for additional bumping as defined in C.5.2.3
      if(!bNewPicture && nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL_N && nalu.m_nalUnitType <= NAL_UNIT_RESERVED_VCL31)
      {
        xWriteOutput( pcListPic, nalu.m_temporalId );
      }
    }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    delete backupStats;
#endif
  }

  xFlushOutput( pcListPic );

  // get the number of checksum errors
  UInt nRet = m_cDecLib.getNumberOfChecksumErrorsDetected();

  // delete buffers
  m_cDecLib.deletePicBuffer();
  
#if GENERATE_BG_PIC
  bg_NewPicYuvRec->destroy();
  delete bg_NewPicYuvRec;
  bg_NewPicYuvRec = NULL;
#endif
#if BLOCK_GEN
  bg_NewBlocksRec->destroy();
  delete bg_NewBlocksRec;
  bg_NewBlocksRec = NULL;
#endif
  /*
#if GENERATE_RESI_PIC
  bg_NewPicYuvResi->destroy();
  delete bg_NewPicYuvResi;
  bg_NewPicYuvResi = NULL;
#endif

#if GENERATE_UPDATE_RESI_PIC
  bg_NewPicYuvUpdateResi->destroy();
  delete bg_NewPicYuvUpdateResi;
  bg_NewPicYuvUpdateResi = NULL;
#endif
*/
#if GENERATE_TEMPRECO_PIC
  bg_NewPicYuvReco->destroy();
  delete bg_NewPicYuvReco;
  bg_NewPicYuvReco = NULL;
#endif
 
#if BG_REFERENCE_SUBSTITUTION
  if (rcPicYuvTemp)
  {
	  rcPicYuvTemp->destroy();
	  delete rcPicYuvTemp;
	  rcPicYuvTemp = NULL;
  }
#endif
 
  // destroy internal classes
  xDestroyDecLib();

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::DestroyInstance();
#endif

  destroyROM();

  return nRet;
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

Void DecApp::xCreateDecLib()
{
  initROM();

  // create decoder class
  m_cDecLib.create();

  // initialize decoder class
  m_cDecLib.init();
  m_cDecLib.setDecodedPictureHashSEIEnabled(m_decodedPictureHashSEIEnabled);
  if (!m_outputDecodedSEIMessagesFilename.empty())
  {
    std::ostream &os=m_seiMessageFileStream.is_open() ? m_seiMessageFileStream : std::cout;
    m_cDecLib.setDecodedSEIMessageOutputStream(&os);
  }
}

Void DecApp::xDestroyDecLib()
{
  if ( !m_reconFileName.empty() )
  {
    m_cVideoIOYuvReconFile.close();
  }

  // destroy decoder class
  m_cDecLib.destroy();
}


/** \param pcListPic list of pictures to be written to file
    \param tId       temporal sub-layer ID
 */
Void DecApp::xWriteOutput( PicList* pcListPic, UInt tId )
{
  if (pcListPic->empty())
  {
    return;
  }

  PicList::iterator iterPic   = pcListPic->begin();
  Int numPicsNotYetDisplayed = 0;
  Int dpbFullness = 0;
  const SPS* activeSPS = (pcListPic->front()->cs->sps);
  UInt numReorderPicsHighestTid;
  UInt maxDecPicBufferingHighestTid;
  UInt maxNrSublayers = activeSPS->getMaxTLayers();

  if(m_iMaxTemporalLayer == -1 || m_iMaxTemporalLayer >= maxNrSublayers)
  {
    numReorderPicsHighestTid = activeSPS->getNumReorderPics(maxNrSublayers-1);
    maxDecPicBufferingHighestTid =  activeSPS->getMaxDecPicBuffering(maxNrSublayers-1);
  }
  else
  {
    numReorderPicsHighestTid = activeSPS->getNumReorderPics(m_iMaxTemporalLayer);
    maxDecPicBufferingHighestTid = activeSPS->getMaxDecPicBuffering(m_iMaxTemporalLayer);
  }

  while (iterPic != pcListPic->end())
  {
    Picture* pcPic = *(iterPic);
    if(pcPic->neededForOutput && pcPic->getPOC() > m_iPOCLastDisplay)
    {
       numPicsNotYetDisplayed++;
      dpbFullness++;
    }
    else if(pcPic->referenced)
    {
      dpbFullness++;
    }
    iterPic++;
  }

  iterPic = pcListPic->begin();

  if (numPicsNotYetDisplayed>2)
  {
    iterPic++;
  }

  Picture* pcPic = *(iterPic);
  if( numPicsNotYetDisplayed>2 && pcPic->fieldPic ) //Field Decoding
  {
    PicList::iterator endPic   = pcListPic->end();
    endPic--;
    iterPic   = pcListPic->begin();
    while (iterPic != endPic)
    {
      Picture* pcPicTop = *(iterPic);
      iterPic++;
      Picture* pcPicBottom = *(iterPic);

      if ( pcPicTop->neededForOutput && pcPicBottom->neededForOutput &&
          (numPicsNotYetDisplayed >  numReorderPicsHighestTid || dpbFullness > maxDecPicBufferingHighestTid) &&
          (!(pcPicTop->getPOC()%2) && pcPicBottom->getPOC() == pcPicTop->getPOC()+1) &&
          (pcPicTop->getPOC() == m_iPOCLastDisplay+1 || m_iPOCLastDisplay < 0))
      {
        // write to file
        numPicsNotYetDisplayed = numPicsNotYetDisplayed-2;
        if ( !m_reconFileName.empty() )
        {
          const Window &conf = pcPicTop->cs->sps->getConformanceWindow();
          const Window  defDisp = (m_respectDefDispWindow && pcPicTop->cs->sps->getVuiParametersPresentFlag()) ? pcPicTop->cs->sps->getVuiParameters()->getDefaultDisplayWindow() : Window();
          const Bool isTff = pcPicTop->topField;

          Bool display = true;
          if( m_decodedNoDisplaySEIEnabled )
          {
            SEIMessages noDisplay = getSeisByType( pcPic->SEIs, SEI::NO_DISPLAY );
            const SEINoDisplay *nd = ( noDisplay.size() > 0 ) ? (SEINoDisplay*) *(noDisplay.begin()) : NULL;
            if( (nd != NULL) && nd->m_noDisplay )
            {
              display = false;
            }
          }

          if (display)
          {
            m_cVideoIOYuvReconFile.write( pcPicTop->getRecoBuf(), pcPicBottom->getRecoBuf(),
                                           m_outputColourSpaceConvert,
                                           conf.getWindowLeftOffset() + defDisp.getWindowLeftOffset(),
                                           conf.getWindowRightOffset() + defDisp.getWindowRightOffset(),
                                           conf.getWindowTopOffset() + defDisp.getWindowTopOffset(),
                                           conf.getWindowBottomOffset() + defDisp.getWindowBottomOffset(), NUM_CHROMA_FORMAT, isTff );
          }
        }

        // update POC of display order
        m_iPOCLastDisplay = pcPicBottom->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if ( ! pcPicTop->referenced && pcPicTop->reconstructed )
        {
          pcPicTop->reconstructed = false;
        }
        if ( ! pcPicBottom->referenced && pcPicBottom->reconstructed )
        {
          pcPicBottom->reconstructed = false;
        }
        pcPicTop->neededForOutput = false;
        pcPicBottom->neededForOutput = false;
      }
    }
  }
  else if( !pcPic->fieldPic ) //Frame Decoding
  {
    iterPic = pcListPic->begin();

    while (iterPic != pcListPic->end())
    {
      pcPic = *(iterPic);

      if(pcPic->neededForOutput && pcPic->getPOC() > m_iPOCLastDisplay &&
        (numPicsNotYetDisplayed >  numReorderPicsHighestTid || dpbFullness > maxDecPicBufferingHighestTid))
      {
        // write to file
        numPicsNotYetDisplayed--;
        if (!pcPic->referenced)
        {
          dpbFullness--;
        }


        if (!m_reconFileName.empty())
        {
          const Window &conf    = pcPic->cs->sps->getConformanceWindow();
          const Window  defDisp = (m_respectDefDispWindow && pcPic->cs->sps->getVuiParametersPresentFlag()) ? pcPic->cs->sps->getVuiParameters()->getDefaultDisplayWindow() : Window();

          m_cVideoIOYuvReconFile.write( pcPic->getRecoBuf(),
                                        m_outputColourSpaceConvert,
                                        conf.getWindowLeftOffset()   + defDisp.getWindowLeftOffset(),
                                        conf.getWindowRightOffset()  + defDisp.getWindowRightOffset(),
                                        conf.getWindowTopOffset()    + defDisp.getWindowTopOffset(),
                                        conf.getWindowBottomOffset() + defDisp.getWindowBottomOffset(),
                                        NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
        }

        if (m_seiMessageFileStream.is_open())
        {
          m_cColourRemapping.outputColourRemapPic (pcPic, m_seiMessageFileStream);
        }

        // update POC of display order
        m_iPOCLastDisplay = pcPic->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if (!pcPic->referenced && pcPic->reconstructed)
        {
          pcPic->reconstructed = false;
        }
        pcPic->neededForOutput = false;
      }

      iterPic++;
    }
  }
}

/** \param pcListPic list of pictures to be written to file
 */
Void DecApp::xFlushOutput( PicList* pcListPic )
{
  if(!pcListPic || pcListPic->empty())
  {
    return;
  }
  PicList::iterator iterPic   = pcListPic->begin();

  iterPic   = pcListPic->begin();
  Picture* pcPic = *(iterPic);

  if (pcPic->fieldPic ) //Field Decoding
  {
    PicList::iterator endPic   = pcListPic->end();
    endPic--;
    Picture *pcPicTop, *pcPicBottom = NULL;
    while (iterPic != endPic)
    {
      pcPicTop = *(iterPic);
      iterPic++;
      pcPicBottom = *(iterPic);

      if ( pcPicTop->neededForOutput && pcPicBottom->neededForOutput && !(pcPicTop->getPOC()%2) && (pcPicBottom->getPOC() == pcPicTop->getPOC()+1) )
      {
        // write to file
        if ( !m_reconFileName.empty() )
        {
          const Window &conf = pcPicTop->cs->sps->getConformanceWindow();
          const Window  defDisp = (m_respectDefDispWindow && pcPicTop->cs->sps->getVuiParametersPresentFlag()) ? pcPicTop->cs->sps->getVuiParameters()->getDefaultDisplayWindow() : Window();
          const Bool isTff = pcPicTop->topField;
          m_cVideoIOYuvReconFile.write( pcPicTop->getRecoBuf(), pcPicBottom->getRecoBuf(),
                                         m_outputColourSpaceConvert,
                                         conf.getWindowLeftOffset() + defDisp.getWindowLeftOffset(),
                                         conf.getWindowRightOffset() + defDisp.getWindowRightOffset(),
                                         conf.getWindowTopOffset() + defDisp.getWindowTopOffset(),
                                         conf.getWindowBottomOffset() + defDisp.getWindowBottomOffset(), NUM_CHROMA_FORMAT, isTff );
        }

        // update POC of display order
        m_iPOCLastDisplay = pcPicBottom->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if( ! pcPicTop->referenced && pcPicTop->reconstructed )
        {
          pcPicTop->reconstructed = false;
        }
        if( ! pcPicBottom->referenced && pcPicBottom->reconstructed )
        {
          pcPicBottom->reconstructed = false;
        }
        pcPicTop->neededForOutput = false;
        pcPicBottom->neededForOutput = false;

        if(pcPicTop)
        {
          pcPicTop->destroy();
          delete pcPicTop;
          pcPicTop = NULL;
        }
      }
    }
    if(pcPicBottom)
    {
      pcPicBottom->destroy();
      delete pcPicBottom;
      pcPicBottom = NULL;
    }
  }
  else //Frame decoding
  {
    while (iterPic != pcListPic->end())
    {
      pcPic = *(iterPic);

      if (pcPic->neededForOutput)
      {
        // write to file

        if (!m_reconFileName.empty())
        {
          const Window &conf    = pcPic->cs->sps->getConformanceWindow();
          const Window  defDisp = (m_respectDefDispWindow && pcPic->cs->sps->getVuiParametersPresentFlag()) ? pcPic->cs->sps->getVuiParameters()->getDefaultDisplayWindow() : Window();

          m_cVideoIOYuvReconFile.write( pcPic->getRecoBuf(),
                                        m_outputColourSpaceConvert,
                                        conf.getWindowLeftOffset()   + defDisp.getWindowLeftOffset(),
                                        conf.getWindowRightOffset()  + defDisp.getWindowRightOffset(),
                                        conf.getWindowTopOffset()    + defDisp.getWindowTopOffset(),
                                        conf.getWindowBottomOffset() + defDisp.getWindowBottomOffset(),
                                        NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
        }

        if (m_seiMessageFileStream.is_open())
        {
          m_cColourRemapping.outputColourRemapPic (pcPic, m_seiMessageFileStream);
        }

        // update POC of display order
        m_iPOCLastDisplay = pcPic->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if (!pcPic->referenced && pcPic->reconstructed)
        {
          pcPic->reconstructed = false;
        }
        pcPic->neededForOutput = false;
      }
      if(pcPic != NULL)
      {
        pcPic->destroy();
        delete pcPic;
        pcPic = NULL;
      }
      iterPic++;
    }
  }
  pcListPic->clear();
  m_iPOCLastDisplay = -MAX_INT;
}

/** \param nalu Input nalu to check whether its LayerId is within targetDecLayerIdSet
 */
Bool DecApp::isNaluWithinTargetDecLayerIdSet( InputNALUnit* nalu )
{
  if ( m_targetDecLayerIdSet.size() == 0 ) // By default, the set is empty, meaning all LayerIds are allowed
  {
    return true;
  }
  for (std::vector<Int>::iterator it = m_targetDecLayerIdSet.begin(); it != m_targetDecLayerIdSet.end(); it++)
  {
    if ( nalu->m_nuhLayerId == (*it) )
    {
      return true;
    }
  }
  return false;
}

//! \}
