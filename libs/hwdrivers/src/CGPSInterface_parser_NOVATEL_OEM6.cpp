/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/utils/CMemoryStream.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace std;

MRPT_TODO("check crc")

bool  CGPSInterface::implement_parser_NOVATEL_OEM6(size_t &out_minimum_rx_buf_to_decide)
{
	using namespace mrpt::obs::gnss;

	out_minimum_rx_buf_to_decide = sizeof(nv_oem6_short_header_t);

	const size_t nBytesAval = m_rx_buffer.size();  // Available for read
	if (nBytesAval<out_minimum_rx_buf_to_decide) 
		return true; // no need to skip 1 byte

	// If the synch bytes do not match, it is not a valid frame:
	uint8_t peek_buffer[3];
	m_rx_buffer.peek_many(&peek_buffer[0],3);
	// Short header?
	const bool is_short_hdr = 
		peek_buffer[0]==nv_oem6_short_header_t::SYNCH0 && 
		peek_buffer[1]==nv_oem6_short_header_t::SYNCH1 && 
		peek_buffer[2]==nv_oem6_short_header_t::SYNCH2;

	const bool is_regular_hdr = 
		peek_buffer[0]==nv_oem6_header_t::SYNCH0 && 
		peek_buffer[1]==nv_oem6_header_t::SYNCH1 && 
		peek_buffer[2]==nv_oem6_header_t::SYNCH2;
			
	if (!is_short_hdr && !is_regular_hdr)
		return false; // skip 1 byte, we dont recognize this format

	if (is_short_hdr)
	{
		if (nBytesAval<sizeof(nv_oem6_short_header_t)) {
			out_minimum_rx_buf_to_decide = sizeof(nv_oem6_short_header_t);
			return true; // we must wait for more data in the buffer
		}
		nv_oem6_short_header_t hdr;
		m_rx_buffer.peek_many(reinterpret_cast<uint8_t*>(&hdr), sizeof(hdr));
		const uint32_t expected_total_msg_len = sizeof(hdr) + hdr.msg_len + 4 /*crc*/;
		if (nBytesAval<expected_total_msg_len) {
			out_minimum_rx_buf_to_decide = expected_total_msg_len;
			return true; // we must wait for more data in the buffer
		}

		std::vector<uint8_t> buf(expected_total_msg_len);
		m_rx_buffer.pop_many(reinterpret_cast<uint8_t*>(&buf[0]), sizeof(hdr));
		m_rx_buffer.pop_many(reinterpret_cast<uint8_t*>(&buf[sizeof(hdr)]), hdr.msg_len + 4 /*crc*/ );

		// Deserialize the message:
		// 1st, test if we have a specific data structure for this msg_id:
		bool use_generic_container;
		{
			gnss_message* dummy = gnss_message::Factory( (gnss_message_type_t)(NV_OEM6_MSG2ENUM + hdr.msg_id ) );
			use_generic_container = (dummy==NULL);
			delete dummy;
		}
		// ------ Serialization format:
		//const int32_t msg_id = message_type;
		//out << msg_id;
		//this->internal_writeToStream(out);  == >  out << static_cast<uint32_t>(DATA_LEN); out.WriteBuffer(DATA_PTR,DATA_LEN); }
		// ------
		mrpt::utils::CMemoryStream tmpStream;
		const uint32_t msg_id = use_generic_container ? 
			(uint32_t)(NV_OEM6_GENERIC_SHORT_FRAME)
			: 
			(uint32_t) hdr.msg_id+NV_OEM6_MSG2ENUM;
		tmpStream << (uint32_t )(msg_id);
		tmpStream << (uint32_t)(expected_total_msg_len);  // This len = hdr + hdr.msg_len + 4 (crc);
		tmpStream.WriteBuffer(&buf[0],buf.size());

		tmpStream.Seek(0);
		gnss_message_ptr msg( gnss_message::readAndBuildFromStream(tmpStream) );
		if (!msg.get()) {
			std::cerr << "[CGPSInterface::implement_parser_NOVATEL_OEM6] Error parsing binary packet msg_id="<< hdr.msg_id<<"\n";
			return true;
		}
		m_just_parsed_messages.messages[msg->message_type] = msg;
		m_just_parsed_messages.originalReceivedTimestamp = mrpt::system::now();
		
		MRPT_TODO("check UTC time conversion!")
		if (!CObservationGPS::GPS_time_to_UTC(hdr.week,hdr.ms_in_week*1e-3,m_just_parsed_messages.timestamp))
			m_just_parsed_messages.timestamp =  mrpt::system::now();

		m_just_parsed_messages.sensorLabel = msg->getMessageTypeAsString();

		flushParsedMessagesNow();
		return true;
	} // end short hdr

	if (is_regular_hdr)
	{
		if (nBytesAval<sizeof(nv_oem6_header_t)) {
			out_minimum_rx_buf_to_decide = sizeof(nv_oem6_header_t);
			return true; // we must wait for more data in the buffer
		}
		nv_oem6_header_t hdr;
		m_rx_buffer.peek_many(reinterpret_cast<uint8_t*>(&hdr), sizeof(hdr));
		const uint32_t expected_total_msg_len = sizeof(hdr) + hdr.msg_len + 4 /*crc*/;
		if (nBytesAval<expected_total_msg_len)
		{
			out_minimum_rx_buf_to_decide = expected_total_msg_len;
			return true; // we must wait for more data in the buffer
		}

		std::vector<uint8_t> buf(expected_total_msg_len);
		m_rx_buffer.pop_many(reinterpret_cast<uint8_t*>(&buf[0]), sizeof(hdr));
		m_rx_buffer.pop_many(reinterpret_cast<uint8_t*>(&buf[sizeof(hdr)]), hdr.msg_len + 4 /*crc*/ );

		// Deserialize the message:
		// 1st, test if we have a specific data structure for this msg_id:
		bool use_generic_container;
		{
			gnss_message* dummy = gnss_message::Factory( (gnss_message_type_t)(NV_OEM6_MSG2ENUM + hdr.msg_id ) );
			use_generic_container = (dummy==NULL);
			delete dummy;
		}
		// ------ Serialization format:
		//const int32_t msg_id = message_type;
		//out << msg_id;
		//this->internal_writeToStream(out);  == >  out << static_cast<uint32_t>(DATA_LEN); out.WriteBuffer(DATA_PTR,DATA_LEN); }
		// ------
		mrpt::utils::CMemoryStream tmpStream;
		const int32_t msg_id = use_generic_container ? 
			(uint32_t)(NV_OEM6_GENERIC_FRAME)
			: 
			(uint32_t) hdr.msg_id+NV_OEM6_MSG2ENUM;
		tmpStream << msg_id;
		tmpStream << (uint32_t)(expected_total_msg_len);
		tmpStream.WriteBuffer(&buf[0],buf.size());

		tmpStream.Seek(0);
		gnss_message_ptr msg( gnss_message::readAndBuildFromStream(tmpStream) );
		if (!msg.get()) {
			std::cerr << "[CGPSInterface::implement_parser_NOVATEL_OEM6] Error parsing binary packet msg_id="<< hdr.msg_id<<"\n";
			return true;
		}
		m_just_parsed_messages.messages[msg->message_type] = msg;
		m_just_parsed_messages.originalReceivedTimestamp = mrpt::system::now();
		MRPT_TODO("check UTC time conversion!")
		if (!CObservationGPS::GPS_time_to_UTC(hdr.week,hdr.ms_in_week*1e-3,m_just_parsed_messages.timestamp))
			m_just_parsed_messages.timestamp =  mrpt::system::now();
		m_just_parsed_messages.sensorLabel = msg->getMessageTypeAsString();
		flushParsedMessagesNow();
		return true;
	} // end regular hdr

	// Shouldnt arrive here, but MSVC complies anyway:
	return false;
}
