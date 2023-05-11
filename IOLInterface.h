#ifndef __IOL_INTERFACE_H__
#define __IOL_INTERFACE_H__

#include "CaffePrototxtParserInterface.h"

namespace google
{
	namespace protobuf
	{
		enum LogLevel;

		class Message;
		typedef void LogHandler(LogLevel level, const char* filename, int line, const std::string& message);
	}
}

typedef struct mxArray_tag mxArray;


namespace iol
{

inline void ProtobufErrorHandler(::google::protobuf::LogLevel level, const char* filename, int line, const std::string& message)
{
	if (level >= 2) // LOGLEVEL_ERROR
		CV_Error(cv::Error::StsParseError, message);
}

#define DEF_READ_PROTOTXT_FUNC(MessageClass) \
	cv::Ptr<::google::protobuf::Message> ReadPrototxtFile(const string& protoxFileBuffer) \
	{ \
		::google::protobuf::SetLogHandler((::google::protobuf::LogHandler*)iol::ProtobufErrorHandler); \
		::google::protobuf::Message* pMessage = new MessageClass; \
		::google::protobuf::TextFormat::ParseFromString(protoxFileBuffer, pMessage); \
		return pMessage; \
	}

typedef cv::Ptr<::google::protobuf::Message> (* ReadPrototxtFileFptr)(const string& protoxFileBuffer);

IOL_EXPORTS ifstream OpenInputFileStream(const string& filePath);
IOL_EXPORTS ofstream OpenOutputFileStream(const string& filePath);
IOL_EXPORTS void ParseCaffePrototxt(string prototxtFilePath, ICaffeNetParamTraverser& traverser);
IOL_EXPORTS mxArray* ReadPrototxtToMatlab(const string& prototxtFilePath, ReadPrototxtFileFptr pReadPrototxtFileFunc = (ReadPrototxtFileFptr)nullptr);
}


#endif // __IOL_INTERFACE_H__