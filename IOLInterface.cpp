#include "IOLInterface.h"
#include "CaffePrototxtParser.h"
#include "caffe_prototxt_bison.h"
#include <google/protobuf/text_format.h>
#include <mex.h>

namespace iol
{

ifstream OpenInputFileStream(const string& filePath)
{
	ifstream ifs;
	ifs.exceptions(std::ios::failbit);
	try
	{
		ifs.open(filePath);
	}
	catch (exception& e)
	{
		CV_Error(cv::Error::StsBadArg, "Open " + filePath + " failed: " + strerror(errno));
	}

	return ifs;
}

ofstream OpenOutputFileStream(const string& filePath)
{
	ofstream ofs;
	ofs.exceptions(std::ios::failbit);
	try
	{
		ofs.open(filePath);
	}
	catch (exception& e)
	{
		CV_Error(cv::Error::StsBadArg, "Open " + filePath + " failed: " + strerror(errno));
	}

	return ofs;
}

void ParseCaffePrototxt(string prototxtFilePath, ICaffeNetParamTraverser& traverser)
{
	YYParseCaffePrototxt(prototxtFilePath);
	CaffeNetParamDef netParam = gCaffePrototxtParser.GetNetParam();

	netParam.TraverseNet(traverser);
}

mxArray* CreateMxMsgStructVec(const ::google::protobuf::Message* pMessage, int numElms)
{
	const ::google::protobuf::Descriptor* pMsgDesc = pMessage->GetDescriptor();
	const ::google::protobuf::Reflection* pMsgRefl = pMessage->GetReflection();
	int definedFieldCount = pMsgDesc->field_count();
	int fieldCount = 0;

	char** ppFieldNames;
	ppFieldNames = new char *[definedFieldCount];
	for (int iFld = 0; iFld < definedFieldCount; ++iFld)
	{
		const ::google::protobuf::FieldDescriptor* pFieldDescriptor = pMsgDesc->field(iFld);
		string fieldName;

		if (pFieldDescriptor->containing_oneof())
			if (pMsgRefl->HasField(*pMessage, pFieldDescriptor))
				fieldName = pFieldDescriptor->containing_oneof()->name();
			else
				continue;
		else
			fieldName = pFieldDescriptor->name();
		ppFieldNames[fieldCount] = new char[fieldName.size() + 1];
		strcpy(ppFieldNames[fieldCount], fieldName.c_str());
		++fieldCount;
	}

	mxArray* pMsgStructMat = mxCreateStructMatrix(numElms, 1, fieldCount, (const char **)ppFieldNames);

	for (int iFld = 0; iFld < fieldCount; ++iFld)
		delete[] ppFieldNames[iFld];
	delete[] ppFieldNames;

	return pMsgStructMat;
}

void TraversePrototxtMessage(const ::google::protobuf::Message* pMessage, mxArray *pMsgStructMat, int idxToStructArray = 0);

void AddOneField(const ::google::protobuf::Message* pMessage, const ::google::protobuf::FieldDescriptor* pFldDesc, mxArray *pMsgStructMat, int idxToStructArray = 0, bool checkOneof = true)
{
	const ::google::protobuf::Reflection* pMsgRefl = pMessage->GetReflection();

	string fieldName = pFldDesc->name();
	mxArray* pFldVal;
	if (pFldDesc->is_repeated())
	{
		int numRepeat = pMsgRefl->FieldSize(*pMessage, pFldDesc);
		if (numRepeat == 0)
			return;

#define CASE_REPEAT_FIELD_TYPE(cppType, method) \
	case ::google::protobuf::FieldDescriptor::CPPTYPE_##cppType: \
	{ \
		pFldVal = mxCreateDoubleMatrix(numRepeat, 1, mxREAL); \
		double* p = mxGetPr(pFldVal); \
		for (int iRep = 0; iRep < numRepeat; ++iRep) \
			*p++ = (double)pMsgRefl->GetRepeated##method(*pMessage, pFldDesc, iRep); \
		mxSetField(pMsgStructMat, idxToStructArray, fieldName.c_str(), pFldVal); \
		break; \
	}
		switch (pFldDesc->cpp_type())
		{
			CASE_REPEAT_FIELD_TYPE(INT32, Int32)
				CASE_REPEAT_FIELD_TYPE(INT64, Int64)
				CASE_REPEAT_FIELD_TYPE(UINT32, UInt32)
				CASE_REPEAT_FIELD_TYPE(UINT64, UInt64)
				CASE_REPEAT_FIELD_TYPE(FLOAT, Float)
				CASE_REPEAT_FIELD_TYPE(DOUBLE, Double)
		case ::google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
			{
				pFldVal = mxCreateLogicalMatrix(numRepeat, 1);
				unsigned char* p = (unsigned char *)mxGetData(pFldVal);
				for (int iRep = 0; iRep < numRepeat; ++iRep)
					*p++ = (unsigned char)pMsgRefl->GetRepeatedBool(*pMessage, pFldDesc, iRep);
				mxSetField(pMsgStructMat, idxToStructArray, fieldName.c_str(), pFldVal);
				break;
			}
		case ::google::protobuf::FieldDescriptor::CPPTYPE_STRING:
		{
			pFldVal = mxCreateCellMatrix(numRepeat, 1);
			for (int iRep = 0; iRep < numRepeat; ++iRep)
				mxSetCell(pFldVal, iRep, mxCreateString(pMsgRefl->GetRepeatedString(*pMessage, pFldDesc, iRep).c_str()));
			mxSetField(pMsgStructMat, idxToStructArray, fieldName.c_str(), pFldVal);
			break;
		}
		case ::google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
		{
			pFldVal = CreateMxMsgStructVec(&pMsgRefl->GetRepeatedMessage(*pMessage, pFldDesc, 0), numRepeat);
			for (int iRep = 0; iRep < numRepeat; ++iRep)
				TraversePrototxtMessage(&pMsgRefl->GetRepeatedMessage(*pMessage, pFldDesc, iRep), pFldVal, iRep);
			mxSetField(pMsgStructMat, idxToStructArray, fieldName.c_str(), pFldVal);
			break;
		}
		default:
			mexErrMsgTxt("Unrecognized field type");
		}
	}
	else
	{
		if (!pMsgRefl->HasField(*pMessage, pFldDesc))
			return;

		const ::google::protobuf::OneofDescriptor *pOneofDesc = pFldDesc->containing_oneof();
		if (pOneofDesc && checkOneof)
		{
			if (!pMsgRefl->HasOneof(*pMessage, pOneofDesc))
				return;

			const char* ppFldNames[1] = { fieldName.c_str() };
			mxArray* pOneofStruct = mxCreateStructMatrix(1, 1, 1, (const char **)ppFldNames);
			AddOneField(pMessage, pFldDesc, pOneofStruct, idxToStructArray, false);
			mxSetField(pMsgStructMat, 0, pOneofDesc->name().c_str(), pOneofStruct);
			return;
		}

#define CASE_FIELD_TYPE(cppType, method) \
	case ::google::protobuf::FieldDescriptor::CPPTYPE_##cppType: \
	{ \
		pFldVal = mxCreateDoubleScalar((double)pMsgRefl->Get##method(*pMessage, pFldDesc)); \
		mxSetField(pMsgStructMat, 0, fieldName.c_str(), pFldVal); \
		break; \
	}
		switch (pFldDesc->cpp_type())
		{
			CASE_FIELD_TYPE(INT32, Int32)
			CASE_FIELD_TYPE(INT64, Int64)
			CASE_FIELD_TYPE(UINT32, UInt32)
			CASE_FIELD_TYPE(UINT64, UInt64)
			CASE_FIELD_TYPE(FLOAT, Float)
			CASE_FIELD_TYPE(DOUBLE, Double)
		case ::google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
		{
			pFldVal = mxCreateLogicalScalar(pMsgRefl->GetBool(*pMessage, pFldDesc));
			mxSetField(pMsgStructMat, 0, fieldName.c_str(), pFldVal);
			//mexPrintf("%s, %d\n", fieldName.c_str(), pMsgRefl->GetBool(*pMessage, pFldDesc));
			break;
		}
		case ::google::protobuf::FieldDescriptor::CPPTYPE_STRING:
		{
			pFldVal = mxCreateString(pMsgRefl->GetString(*pMessage, pFldDesc).c_str());
			mxSetField(pMsgStructMat, 0, fieldName.c_str(), pFldVal);
			//mexPrintf("%s, %s\n", fieldName.c_str(), pMsgRefl->GetString(*pMessage, pFldDesc).c_str());
			break;
		}
		case ::google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
		{
			pFldVal = mxCreateString(pMsgRefl->GetEnum(*pMessage, pFldDesc)->name().c_str());
			mxSetField(pMsgStructMat, 0, fieldName.c_str(), pFldVal);
			//mexPrintf("%s, %s\n", fieldName.c_str(), pMsgRefl->GetEnum(*pMessage, pFldDesc)->name().c_str());
			break;
		}
		case ::google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
		{
			pFldVal = CreateMxMsgStructVec(&pMsgRefl->GetMessage(*pMessage, pFldDesc), 1);
			TraversePrototxtMessage(&pMsgRefl->GetMessage(*pMessage, pFldDesc), pFldVal);
			mxSetField(pMsgStructMat, 0, fieldName.c_str(), pFldVal);
			//mexPrintf((fieldName + "\n").c_str());
			//mexEvalString("drawnow;");
			break;
		}
		default:
			mexErrMsgTxt("Unrecognized field type");
		}
	}
}

void TraversePrototxtMessage(const ::google::protobuf::Message* pMessage, mxArray *pMsgStructMat, int idxToStructArray)
{

	const ::google::protobuf::Descriptor* pMsgDesc = pMessage->GetDescriptor();
	const ::google::protobuf::Reflection* pMsgRefl = pMessage->GetReflection();
	int fieldCount = pMsgDesc->field_count();

	for (int iFld = 0; iFld < fieldCount; ++iFld)
	{
		const ::google::protobuf::FieldDescriptor* pFldDesc = pMsgDesc->field(iFld);
		AddOneField(pMessage, pFldDesc, pMsgStructMat, idxToStructArray, true);
	}
}

mxArray* ReadPrototxtToMatlab(const string& prototxtFilePath, ReadPrototxtFileFptr pReadPrototxtFileFunc)
{
	CV_Assert(pReadPrototxtFileFunc);

	ifstream prototxtFile = OpenInputFileStream(prototxtFilePath);

	string inputBuffer;
	getline(prototxtFile, inputBuffer, (char)EOF);
	prototxtFile.close();

	cv::Ptr<::google::protobuf::Message> pMessage = pReadPrototxtFileFunc(inputBuffer);
	mxArray* pMxMsgStruct = CreateMxMsgStructVec(pMessage, 1);
	TraversePrototxtMessage(pMessage, pMxMsgStruct);

	return pMxMsgStruct;
}

}