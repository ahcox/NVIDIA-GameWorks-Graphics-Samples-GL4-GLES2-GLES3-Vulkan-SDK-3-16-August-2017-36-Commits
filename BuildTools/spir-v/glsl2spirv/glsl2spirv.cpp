// glsl2spirv.cpp : Defines the entry point for the console application.
//

#ifdef _WIN32
#include <Windows.h>
#include <tchar.h>
#include <Shlwapi.h>
#define NVCHAR wchar_t
#define STRING_TAG "%ls"
#else
#include <string.h>
#define NVCHAR char
#define _strnicmp strncasecmp
#define _snprintf_s snprintf
#define INVALID_HANDLE_VALUE NULL
#define HANDLE FILE*
#define STRING_TAG "%s"
#define MAX_PATH 1024
#endif

#include <stdlib.h>
#include <stdio.h>
#include "SimpleOpt.h"
#include "../include/shaderc/shaderc.hpp"
#include "vulkan/vulkan.h"
#include <cstdio>
#include <iostream>

char* vsInfile = NULL;
char* fsInfile = NULL;
char* gsInfile = NULL;
char* tcsInfile = NULL;
char* tesInfile = NULL;
char* csInfile = NULL;

const char VS_TAG[] = "GLSL_VS";
const char FS_TAG[] = "GLSL_FS";
const char GS_TAG[] = "GLSL_GS";
const char TCS_TAG[] = "GLSL_TCS";
const char TES_TAG[] = "GLSL_TES";
const char CS_TAG[] = "GLSL_CS";

enum ECommandLineIDs
{
	CMDLN_HELP,
	CMDLN_OUTFILE,
	CMDLINE_VS_INFILE,
	CMDLINE_FS_INFILE,
	CMDLINE_GS_INFILE,
	CMDLINE_TCS_INFILE,
	CMDLINE_TES_INFILE,
	CMDLINE_CS_INFILE,
	CMDLINE_OUTPUT_CPP
};

#ifdef _WIN32
CSimpleOptW::SOption rgOptions[] = 
{
	{ CMDLN_HELP, L"-h", SO_NONE },
	{ CMDLN_OUTFILE, L"-o", SO_REQ_SEP },
	{ CMDLINE_VS_INFILE, L"-vs", SO_REQ_SEP },
	{ CMDLINE_FS_INFILE, L"-fs", SO_REQ_SEP },
	{ CMDLINE_GS_INFILE, L"-gs", SO_REQ_SEP },
	{ CMDLINE_TCS_INFILE, L"-tcs", SO_REQ_SEP },
	{ CMDLINE_TES_INFILE, L"-tes", SO_REQ_SEP },
	{ CMDLINE_CS_INFILE, L"-cs", SO_REQ_SEP },
	{ CMDLINE_OUTPUT_CPP, L"-cpp", SO_REQ_SEP },
	SO_END_OF_OPTIONS
};
#else
CSimpleOptA::SOption rgOptions[] = 
{
	{ CMDLN_HELP, "-h", SO_NONE },
	{ CMDLN_OUTFILE, "-o", SO_REQ_SEP },
	{ CMDLINE_VS_INFILE, "-vs", SO_REQ_SEP },
	{ CMDLINE_FS_INFILE, "-fs", SO_REQ_SEP },
	{ CMDLINE_GS_INFILE, "-gs", SO_REQ_SEP },
	{ CMDLINE_TCS_INFILE, "-tcs", SO_REQ_SEP },
	{ CMDLINE_TES_INFILE, "-tes", SO_REQ_SEP },
	{ CMDLINE_CS_INFILE, "-cs", SO_REQ_SEP },
	{ CMDLINE_OUTPUT_CPP, "-cpp", SO_REQ_SEP },
	SO_END_OF_OPTIONS
};
#endif

void ConvertPathSlashes(NVCHAR* str) {
#ifdef _WIN32
	while (*str) {
		if (*str == '/')
			*str = '\\';
		str++;
	}
#endif
}

char* LoadFileToString(NVCHAR* filename) {
	ConvertPathSlashes(filename);

#ifdef _WIN32
	HANDLE fp = CreateFileW(filename,
		GENERIC_READ,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		NULL);

	if (fp == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "Fatal Error: could not open shader file %ls\n", filename);
		return NULL;
	}

	LARGE_INTEGER size;
	if (!GetFileSizeEx(fp, &size)) {
		fprintf(stderr, "Fatal Error: could not get shader file size %ls\n", filename);
		return NULL;
	}

	if (size.HighPart > 0) {
		fprintf(stderr, "Fatal Error: shader file is >2GB! %ls\n", filename);
		return NULL;
	}

	char* text = new char[size.LowPart + 1];

	if (!text) {
		fprintf(stderr, "Fatal Error: could not allocate memory for shader file %ls\n", filename);
		return NULL;
	}

	DWORD read = 0;
	if (!ReadFile(fp, text, size.LowPart, &read, NULL)) {
		fprintf(stderr, "Fatal Error: could not read shader file %ls\n", filename);
		return NULL;
	}

	CloseHandle(fp);

	if (read > size.LowPart) read = size.LowPart;

	text[read] = '\0';

	return text;
#else
	FILE* fp = fopen(filename,"r");

        if (!fp) {
		fprintf(stderr, "Fatal Error: could not open shader file %s\n", filename);
		return NULL;
	}
	
	fseek(fp, 0L, SEEK_END);
	size_t filesize = ftell(fp);
	fseek(fp, 0L, SEEK_SET);

	char* text = new char[filesize + 1];

	if (!text) {
		fprintf(stderr, "Fatal Error: could not allocate memory for shader file %s\n", filename);
		return NULL;
	}

	size_t read = fread(text, 1, filesize, fp);
	if (read < filesize) {
		fprintf(stderr, "Fatal Error: could not read shader file %s\n", filename);
		return NULL;
	}

	fclose(fp);

	if (read > filesize) read = filesize;

	text[filesize] = '\0';

	return text;
#endif
}


bool ParseCombinedFile(NVCHAR* filename) {
	ConvertPathSlashes(filename);
	char* fileText = LoadFileToString(filename);

	if (!fileText)
		return false;

	// walk the file, looking for tags.  when we find a tag, change the hash to a null char
	// and grab the pointer to the start of the next line for the desired shader
	char* ptr = fileText;
	while (ptr[0]) {
		while (ptr[0] && ptr[0] != '#') {
			ptr++;
		}

		if (ptr[0] == '#') {
			bool closeOut = true;
			char* hash = ptr;
			char* tag = ptr + 1;
			char** shaderPtrAddr = NULL;
			// found hash.  But is it a shader tag?
			if (!_strnicmp(tag, VS_TAG, strlen(VS_TAG))) {
				shaderPtrAddr = &vsInfile;
			}
			else if (!_strnicmp(tag, FS_TAG, strlen(FS_TAG))) {
				shaderPtrAddr = &fsInfile;
			}
			else if (!_strnicmp(tag, GS_TAG, strlen(GS_TAG))) {
				shaderPtrAddr = &gsInfile;
			}
			else if (!_strnicmp(tag, TCS_TAG, strlen(TCS_TAG))) {
				shaderPtrAddr = &tcsInfile;
			}
			else if (!_strnicmp(tag, TES_TAG, strlen(TES_TAG))) {
				shaderPtrAddr = &tesInfile;
			}
			else if (!_strnicmp(tag, CS_TAG, strlen(CS_TAG))) {
				shaderPtrAddr = &csInfile;
			}

			if (closeOut) {
				// advance the ptr to the end of the line or the end of the string, whichever is first
				while (ptr[0] && ptr[0] != '\n' && ptr[0] != '\r')
					ptr++;

				if (shaderPtrAddr)
					*shaderPtrAddr = ptr;

				// if it is a valid tag, then null out the leading # to close the previous shader
				if (shaderPtrAddr)
					hash[0] = '\0';
			}
		}
	}


	return true;
}

// Compiles a shader to a SPIR-V binary. Returns the binary as
// a vector of 32-bit words.
std::vector<uint32_t> compile_file(const std::string& source_name,
	shaderc_shader_kind kind,
	const std::string& source) {
	shaderc::Compiler compiler;
	shaderc::CompileOptions options;

	shaderc::SpvCompilationResult module = compiler.CompileGlslToSpv(
		source.c_str(), source.size(), kind, source_name.c_str(), options);

	if (module.GetCompilationStatus() != shaderc_compilation_status_success) {
		fprintf(stderr, "Shader compile error: %s\n", module.GetErrorMessage().c_str());
		return std::vector<uint32_t>();
	}

	std::vector<uint32_t> result(module.cbegin(), module.cend());
	return result;
}

const int MAX_CHARS = 2048;
HANDLE binaryFp = INVALID_HANDLE_VALUE;

NVCHAR* binaryOutfileName = NULL;

int32_t openBinaryFile(NVCHAR* outfile) {
	ConvertPathSlashes(outfile);
#ifdef _WIN32
	binaryFp  = CreateFileW(outfile,
		GENERIC_WRITE,
		0,
		NULL,
		CREATE_ALWAYS,
		FILE_ATTRIBUTE_NORMAL,
		NULL);
#else
	binaryFp  = fopen(outfile, "w");
#endif

	binaryOutfileName = outfile;

	if (binaryFp == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "Fatal Error: could not open output file " STRING_TAG "\n", outfile);
		return -1;
	}

	return 0;
}

void closeBinaryFile() {
#ifdef _WIN32
	CloseHandle(binaryFp);
#else
	fclose(binaryFp);
#endif
}


int32_t  WriteFileWrapper(HANDLE fp, const unsigned char* str, uint32_t length) {
#ifdef _WIN32
	DWORD written = 0;
	if (!WriteFile(fp, str, length, &written, NULL)) {
		return 0;
	}
#else
        int32_t written = fwrite(str, 1, length, fp);
        if (written < 0) {
                return 0;
        }
#endif
    return (int32_t)written;
}

int32_t writeBinary(uint32_t size, const uint8_t* data) {
	int32_t written = 0;	
        written = WriteFileWrapper(binaryFp, data, size);
	if (written <= 0) {
		fprintf(stderr, "Fatal Error: could not write output file " STRING_TAG "\n", binaryOutfileName);
		return -1;
	}

	if (written != size) {
		fprintf(stderr, "Fatal Error: could not write output (expected %d, wrote %d)\n",
			size, written);
		return -1;
	}

	return 0;
}

HANDLE hexHeaderFp = INVALID_HANDLE_VALUE;
HANDLE hexSourceFp = INVALID_HANDLE_VALUE;
NVCHAR headerFilename[MAX_CHARS];
NVCHAR sourceFilename[MAX_CHARS];

int32_t openHexfiles(NVCHAR* filename, NVCHAR* prefix, uint32_t totalSize) {
	ConvertPathSlashes(filename);

	char mbPrefix[MAX_CHARS];
	char str[MAX_CHARS];

#ifdef _WIN32
	wcstombs_s(NULL, mbPrefix, prefix, MAX_CHARS);

	wnsprintf(headerFilename, MAX_CHARS, L"%s.h", filename);
	wnsprintf(sourceFilename, MAX_CHARS, L"%s.cpp", filename);

	fprintf(stderr, "headerFilename %ls\n", headerFilename);
	fprintf(stderr, "sourceFilename %ls\n", sourceFilename);

	hexHeaderFp = CreateFileW(headerFilename,
		GENERIC_WRITE,
		0,
		NULL,
		CREATE_ALWAYS,
		FILE_ATTRIBUTE_NORMAL,
		NULL);

	hexSourceFp = CreateFileW(sourceFilename,
		GENERIC_WRITE,
		0,
		NULL,
		CREATE_ALWAYS,
		FILE_ATTRIBUTE_NORMAL,
		NULL);
#else
	strcpy(mbPrefix, prefix);

	sprintf(headerFilename, "%s.h", filename);
	sprintf(sourceFilename, "%s.cpp", filename);

	fprintf(stderr, "headerFilename %s\n", headerFilename);
	fprintf(stderr, "sourceFilename %s\n", sourceFilename);

	hexHeaderFp = fopen(headerFilename, "w");
	hexSourceFp = fopen(sourceFilename, "w");
#endif

	if (hexHeaderFp == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "Fatal Error: could not open output file " STRING_TAG "\n", headerFilename);
		return -1;
	}

	if (hexSourceFp == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "Fatal Error: could not open output file " STRING_TAG "\n", sourceFilename);
		return -1;
	}

	_snprintf_s(str, MAX_CHARS, "extern const int %sLength;\n", mbPrefix);
	int length = strlen(str);

	if (!WriteFileWrapper(hexHeaderFp, (const unsigned char*)str, length)) {
		fprintf(stderr, "Fatal Error: could not write output file " STRING_TAG "\n", headerFilename);
		return -1;
	}

	_snprintf_s(str, MAX_CHARS, "extern const unsigned char %sData[];\n", mbPrefix);
	length = strlen(str);

	if (!WriteFileWrapper(hexHeaderFp, (const unsigned char*)str, length)) {
		fprintf(stderr, "Fatal Error: could not write output file " STRING_TAG "\n", headerFilename);
		return -1;
	}

#ifdef _WIN32
	CloseHandle(hexHeaderFp);
#else
	fclose(hexHeaderFp);
#endif

	_snprintf_s(str, MAX_CHARS, "const int %sLength = %d;\n", mbPrefix, totalSize);
	length = strlen(str);

	if (!WriteFileWrapper(hexSourceFp, (const unsigned char*)str, length)) {
		fprintf(stderr, "Fatal Error: could not write output file " STRING_TAG "\n", sourceFilename);
		return -1;
	}

	_snprintf_s(str, MAX_CHARS, "const unsigned char %sData[] = {\n", mbPrefix);
	length = strlen(str);

	if (!WriteFileWrapper(hexSourceFp, (const unsigned char*)str, length)) {
		fprintf(stderr, "Fatal Error: could not write output file " STRING_TAG "\n", sourceFilename);
		return -1;
	}

	return 0;
}

int32_t writeHexcode(uint32_t size, const uint8_t* data) {
	char str[MAX_CHARS];
	int32_t length;

	int remaining = size;
	while (remaining >= 8) {
		_snprintf_s(str, MAX_CHARS, "0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x,\n",
			(unsigned int)data[0], (unsigned int)data[1], (unsigned int)data[2], (unsigned int)data[3],
			(unsigned int)data[4], (unsigned int)data[5], (unsigned int)data[6], (unsigned int)data[7]);
		length = strlen(str);

		if (!WriteFileWrapper(hexSourceFp, (const unsigned char*)str, length)) {
			fprintf(stderr, "Fatal Error: could not write output file " STRING_TAG "\n", sourceFilename);
			return -1;
		}

		remaining -= 8;
		data += 8;
	}

	for (int i = 0; i < remaining; i++) {
		_snprintf_s(str, MAX_CHARS, "0x%02x, ",
			(unsigned int)data[i]);
		length = strlen(str);

		if (!WriteFileWrapper(hexSourceFp, (const unsigned char*)str, length)) {
			fprintf(stderr, "Fatal Error: could not write output file " STRING_TAG "\n", sourceFilename);
			return -1;
		}

	}

	return 0;
}


int32_t closeHexfiles() {
	const int MAX_CHARS = 2048;
	char str[MAX_CHARS];
	int32_t length;

	_snprintf_s(str, MAX_CHARS, "\n};\n");
	length = strlen(str);

	if (!WriteFileWrapper(hexSourceFp, (const unsigned char*)str, length)) {
		fprintf(stderr, "Fatal Error: could not write output file " STRING_TAG "\n", sourceFilename);
		return -1;
	}

#ifdef _WIN32
	CloseHandle(hexSourceFp);
#else
	fclose(hexSourceFp);
#endif

	return 0;
}

static void PrintUsage(const NVCHAR *appName)
{
	fwprintf(stdout, L"Usage: %s -o fileName [Options] [Combined Shader File]\n", appName);
	fwprintf(stdout, L"\n");
	fwprintf(stdout, L"-o fileName        : Specify output binary file name\n");
	fwprintf(stdout, L"-vs fileName       : Vertex Shader\n");
	fwprintf(stdout, L"-fs fileName       : Fragment Shader\n");
	fwprintf(stdout, L"-gs fileName       : Geometry Shader\n");
	fwprintf(stdout, L"-tcs fileName      : Tessellation Control Shader\n");
	fwprintf(stdout, L"-tes fileName      : Tessellation Evaluation Shader\n");
	fwprintf(stdout, L"-cs fileName       : Compute Shader\n");
	fwprintf(stdout, L"-cpp namePrefix    : Output in C-source using given name prefix\n");
}

#ifdef _WIN32
int wmain(int argc, NVCHAR* argv[])
#else
int main(int argc, NVCHAR* argv[])
#endif
{
	const int MAX_STAGES = 6;

	NVCHAR* outfile = NULL;
	NVCHAR* writeCSource = NULL;

	bool showHelp = false;

#ifdef _WIN32
	CSimpleOptW args(argc, argv, rgOptions);
#else
	CSimpleOptA args(argc, argv, rgOptions);
#endif
	args.SetFlags(SO_O_EXACT);

	while (args.Next())
	{
		if (args.LastError() == SO_SUCCESS)
		{
			switch (args.OptionId())
			{
			case CMDLN_HELP:
				showHelp = true;

				break;

			case CMDLN_OUTFILE:
				outfile = args.OptionArg();

				break;
			case CMDLINE_VS_INFILE:
				vsInfile = LoadFileToString(args.OptionArg());
				if (!vsInfile)
					return -1;

				break;

			case CMDLINE_FS_INFILE:
				fsInfile = LoadFileToString(args.OptionArg());
				if (!fsInfile)
					return -1;

				break;

			case CMDLINE_GS_INFILE:
				gsInfile = LoadFileToString(args.OptionArg());
				if (!gsInfile)
					return -1;

				break;

			case CMDLINE_TCS_INFILE:
				tcsInfile = LoadFileToString(args.OptionArg());
				if (!tcsInfile)
					return -1;

				break;

			case CMDLINE_TES_INFILE:
				tesInfile = LoadFileToString(args.OptionArg());
				if (!tesInfile)
					return -1;

				break;

			case CMDLINE_CS_INFILE:
				csInfile = LoadFileToString(args.OptionArg());
				if (!csInfile)
					return -1;

				break;

			case CMDLINE_OUTPUT_CPP:
				writeCSource = args.OptionArg();

				break;

			};
		}
	}

	if (showHelp) {
		PrintUsage(argv[0]);
		return 0;
	}

	const int fileCount = args.FileCount();

	char** includePaths = new char*[fileCount];

	for (int i = 0; i < fileCount; i++) {
		if (!ParseCombinedFile(args.File(i)))
			return -1;

		char* path = new char[2 * MAX_PATH];
		NVCHAR* filenamePtr = NULL;

#ifdef _WIN32
		NVCHAR wPath[MAX_PATH];
		wPath[0] = 0;
		GetFullPathName(args.File(i), MAX_PATH, wPath, &filenamePtr);
		// get rid of the filename...
		if (filenamePtr)
			*filenamePtr = 0;
		wcstombs_s(NULL, path, 2 * MAX_PATH, wPath, MAX_PATH);
#else
		realpath(args.File(i), path);
		// get rid of the filename...
//		if (filenamePtr)
//			*filenamePtr = 0;
#endif
		includePaths[i] = path;
	}

	if (fileCount == 0 &&
		vsInfile == NULL &&
		fsInfile == NULL &&
		gsInfile == NULL &&
		tcsInfile == NULL &&
		tesInfile == NULL &&
		csInfile == NULL) {
		fwprintf(stderr, L"Error: No input file(s) are specified.\n");
		PrintUsage(argv[0]);
		return -1;
	}

	if (outfile == NULL) {
		fwprintf(stderr, L"Error: No output file is specified.\n");
		PrintUsage(argv[0]);
		return -1;
	}

	struct ShaderStage {
		char* source;
		shaderc_shader_kind stage;
		VkShaderStageFlagBits stageflag;
		std::vector<uint32_t> binary;
	};
	ShaderStage* stages = new ShaderStage[MAX_STAGES];
	uint32_t count = 0;

	if (vsInfile) {
		stages[count].source = vsInfile;
		stages[count].stage = shaderc_glsl_vertex_shader;
		stages[count].stageflag = VK_SHADER_STAGE_VERTEX_BIT;
		count++;
	}

	if (fsInfile) {
		stages[count].source = fsInfile;
		stages[count].stage = shaderc_glsl_fragment_shader;
		stages[count].stageflag = VK_SHADER_STAGE_FRAGMENT_BIT;
		count++;
	}

	if (gsInfile) {
		stages[count].source = gsInfile;
		stages[count].stage = shaderc_glsl_geometry_shader;
		stages[count].stageflag = VK_SHADER_STAGE_GEOMETRY_BIT;
		count++;
	}

	if (tcsInfile) {
		stages[count].source = tcsInfile;
		stages[count].stage = shaderc_glsl_tess_control_shader;
		stages[count].stageflag = VK_SHADER_STAGE_TESSELLATION_CONTROL_BIT;
		count++;
	}

	if (tesInfile) {
		stages[count].source = tesInfile;
		stages[count].stage = shaderc_glsl_tess_evaluation_shader;
		stages[count].stageflag = VK_SHADER_STAGE_TESSELLATION_EVALUATION_BIT;
		count++;
	}

	if (csInfile) {
		stages[count].source = csInfile;
		stages[count].stage = shaderc_glsl_compute_shader;
		stages[count].stageflag = VK_SHADER_STAGE_COMPUTE_BIT;
		count++;
	}

	for (uint32_t i = 0; i < count; i++) {
		ShaderStage& stage = stages[i];
		stage.binary = compile_file("shader", stage.stage, stage.source);
		if (stage.binary.size() == 0)
			return -1;
	}

	struct StageTableEntry {
		uint32_t offset;
		uint32_t size;
		uint32_t kind;
	};

	StageTableEntry* stageTable = new StageTableEntry[count];

	// header + count + table
	uint32_t offset = 8 + 4 + count * sizeof(StageTableEntry);
	for (uint32_t i = 0; i < count; i++) {
		StageTableEntry& stage = stageTable[i];

		stage.offset = offset;
		stage.size = stages[i].binary.size() * 4;
		offset += stage.size;
		stage.kind = stages[i].stageflag;
	}

	int32_t(*writeData)(uint32_t size, const uint8_t* data) = NULL;

	if (writeCSource) {
		writeData = writeHexcode;

		if (openHexfiles(outfile, writeCSource, stageTable[count - 1].offset + stageTable[count - 1].size))
			return -1;
	} else {
		writeData = writeBinary;

			// Write our binary file:
			// NVSPRV00
			// <32bit stage count>
			// <32bit stage offset><32bit stage size><32bit stage flag>
			if (openBinaryFile(outfile))
				return -1;
	}

	const uint32_t headerLen = 8;
	const uint8_t header[] = "NVSPRV00";

	if (writeData(headerLen, header))
		return -1;

	if (writeData(sizeof(count), (uint8_t*)&count))
		return -1;

	if (writeData(count*sizeof(StageTableEntry), (uint8_t*)stageTable))
		return -1;

	for (uint32_t i = 0; i < count; i++) {
		if (writeData(stages[i].binary.size() * 4, (uint8_t*)(stages[i].binary.data())))
			return -1;
	}

	if (writeCSource) {
		closeHexfiles();
	} else	{
		closeBinaryFile();
	}

	return 0;
}

