#include "exporter.h"

#pragma warning (disable: 4996)

namespace output
{
	int Exporter::pictureCounter = 0;
	char* Exporter::targetFolder = "./Temp/";

	Exporter::~Exporter() {}

	int Exporter::GetPictureCounter()
	{
		return pictureCounter;
	}

	void Exporter::Reset()
	{
		pictureCounter = 0;
	}

	void Exporter::OutputScreenShot(const int screenWidth, const int screenHeight)
	{
		if (pictureCounter == 0)
		{
			system("mkdir Screenshots");
			system("del /Q Screenshots");
		}

		gfx::Texture Picture(screenWidth, screenHeight);

		glReadPixels(0, 0, screenWidth, screenHeight, GL_RGB, GL_UNSIGNED_BYTE, Picture.getData());

		static const char *s_psFILE_NAME = "Image%03d.bmp";
		std::string PictureName;
		PictureName.assign("./Screenshots/");
		PictureName.append(s_psFILE_NAME);

		static const int g_lPICTURE_FILE_NAME_LENGTH = 64;
		char g_sPictureName[g_lPICTURE_FILE_NAME_LENGTH] = "\0";

		sprintf(g_sPictureName, PictureName.c_str(), pictureCounter);

		Picture.saveAsFile(g_sPictureName);
		++pictureCounter;
	}
}