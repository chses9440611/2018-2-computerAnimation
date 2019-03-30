#pragma once

#include <string>

#include "glad/glad.h"
#include "gfx.h"

namespace output
{
	// static usage only
	class Exporter final
	{
	private:
		Exporter() = delete;
		~Exporter();

		static int pictureCounter;
		static char* targetFolder;
	public:

		static int GetPictureCounter();

		static void Reset();
		//static void OutputVideo(const char* fileNamePrefix);
		static void OutputScreenShot(const int screenWidth, const int screenHeight);
	};
}