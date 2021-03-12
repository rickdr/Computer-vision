#include <cstdlib>
#include <string>

#include "utilities/General.h"
#include "Assignment3.h"
#include "../camera_calibration.h"
#include "main.h"

using namespace nl_uu_science_gmt;

int main(
		int argc, char** argv)
{	
	//camera_calibration calibration = camera_calibration();
	//camera_calibration::color_model("data/", 4);
	//calibration.calibration(argc, argv, "data/", 2)

	Assignment3::showKeys();
	Assignment3 vr("data" + std::string(PATH_SEP), 4);
	vr.run(argc, argv);

	return EXIT_SUCCESS;
}