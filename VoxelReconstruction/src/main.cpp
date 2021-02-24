#include <cstdlib>
#include <string>

#include "utilities/General.h"
#include "VoxelReconstruction.h"
#include "../camera_calibration.cpp"

using namespace nl_uu_science_gmt;

int main(
		int argc, char** argv)
{
	//camera_calibration(argc, argv, "data/", 2);

	//cout << "data" + std::string(PATH_SEP);

	VoxelReconstruction::showKeys();
	VoxelReconstruction vr("data" + std::string(PATH_SEP), 4);
	vr.run(argc, argv);

	return EXIT_SUCCESS;
}
