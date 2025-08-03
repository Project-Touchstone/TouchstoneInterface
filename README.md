** Dependencies: **
- Eigen
	- Unzip the Eigen folder somewhere on your computer.
	- Create build folder in the Eigen folder.
	- Open a terminal in the Eigen build folder.
	- Run the following commands:
		```bash
		cmake ..
		cmake --build .
		'''
	- Create the following environment variable:
		EIGEN3_INCLUDE_DIR: <path_to_eigen>/Eigen
- Boost
	- Unzip the Boost folder somewhere on your computer.
	- Open a terminal in the boost version folder.
	- Run the following commands:
		```bash
		./bootstrap.bat
		./b2
		b2 install --prefix=<boost folder>
		```
	- Delete the original boost version folder