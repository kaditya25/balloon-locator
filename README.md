# Lab 5

## Cloning the repo
In a directory of your choice (e.g., ''~/Workspace''), do the following:
```bash
git clone https://gitlab.com/todd.humphreys/lab5-aerial-robotics.git
```

## Running the code
In the lab5-aerial-robotics directory, do the following to build the code:
```bash
mkdir build
cd build
cmake ..
make
```

The `locateBalloon` executable, which after the build process is located in the exe directory, 
requires one input: the path to an image to display.  Run the executable
as follows:
```bash
cd exe
./locateBalloon ../images/frame00077.jpg
```

## Resources
- [OpenCV 4.0.1 Documentation](https://docs.opencv.org/4.0.1/)
- [Eigen Documentation](http://eigen.tuxfamily.org/dox/)
