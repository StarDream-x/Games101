# Repo for Games101

This repo is my solutions for games101.

## Getting Start

1. Open the corresponding hw folder
2. create build folder with `mkdir build`
3. open build folder with `cd build`
4. run cmake with `cmake ..`
5. make it with `make [-j4]`, here 4 means use 4 cores to run it
6. run the project with `./projectName`, here projectName is the corresponding project's name

## TODO
- [x] Hw0
- [x] Hw1
- [x] Hw2
- [ ] Hw3
- [ ] Hw4
- [ ] Hw5
- [ ] Hw6
- [ ] Hw7
- [ ] Hw8

## FIX TABLE

1. For the bonus part of HW1, we can sometimes get segment fault, which is caused by an ERROR in `set_pixel()`, `rasterizer.cpp`. The original code is shown below.
```cpp
if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height) return;
auto ind = (height-point.y())*width + point.x();
```
Due to `point.y() == 0` can happen, which will cause `ind > w*h` and segment fault, the following modification is neccessary. 
```cpp
auto ind = (height-point.y()-1)*width + point.x();
```