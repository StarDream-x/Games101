# Repo for hw1

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