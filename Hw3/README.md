# Repo for Hw3

## FIX TABLE

1. There may be something wrong with the TBN matrix because the annotation shows:
```cpp
// Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
```
However, in this case $t\cdot n\neq0$. We may should modify it into 
```cpp
// Vector t = (x*y/sqrt(x*x+z*z),-sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
```
Futher study is needed.