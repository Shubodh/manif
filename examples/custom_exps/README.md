## Run:

```
cmake .                                                                            
make                                              
./manifCustom
```                   

```
python3 plots.py <ground truth poses file path> <initial poses file path> <optimised poses file path>
eg: python3 plots.py -gt gt.txt -init init.txt -opt opt.txt
```

## File Structure:
* `src/sam.cpp`: Main file                     
* `src/plots.py`: Visualize ground truth, inital pose and optimised trajectory.
* `src/sam_fixed_alloc.cpp`: Same as `sam.cpp` but with fixed memory allocation (will only work with increased stack size).

